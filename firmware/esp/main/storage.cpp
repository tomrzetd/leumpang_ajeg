// =============================================================================
//  SD card (core0, prioritas rendah)
//   /sd/motions/slotN.lam   template gerak (dimuat ke MotionStore saat boot)
//   /sd/bbx/LBBnnnnn.lbb    blackbox 100 Hz (file baru tiap boot / 50 MB)
//  SDMMC tidak mematikan cache flash, jadi aman ditulis saat robot berjalan;
//  kontrol hanya mendorong ke ring buffer dan tidak pernah menunggu kartu.
// =============================================================================
#include <initializer_list>
#include "shared.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "soc/soc_caps.h"
#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif
#include "leumpang/blackbox.h"
#include "leumpang/motion.h"

namespace {

constexpr char MOUNT[] = "/sd";
bool             s_ok = false;
sdmmc_card_t*    s_card = nullptr;
la::BlackboxRing s_ring;
FILE*            s_bbx = nullptr;
size_t           s_bbxBytes = 0;
uint32_t         s_bbxIndex = 0;

#if LA_SD_ENABLE
bool mountOnce(int pwrLevel) {
    if (SD_PWR_EN >= 0) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << (SD_PWR_EN & 63);
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        gpio_set_level((gpio_num_t)SD_PWR_EN, 1 - pwrLevel);   // mati dulu
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level((gpio_num_t)SD_PWR_EN, pwrLevel);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32P4
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;      // IOMUX: 40 MHz
#else
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;        // GPIO matrix + kabel modul: 20 MHz lebih aman
#endif
#if SOC_SDMMC_IO_POWER_EXTERNAL
    static sd_pwr_ctrl_handle_t s_pwr = nullptr;
    if (SD_LDO_CHAN >= 0 && !s_pwr) {
        sd_pwr_ctrl_ldo_config_t lc = {};
        lc.ldo_chan_id = SD_LDO_CHAN;
        if (sd_pwr_ctrl_new_on_chip_ldo(&lc, &s_pwr) != ESP_OK) s_pwr = nullptr;
    }
    if (s_pwr) host.pwr_ctrl_handle = s_pwr;
#endif
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
#if CONFIG_IDF_TARGET_ESP32P4
    host.slot = SDMMC_HOST_SLOT_0;        // IOMUX 39..44
#else
    slot.clk = (gpio_num_t)SD_CLK; slot.cmd = (gpio_num_t)SD_CMD; slot.d0 = (gpio_num_t)SD_D0;
    slot.d1 = (gpio_num_t)SD_D1; slot.d2 = (gpio_num_t)SD_D2; slot.d3 = (gpio_num_t)SD_D3;
#endif
    slot.width = LA_SD_4BIT ? 4 : 1;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_sdmmc_mount_config_t mc = {};
    mc.format_if_mount_failed = false;
    mc.max_files = 6;
    mc.allocation_unit_size = 16 * 1024;
    return esp_vfs_fat_sdmmc_mount(MOUNT, &host, &slot, &mc, &s_card) == ESP_OK;
}
#endif

void ensureDir(const char* p) { struct stat st; if (stat(p, &st) != 0) mkdir(p, 0775); }

void loadMotions() {
    la::MotionStore& store = control_motions();
    char path[48];
    uint8_t* buf = (uint8_t*)malloc(la::MOTION_MAX_BYTES);
    if (!buf) return;
    for (int i = 0; i < la::MOTION_SLOTS; ++i) {
        snprintf(path, sizeof(path), "%s/motions/slot%d.lam", MOUNT, i);
        FILE* f = fopen(path, "rb");
        if (!f) continue;
        const size_t n = fread(buf, 1, la::MOTION_MAX_BYTES, f);
        fclose(f);
        la::MotionError e;
        if (store.load(i, buf, n, &e)) logPrintf("SD: motion slot %d '%s' dimuat\r\n", i, store.writable(i)->name);
        else logPrintf("SD: %s ditolak (%s)\r\n", path, la::motionErrorName(e));
    }
    free(buf);
}

void openBlackbox() {
    char path[48];
    // cari indeks berikutnya
    DIR* d = opendir("/sd/bbx");
    if (d) {
        struct dirent* e;
        while ((e = readdir(d)) != nullptr) {
            unsigned idx = 0;
            if (sscanf(e->d_name, "LBB%05u", &idx) == 1 && idx >= s_bbxIndex) s_bbxIndex = idx + 1;
        }
        closedir(d);
    }
    snprintf(path, sizeof(path), "/sd/bbx/LBB%05u.LBB", (unsigned)(s_bbxIndex++ % 100000));
    s_bbx = fopen(path, "wb");
    s_bbxBytes = 0;
    if (!s_bbx) { logPrintf("SD: gagal membuat %s\r\n", path); return; }
    setvbuf(s_bbx, nullptr, _IOFBF, 8192);
    uint8_t hdr[16];
    la::blackboxHeader(hdr, (uint32_t)(esp_timer_get_time() / 1000));
    fwrite(hdr, 1, sizeof(hdr), s_bbx);
    logPrintf("SD: blackbox -> %s\r\n", path);
}

void storageTask(void*) {
    la::BlackboxRecord r;
    uint8_t rec[la::BLACKBOX_REC_SIZE];
    int64_t lastFlush = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_bbx) continue;
        while (s_ring.pop(r)) {
            r.encode(rec);
            if (fwrite(rec, 1, sizeof(rec), s_bbx) != sizeof(rec)) { fclose(s_bbx); s_bbx = nullptr; break; }
            s_bbxBytes += sizeof(rec);
        }
        if (!s_bbx) { logPrintf("SD: tulis blackbox gagal, berhenti merekam\r\n"); continue; }
        const int64_t now = esp_timer_get_time();
        if (now - lastFlush > 1000000) { fflush(s_bbx); fsync(fileno(s_bbx)); lastFlush = now; }
        if (s_bbxBytes > BBX_ROTATE_BYTES) { fclose(s_bbx); s_bbx = nullptr; openBlackbox(); }
    }
}

}  // namespace

bool storage_sd_ok() { return s_ok; }

static la::RobotProfile s_profile;
static bool s_profileOk = false;
static la::FileOps* s_files = nullptr;

const la::RobotProfile* storage_profile() { return s_profileOk ? &s_profile : nullptr; }
la::FileOps* storage_files() { return s_files; }

static void loadProfile() {
    const char* path = "/sd/config/robot.json";
    static char buf[8192];
    FILE* f = fopen(path, "rb");
    if (!f) {                                                   // belum ada: tulis bawaan sebagai titik awal
        const size_t n = la::writeProfile(la::defaultProfile(), buf, sizeof buf);
        FILE* w = fopen(path, "wb");
        if (w && n) { fwrite(buf, 1, n, w); fclose(w); logPrintf("SD: %s dibuat dari profil bawaan KHR-3HV\r\n", path); }
        else if (w) fclose(w);
        return;
    }
    const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    char err[128] = "";
    if (n >= sizeof(buf) - 1) { logPrintf("SD: %s terlalu besar, pakai profil bawaan\r\n", path); return; }
    if (!la::parseProfile(buf, n, s_profile, err, sizeof err)) {
        logPrintf("SD: profil %s TIDAK valid (%s) - pakai profil bawaan\r\n", path, err);
        return;
    }
    s_profileOk = true;
    logPrintf("SD: profil '%s' (%s, %u servo)\r\n", s_profile.name, s_profile.humanoid() ? "humanoid-khr" : "generic",
              (unsigned)s_profile.presentCount());
}

void storage_mount() {
#if LA_SD_ENABLE
    s_ok = mountOnce(0);
    if (!s_ok && SD_PWR_EN >= 0) s_ok = mountOnce(1);          // polaritas EN belum pasti -> coba keduanya
    if (!s_ok) { logPrintf("SD: tidak ada kartu / gagal mount (profil bawaan; blackbox, motion & berkas nonaktif)\r\n"); return; }
    logPrintf("SD: %s %llu MB terpasang di /sd\r\n", s_card->cid.name,
              (unsigned long long)((uint64_t)s_card->csd.capacity * s_card->csd.sector_size / (1024 * 1024)));
    for (const char* d : {"/sd/motions", "/sd/bbx", "/sd/config", "/sd/scripts", "/sd/poses", "/sd/rec"}) ensureDir(d);
    loadProfile();
    s_files = la::posixFileOps(MOUNT);
#endif
}

void storage_init() {
#if LA_SD_ENABLE
    if (!s_ok) return;
    loadMotions();
#if LA_BLACKBOX_DEFAULT
    openBlackbox();
    if (s_bbx) control_set_blackbox(&s_ring);
#endif
    xTaskCreatePinnedToCore(storageTask, "Storage", STACK_STORAGE, nullptr, PRIO_STORAGE, nullptr, CORE_SYSTEM);
#endif
}

void storage_save_motion(int slot, const uint8_t* d, size_t n) {
    if (!s_ok || slot < 0 || slot >= la::MOTION_SLOTS) return;
    char path[48];
    snprintf(path, sizeof(path), "%s/motions/slot%d.lam", MOUNT, slot);
    FILE* f = fopen(path, "wb");
    if (!f) { logPrintf("SD: gagal menyimpan %s\r\n", path); return; }
    const bool ok = fwrite(d, 1, n, f) == n;
    fclose(f);
    logPrintf("SD: motion slot %d %s\r\n", slot, ok ? "disimpan" : "GAGAL disimpan");
}
