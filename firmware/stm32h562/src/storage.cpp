// microSD: config + trim (/CFG.BIN), motion (/MOTIONS/SLOTn.LAM), blackbox (/BBX/LBBnnnnn.LBB)
// Dipanggil HANYA dari main loop. ISR kontrol hanya mendorong record ke ring buffer.
#include "board.h"
#include "app.h"
#include "ff.h"
#include "leumpang/lap.h"
#include "leumpang/files.h"
#include "leumpang/profile.h"

#include <initializer_list>
#include <stdio.h>
#include <string.h>

namespace {
FATFS    s_fs;
bool     s_ok = false;
FIL      s_bbx;
bool     s_bbxOpen = false;
uint32_t s_bbxBytes = 0, s_lastSyncMs = 0;
la::BlackboxRing* s_ring = nullptr;
uint8_t  s_buf[la::MOTION_MAX_BYTES];

constexpr uint32_t CFG_MAGIC = 0x4C415348;   // "HSAL"
constexpr uint16_t CFG_VER = 1;
struct CfgBlob { uint32_t magic; uint16_t ver, size; la::RuntimeConfig cfg; int16_t trim[2][la::SERVO_SLOTS]; uint16_t crc; };

void openBlackbox() {
    DIR d; FILINFO fi; unsigned next = 0;
    if (f_opendir(&d, "/BBX") == FR_OK) {
        while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
            unsigned idx = 0;
            if (sscanf(fi.fname, "LBB%05u", &idx) == 1 && idx >= next) next = idx + 1;
        }
        f_closedir(&d);
    }
    char path[24];
    snprintf(path, sizeof(path), "/BBX/LBB%05u.LBB", next % 100000);
    if (f_open(&s_bbx, path, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) { la_log_main("SD: gagal membuat %s\r\n", path); return; }
    uint8_t hdr[16]; UINT bw;
    la::blackboxHeader(hdr, HAL_GetTick());
    f_write(&s_bbx, hdr, sizeof(hdr), &bw);
    s_bbxOpen = true; s_bbxBytes = 0;
    la_log_main("SD: blackbox -> %s\r\n", path);
}
}  // namespace

// ---- layanan berkas LAP di atas FatFs (main loop saja) -------------------------
namespace {
la::FileStatus st(FRESULT r) {
    switch (r) {
        case FR_OK: return la::FS_OK;
        case FR_NO_FILE: case FR_NO_PATH: return la::FS_NOT_FOUND;
        case FR_EXIST: return la::FS_EXISTS;
        case FR_DENIED: case FR_WRITE_PROTECTED: return la::FS_DENIED;
        case FR_NOT_READY: case FR_NOT_ENABLED: case FR_NO_FILESYSTEM: return la::FS_NO_SD;
        default: return la::FS_IO;
    }
}
void abs(const char* rel, char* out) { snprintf(out, 104, "/%s", rel); }

class FatFsOps final : public la::FileOps {
public:
    bool available() override { return s_ok; }
    int list(const char* path, uint16_t start, la::FileEntry* out, int max, bool& more) override {
        char p[104]; abs(path, p);
        DIR d; FILINFO fi;
        const FRESULT r = f_opendir(&d, p);
        if (r != FR_OK) return -(int)st(r);
        int idx = 0, n = 0; more = false;
        while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
            if (idx++ < start) continue;
            if (n >= max) { more = true; break; }
            if (strlen(fi.fname) >= sizeof(out[n].name)) continue;
            strcpy(out[n].name, fi.fname);
            out[n].dir = (fi.fattrib & AM_DIR) != 0; out[n].size = (uint32_t)fi.fsize;
            ++n;
        }
        f_closedir(&d);
        return n;
    }
    int read(const char* path, uint32_t off, uint8_t* buf, size_t len) override {
        char p[104]; abs(path, p); FIL f; UINT br = 0;
        FRESULT r = f_open(&f, p, FA_READ);
        if (r != FR_OK) return -(int)st(r);
        r = f_lseek(&f, off);
        if (r == FR_OK) r = f_read(&f, buf, (UINT)len, &br);
        f_close(&f);
        return r == FR_OK ? (int)br : -(int)st(r);
    }
    la::FileStatus write(const char* path, uint32_t off, const uint8_t* d, size_t n, bool trunc) override {
        char p[104]; abs(path, p); FIL f; UINT bw = 0;
        FRESULT r = f_open(&f, p, trunc ? (FA_WRITE | FA_CREATE_ALWAYS) : (FA_WRITE | FA_OPEN_EXISTING));
        if (r != FR_OK) return st(r);
        r = f_lseek(&f, off);
        if (r == FR_OK && n) r = f_write(&f, d, (UINT)n, &bw);
        const FRESULT c = f_close(&f);
        if (r == FR_OK && bw != n) return la::FS_IO;
        return st(r != FR_OK ? r : c);
    }
    la::FileStatus remove(const char* path) override { if (!path[0]) return la::FS_DENIED; char p[104]; abs(path, p); return st(f_unlink(p)); }
    la::FileStatus mkdir(const char* path) override { char p[104]; abs(path, p); return st(f_mkdir(p)); }
    la::FileStatus rename(const char* a, const char* b) override { char pa[104], pb[104]; abs(a, pa); abs(b, pb); return st(f_rename(pa, pb)); }
    la::FileStatus stat(const char* path, uint32_t& size, bool& dir) override {
        if (!path[0]) { size = 0; dir = true; return s_ok ? la::FS_OK : la::FS_NO_SD; }
        char p[104]; abs(path, p); FILINFO fi;
        const FRESULT r = f_stat(p, &fi);
        if (r != FR_OK) return st(r);
        size = (uint32_t)fi.fsize; dir = (fi.fattrib & AM_DIR) != 0;
        return la::FS_OK;
    }
};
FatFsOps s_ops;
la::RobotProfile s_profile;
bool s_profileOk = false;
char s_json[8192];

void loadProfile() {
    FIL f; UINT n = 0;
    if (f_open(&f, "/config/robot.json", FA_READ) != FR_OK) {        // belum ada: tulis profil bawaan
        const size_t k = la::writeProfile(la::defaultProfile(), s_json, sizeof s_json);
        UINT bw = 0;
        if (k && f_open(&f, "/config/robot.json", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
            f_write(&f, s_json, (UINT)k, &bw); f_close(&f);
            la_log_main("SD: /config/robot.json dibuat dari profil bawaan KHR-3HV\r\n");
        }
        return;
    }
    f_read(&f, s_json, sizeof(s_json) - 1, &n);
    f_close(&f);
    char err[128] = "";
    if (n >= sizeof(s_json) - 1) { la_log_main("SD: robot.json terlalu besar, pakai profil bawaan\r\n"); return; }
    if (!la::parseProfile(s_json, n, s_profile, err, sizeof err)) { la_log_main("SD: robot.json TIDAK valid (%s), pakai bawaan\r\n", err); return; }
    s_profileOk = true;
    la_log_main("SD: profil '%s' (%s, %u servo)\r\n", s_profile.name, s_profile.humanoid() ? "humanoid-khr" : "generic",
                (unsigned)s_profile.presentCount());
}
}  // namespace

bool storage_ok() { return s_ok; }
la::FileOps* storage_files() { return s_ok ? &s_ops : nullptr; }
const la::RobotProfile* storage_profile() { return s_profileOk ? &s_profile : nullptr; }

bool storage_mount() {
    if (f_mount(&s_fs, "", 1) != FR_OK) { la_log_main("SD: tidak ada kartu / gagal mount\r\n"); return false; }
    s_ok = true;
    for (const char* d : {"/MOTIONS", "/BBX", "/config", "/scripts", "/poses", "/rec"}) f_mkdir(d);
    loadProfile();
    return true;
}

void storage_begin(la::MotionStore& store, la::BlackboxRing* ring) {
    if (!s_ok) return;
    for (int i = 0; i < la::MOTION_SLOTS; ++i) {
        char path[24]; FIL f; UINT br = 0;
        snprintf(path, sizeof(path), "/MOTIONS/SLOT%d.LAM", i);
        if (f_open(&f, path, FA_READ) != FR_OK) continue;
        f_read(&f, s_buf, sizeof(s_buf), &br);
        f_close(&f);
        la::MotionError e;
        if (store.load(i, s_buf, br, &e)) la_log_main("SD: motion slot %d '%s' dimuat\r\n", i, store.writable(i)->name);
        else la_log_main("SD: %s ditolak (%s)\r\n", path, la::motionErrorName(e));
    }
    if (ring) { s_ring = ring; openBlackbox(); }
}

void storage_service(uint32_t nowMs) {
    if (!s_ok || !s_bbxOpen || !s_ring) return;
    la::BlackboxRecord r; uint8_t rec[la::BLACKBOX_REC_SIZE]; UINT bw;
    int n = 0;
    while (n < 64 && s_ring->pop(r)) {                 // maks 4 KB per putaran main loop
        r.encode(rec);
        if (f_write(&s_bbx, rec, sizeof(rec), &bw) != FR_OK || bw != sizeof(rec)) {
            f_close(&s_bbx); s_bbxOpen = false; la_log_main("SD: tulis blackbox gagal\r\n"); return;
        }
        s_bbxBytes += sizeof(rec); ++n;
    }
    if (nowMs - s_lastSyncMs > 1000) { f_sync(&s_bbx); s_lastSyncMs = nowMs; }
    if (s_bbxBytes > 50u * 1024 * 1024) { f_close(&s_bbx); s_bbxOpen = false; openBlackbox(); }
}

void storage_save_motion(int slot, const uint8_t* d, size_t n) {
    if (!s_ok || slot < 0 || slot >= la::MOTION_SLOTS) return;
    char path[24]; FIL f; UINT bw = 0;
    snprintf(path, sizeof(path), "/MOTIONS/SLOT%d.LAM", slot);
    const bool ok = f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK && f_write(&f, d, (UINT)n, &bw) == FR_OK && bw == n;
    f_close(&f);
    la_log_main("SD: motion slot %d %s\r\n", slot, ok ? "disimpan" : "GAGAL disimpan");
}

bool storage_load_cfg(la::RuntimeConfig* cfg, int16_t trim[2][la::SERVO_SLOTS]) {
    if (!s_ok) return false;
    FIL f; UINT br = 0; CfgBlob b;
    if (f_open(&f, "/CFG.BIN", FA_READ) != FR_OK) return false;
    f_read(&f, &b, sizeof(b), &br); f_close(&f);
    if (br != sizeof(b) || b.magic != CFG_MAGIC || b.ver != CFG_VER || b.size != sizeof(b) ||
        la::lap::crc16((const uint8_t*)&b, offsetof(CfgBlob, crc)) != b.crc) return false;
    *cfg = b.cfg;
    memcpy(trim, b.trim, sizeof(b.trim));
    return true;
}

bool storage_save_cfg(const la::RuntimeConfig& cfg, const int16_t trim[2][la::SERVO_SLOTS]) {
    if (!s_ok) return false;
    CfgBlob b = {};
    b.magic = CFG_MAGIC; b.ver = CFG_VER; b.size = sizeof(b); b.cfg = cfg;
    memcpy(b.trim, trim, sizeof(b.trim));
    b.crc = la::lap::crc16((const uint8_t*)&b, offsetof(CfgBlob, crc));
    FIL f; UINT bw = 0;
    const bool ok = f_open(&f, "/CFG.BIN", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK && f_write(&f, &b, sizeof(b), &bw) == FR_OK && bw == sizeof(b);
    f_close(&f);
    return ok;
}
