// =============================================================================
//  Leumpang Ajeg ("walking steadily") - firmware ESP-IDF untuk ESP32-S3 dan ESP32-P4
//  app_main: NVS, konsol+LAP serial, SD, layanan persistensi, start semua task.
//
//  Peta task:
//    core1 : WalkCtrl (prio 20, 100 Hz)  <- satu-satunya task aplikasi di core1
//    core0 : IMU (17, 500 Hz) | httpd (5) | LAP TCP (4) | Console+LAP serial (3)
//            | LocalUI (2) | Storage SD (1) | Service NVS (1) + Wi-Fi/lwIP bawaan IDF
// =============================================================================
#include "shared.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_rom_crc.h"

// ============================================================================
//  PERSISTENSI NVS TERTUNDA  [V4-NVS]
//  Tulis/erase flash mematikan cache KEDUA core (task kontrol di flash ikut berhenti
//  puluhan ms). V3 menulis NVS langsung dari handler HTTP - bisa terjadi saat jalan.
//  V4: perubahan diterapkan langsung ke RAM; disimpan hanya ketika mode aman.
// ============================================================================
namespace {
constexpr uint32_t PERSIST_MAGIC = 0x4B485234;   // "KHR4"
constexpr uint16_t PERSIST_VER   = 2;      // v2: RuntimeConfig + field gait (Fase 1)
struct PersistBlob {
    uint32_t magic;
    uint16_t version, size;
    RuntimeConfig cfg;
    int16_t  trim[2][SERVO_SLOTS];
    uint32_t crc;
};

volatile uint32_t s_savedGen = 0;
SemaphoreHandle_t s_wifiMux = nullptr;
bool  s_wifiPending = false;
char  s_wifiSsid[33], s_wifiPass[65];

uint32_t blobCrc(const PersistBlob& b) {
    return esp_rom_crc32_le(0, (const uint8_t*)&b, (uint32_t)offsetof(PersistBlob, crc));
}

bool saveCfg(const TelemetrySnapshot& t) {
    PersistBlob b{};
    b.magic = PERSIST_MAGIC; b.version = PERSIST_VER; b.size = sizeof(PersistBlob);
    b.cfg = t.cfg;
    memcpy(b.trim, t.trim, sizeof(b.trim));
    b.crc = blobCrc(b);
    nvs_handle_t h;
    if (nvs_open("bipedcfg", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_blob(h, "cfg", &b, sizeof(b)) == ESP_OK;
    ok = ok && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

bool saveWifi(const char* ssid, const char* pass) {
    nvs_handle_t h;
    if (nvs_open("bipedwifi", NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_str(h, "ssid", ssid) == ESP_OK;
    if (pass) ok = ok && nvs_set_str(h, "pass", pass) == ESP_OK;
    ok = ok && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

void serviceTask(void*) {
    int64_t lastFailUs = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        static TelemetrySnapshot t;
        t = telemetry_copy();
        if (t.seq == 0 || !mode_persist_safe(t.mode)) continue;
        const int64_t now = esp_timer_get_time();
        if (now - lastFailUs < 5000000) continue;   // jangan spam flash bila gagal

        if (t.cfgGen != s_savedGen) {
            if (saveCfg(t)) { s_savedGen = t.cfgGen; logPrintf("NVS: konfigurasi + trim disimpan (gen %lu)\r\n", (unsigned long)t.cfgGen); }
            else { lastFailUs = now; logPrintf("NVS: simpan konfigurasi GAGAL\r\n"); }
        }
        char ssid[33], pass[65];
        bool pend = false;
        xSemaphoreTake(s_wifiMux, portMAX_DELAY);
        if (s_wifiPending) { pend = true; strcpy(ssid, s_wifiSsid); strcpy(pass, s_wifiPass); }
        xSemaphoreGive(s_wifiMux);
        if (pend) {
            if (saveWifi(ssid, pass[0] ? pass : nullptr)) {
                xSemaphoreTake(s_wifiMux, portMAX_DELAY);
                if (!strcmp(ssid, s_wifiSsid) && !strcmp(pass, s_wifiPass)) s_wifiPending = false;
                xSemaphoreGive(s_wifiMux);
                logPrintf("NVS: kredensial Wi-Fi disimpan\r\n");
            } else { lastFailUs = now; logPrintf("NVS: simpan Wi-Fi GAGAL\r\n"); }
        }
    }
}
}  // namespace

bool persist_load_cfg(RuntimeConfig* cfg, int16_t trim[2][SERVO_SLOTS]) {
    nvs_handle_t h;
    if (nvs_open("bipedcfg", NVS_READONLY, &h) != ESP_OK) return false;
    PersistBlob b{};
    size_t len = sizeof(b);
    const esp_err_t e = nvs_get_blob(h, "cfg", &b, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(b) || b.magic != PERSIST_MAGIC || b.version != PERSIST_VER ||
        b.size != sizeof(b) || b.crc != blobCrc(b))
        return false;
    *cfg = b.cfg;
    memcpy(trim, b.trim, sizeof(b.trim));
    return true;
}

bool persist_load_wifi(char* ssid, size_t ssidLen, char* pass, size_t passLen) {
    nvs_handle_t h;
    if (nvs_open("bipedwifi", NVS_READONLY, &h) != ESP_OK) return false;
    size_t l1 = ssidLen, l2 = passLen;
    const bool ok = nvs_get_str(h, "ssid", ssid, &l1) == ESP_OK;
    if (nvs_get_str(h, "pass", pass, &l2) != ESP_OK) pass[0] = 0;
    nvs_close(h);
    return ok;
}

void persist_request_wifi(const char* ssid, const char* pass) {
    wifi_set_sta(ssid, pass);                    // langsung, hanya RAM
    xSemaphoreTake(s_wifiMux, portMAX_DELAY);
    strncpy(s_wifiSsid, ssid, sizeof(s_wifiSsid) - 1); s_wifiSsid[sizeof(s_wifiSsid) - 1] = 0;
    strncpy(s_wifiPass, pass ? pass : "", sizeof(s_wifiPass) - 1); s_wifiPass[sizeof(s_wifiPass) - 1] = 0;
    s_wifiPending = true;
    xSemaphoreGive(s_wifiMux);
}

uint32_t persist_saved_gen() { return s_savedGen; }

// ============================================================================
//  app_main
// ============================================================================
extern "C" void app_main(void) {
    console_early_init();                 // queue log siap sebelum modul lain mencetak
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    s_wifiMux = xSemaphoreCreateMutex();
    storage_mount();                      // SD + /config/robot.json; sebelum console (endpoint USB butuh layanan berkas)
    console_start();
    logPrintf("\r\n=== Leumpang Ajeg (%s) core %u.%u.%u - tekan '?' untuk bantuan ===\r\n",
              LA_BOARD_NAME, la::CORE_VER_MAJOR, la::CORE_VER_MINOR, la::CORE_VER_PATCH);

    RuntimeConfig cfg = runtime_config_default();
    cfg.imuSwap = IMU_SWAP_DEFAULT;
    int16_t trim[2][SERVO_SLOTS] = {};
    if (persist_load_cfg(&cfg, trim)) logPrintf("NVS: konfigurasi + trim dimuat\r\n");
    else                              logPrintf("NVS: konfigurasi default\r\n");

    control_init(cfg, trim, storage_profile());
    storage_init();                       // SD: motion -> MotionStore, blackbox -> ring kontrol
    if (!imu_start())     logPrintf("GAGAL membuat task IMU!\r\n");
    if (!control_start()) logPrintf("GAGAL membuat task kontrol!\r\n");
    web_start();
    lap_tcp_start();
    radio_start();                        // Leumpang Radio (stik PS4) di UART, bila board punya UART bebas
    local_ui_start();
    xTaskCreatePinnedToCore(serviceTask, "Service", STACK_SERVICE, nullptr, PRIO_SERVICE, nullptr, CORE_SYSTEM);
}
