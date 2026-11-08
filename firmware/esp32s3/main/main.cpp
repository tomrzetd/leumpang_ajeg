// =============================================================================
//  Leumpang Ajeg ("walking steadily") - firmware ESP32-S3 N16R8 (ESP-IDF)
//  app_main: NVS, logger, konsol serial, layanan persistensi, start semua task.
//
//  Peta task:
//    core1 : WalkCtrl (prio 20, 100 Hz)  <- satu-satunya task aplikasi di core1
//    core0 : IMU (17, 500 Hz) | httpd (5) | Console (3) | Log (2) | LocalUI (2) | Service (1)
//            + Wi-Fi/lwIP/esp_timer bawaan IDF
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
//  LOGGER NON-BLOCKING
// ============================================================================
namespace {
constexpr size_t LOG_LINE_MAX = 160;
struct LogLine { char text[LOG_LINE_MAX]; };
QueueHandle_t s_logQ = nullptr;

void loggerTask(void*) {
    LogLine ln;
    for (;;) {
        if (xQueueReceive(s_logQ, &ln, portMAX_DELAY) == pdTRUE) {
            fputs(ln.text, stdout);
            fflush(stdout);
            lap_forward_log(ln.text);
        }
    }
}

// Konsol: baca stdin non-blocking (UART0 atau USB-Serial-JTAG), teruskan ke task kontrol.
void consoleTask(void*) {
    const int fd = fileno(stdin);
    const int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0) fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    char buf[16];
    for (;;) {
        const ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) { for (ssize_t i = 0; i < n; ++i) control_push_key(buf[i]); }
        else vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}  // namespace

void logPrintf(const char* fmt, ...) {
    if (!s_logQ) return;
    LogLine ln;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ln.text, sizeof(ln.text), fmt, ap);
    va_end(ap);
    xQueueSend(s_logQ, &ln, 0);                  // tidak pernah memblokir; buang bila penuh
}

// ============================================================================
//  PERSISTENSI NVS TERTUNDA  [V4-NVS]
//  Tulis/erase flash mematikan cache KEDUA core (task kontrol di flash ikut berhenti
//  puluhan ms). V3 menulis NVS langsung dari handler HTTP - bisa terjadi saat jalan.
//  V4: perubahan diterapkan langsung ke RAM; disimpan hanya ketika mode aman.
// ============================================================================
namespace {
constexpr uint32_t PERSIST_MAGIC = 0x4B485234;   // "KHR4"
constexpr uint16_t PERSIST_VER   = 1;
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
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_logQ = xQueueCreate(32, sizeof(LogLine));
    s_wifiMux = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(loggerTask, "Log", STACK_LOGGER, nullptr, PRIO_LOGGER, nullptr, CORE_SYSTEM);
    logPrintf("\r\n=== Leumpang Ajeg (ESP32-S3) core %u.%u.%u - tekan '?' untuk bantuan ===\r\n", la::CORE_VER_MAJOR, la::CORE_VER_MINOR, la::CORE_VER_PATCH);

    RuntimeConfig cfg = runtime_config_default();
    cfg.imuSwap = IMU_SWAP_DEFAULT;
    int16_t trim[2][SERVO_SLOTS] = {};
    if (persist_load_cfg(&cfg, trim)) logPrintf("NVS: konfigurasi + trim dimuat\r\n");
    else                              logPrintf("NVS: konfigurasi default\r\n");

    control_init(cfg, trim);
    if (!imu_start())     logPrintf("GAGAL membuat task IMU!\r\n");
    if (!control_start()) logPrintf("GAGAL membuat task kontrol!\r\n");
    xTaskCreatePinnedToCore(consoleTask, "Console", STACK_CONSOLE, nullptr, PRIO_CONSOLE, nullptr, CORE_SYSTEM);
    web_start();
    lap_tcp_start();
    local_ui_start();
    xTaskCreatePinnedToCore(serviceTask, "Service", STACK_SERVICE, nullptr, PRIO_SERVICE, nullptr, CORE_SYSTEM);
}
