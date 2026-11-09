// =============================================================================
//  Konsol + link LAP serial (core0)
//  Port konsol board (UART0 atau USB-Serial-JTAG, sesuai CONFIG_ESP_CONSOLE_*)
//  dipakai bersama: manusia mengetik tombol, HP/PC mengirim frame LAP biner.
//  Satu-satunya task yang menulis ke port -> frame tidak pernah terpotong log.
// =============================================================================
#include "esp_system.h"
#include "shared.h"
#include "leumpang/motion.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "leumpang/lap_serial.h"

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#else
#include "driver/uart.h"
#endif

namespace {

constexpr size_t LOG_LINE_MAX = 160;
struct LogLine { char text[LOG_LINE_MAX]; };
QueueHandle_t s_logQ = nullptr;
la::lap::SerialLink s_link;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
bool portInit() {
    usb_serial_jtag_driver_config_t c = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    c.rx_buffer_size = 1024; c.tx_buffer_size = 4096;
    return usb_serial_jtag_driver_install(&c) == ESP_OK;
}
int  portRead(uint8_t* b, size_t n, TickType_t t) { return usb_serial_jtag_read_bytes(b, (uint32_t)n, t); }
void portWrite(const uint8_t* d, size_t n) { usb_serial_jtag_write_bytes(d, n, pdMS_TO_TICKS(20)); }
#else
#ifndef CONFIG_ESP_CONSOLE_UART_NUM
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#endif
const uart_port_t CON_UART = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
bool portInit() { return uart_driver_install(CON_UART, 1024, 4096, 0, nullptr, 0) == ESP_OK; }
int  portRead(uint8_t* b, size_t n, TickType_t t) { return uart_read_bytes(CON_UART, b, (uint32_t)n, t); }
void portWrite(const uint8_t* d, size_t n) { uart_write_bytes(CON_UART, d, n); }
#endif

void ioWrite(void*, const uint8_t* d, size_t n) { portWrite(d, n); }
void ioKey(void*, char c) { control_push_key(c); }
bool epSubmit(void*, const la::Command& c) { return control_queue_cmd(c); }
void epSnapshot(void*, la::Telemetry& t) { t = telemetry_copy(); }
uint32_t epSaved(void*) { return persist_saved_gen(); }
void epMotionSaved(void*, int slot, const uint8_t* d, size_t n) { storage_save_motion(slot, d, n); }

uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

void consoleTask(void*) {
    const bool ok = portInit();
    la::lap::Hello self;
    self.role = la::lap::ROLE_ROBOT; self.board = LA_BOARD_ID;
    self.fwMajor = la::CORE_VER_MAJOR; self.fwMinor = la::CORE_VER_MINOR; self.fwPatch = la::CORE_VER_PATCH;
    self.caps = la::lap::CAP_WIFI | la::lap::CAP_LCD | la::lap::CAP_MOTION | la::lap::CAP_GAIT_INTENT | la::lap::CAP_LIVE_POSE | la::lap::CAP_PERCEPTION | la::lap::CAP_SERVO_TOOL |
                (storage_files() ? (uint32_t)(la::lap::CAP_FILES | la::lap::CAP_PROFILE) : 0u) |
                la::lap::CAP_USB_CDC | (storage_sd_ok() ? (uint32_t)la::lap::CAP_SD : 0u);
    strncpy(self.name, LA_BOARD_NAME, 16);
    la::lap::LapEndpoint::Host h;
    h.submit = epSubmit; h.snapshot = epSnapshot; h.savedGen = epSaved;
    h.files = storage_files();
    h.reboot = [](void*) { esp_restart(); };
    h.motions = &control_motions(); h.motionSaved = epMotionSaved;
    la::lap::SerialLink::Io io; io.write = ioWrite; io.key = ioKey;
    s_link.init(io, h, self);
    if (!ok) esp_rom_printf("console: driver port gagal\n");

    uint8_t buf[128];
    bool wasLap = false;
    for (;;) {
        const int n = portRead(buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n > 0) s_link.onRx(buf, (size_t)n, nowMs());
        LogLine ln;
        for (int i = 0; i < 8 && xQueueReceive(s_logQ, &ln, 0) == pdTRUE; ++i) {
            s_link.logLine(ln.text);
            lap_forward_log(ln.text);
        }
        s_link.poll(nowMs());
        // Log ESP-IDF (ESP_LOGx) menulis langsung ke port; matikan selama sesi LAP biner.
        if (s_link.lapMode() != wasLap) {
            wasLap = s_link.lapMode();
            esp_log_level_set("*", wasLap ? ESP_LOG_NONE : ESP_LOG_INFO);
        }
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
    xQueueSend(s_logQ, &ln, 0);                  // tidak pernah memblokir
}

void console_early_init() { s_logQ = xQueueCreate(32, sizeof(LogLine)); }

void console_start() {
    if (xTaskCreatePinnedToCore(consoleTask, "Console", STACK_CONSOLE, nullptr, PRIO_CONSOLE, nullptr, CORE_SYSTEM) != pdPASS)
        esp_rom_printf("GAGAL membuat task konsol\n");
}
