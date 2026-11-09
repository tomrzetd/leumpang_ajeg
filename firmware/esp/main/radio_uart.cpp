// =============================================================================
//  Leumpang Radio (Fase 4B): UART ke board radio (ESP32 klasik / Pico 2 W + Bluepad32).
//  Endpoint LAP kedua (independen dari TCP & konsol): radio mengirim PAD 50 Hz,
//  robot membalas PAD_FEEDBACK (lightbar/getar). Semua pesan LAP lain juga diterima,
//  sehingga board radio kelak bisa menjadi jembatan Wi-Fi.
// =============================================================================
#include "shared.h"

#if LA_RADIO_UART >= 0
#include <string.h>
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "leumpang/lap_endpoint.h"

namespace {
constexpr uart_port_t RADIO_PORT = (uart_port_t)LA_RADIO_UART;
constexpr int RADIO_BAUD = 460800;
la::lap::LapEndpoint s_ep;
volatile uint32_t s_lastPadMs = 0;

uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }
void epWrite(void*, const uint8_t* d, size_t n) { uart_write_bytes(RADIO_PORT, d, n); }
bool epSubmit(void*, const la::Command& c) {
    if (c.type == la::CmdType::PAD) s_lastPadMs = nowMs();
    return control_queue_cmd(c);
}
void epSnapshot(void*, la::Telemetry& t) { t = telemetry_copy(); }
uint32_t epSaved(void*) { return persist_saved_gen(); }

void radioTask(void*) {
    uart_config_t c = {};
    c.baud_rate = RADIO_BAUD;
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    if (uart_driver_install(RADIO_PORT, 1024, 2048, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(RADIO_PORT, &c) != ESP_OK ||
        uart_set_pin(RADIO_PORT, LA_RADIO_TX, LA_RADIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        logPrintf("RADIO: UART%d gagal\r\n", (int)LA_RADIO_UART);
        vTaskDelete(nullptr);
        return;
    }
    la::lap::Hello self;
    self.role = la::lap::ROLE_ROBOT; self.board = LA_BOARD_ID;
    self.fwMajor = la::CORE_VER_MAJOR; self.fwMinor = la::CORE_VER_MINOR; self.fwPatch = la::CORE_VER_PATCH;
    self.caps = lap_self_caps();
    strncpy(self.name, LA_BOARD_NAME, 16);
    la::lap::LapEndpoint::Host h;
    h.write = epWrite; h.submit = epSubmit; h.snapshot = epSnapshot; h.savedGen = epSaved;
    h.files = storage_files();
    h.reboot = [](void*) { esp_restart(); };
    h.motions = &control_motions();
    h.motionSaved = [](void*, int slot, const uint8_t* d, size_t n) { storage_save_motion(slot, d, n); };
    s_ep.init(h, self);
    logPrintf("RADIO: UART%d TX=%d RX=%d %d baud siap\r\n", (int)LA_RADIO_UART, LA_RADIO_TX, LA_RADIO_RX, RADIO_BAUD);
    uint8_t buf[128];
    for (;;) {
        const int n = uart_read_bytes(RADIO_PORT, buf, sizeof buf, pdMS_TO_TICKS(5));
        if (n > 0) s_ep.onBytes(buf, (size_t)n, nowMs());
        s_ep.poll(nowMs());
    }
}
}  // namespace

void radio_start() {
    if (xTaskCreatePinnedToCore(radioTask, "RADIO", STACK_LAP, nullptr, PRIO_LAP, nullptr, CORE_SYSTEM) != pdPASS)
        logPrintf("GAGAL membuat task RADIO\r\n");
}
bool radio_active() { return s_lastPadMs && nowMs() - s_lastPadMs < 1000; }

#else
void radio_start() {}
bool radio_active() { return false; }
#endif
