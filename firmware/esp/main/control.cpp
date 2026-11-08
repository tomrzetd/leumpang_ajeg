// =============================================================================
//  Firmware ESP (S3/P4) - HAL kontrol (core1)
//  Logika robot: la::Controller (core/). File ini hanya:
//   * EspIcsBus  : UART 1,25 Mbps 8E1 + pin arah EN (implementasi la::ServoBus)
//   * QueueInbox : queue FreeRTOS -> la::Inbox
//   * task 100 Hz, boot sync, publikasi telemetry, LED/pin ukur
// =============================================================================
#include "shared.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "leumpang/controller.h"

namespace {

// ============================================================================
//  Bus ICS
// ============================================================================
const uart_port_t ICS_PORT[2] = { UART_NUM_1, UART_NUM_2 };
const int ICS_TXP[2] = { BUS_R_TX, BUS_L_TX };
const int ICS_RXP[2] = { BUS_R_RX, BUS_L_RX };
[[maybe_unused]] const int ICS_ENP[2] = { BUS_R_EN, BUS_L_EN };

inline void icsDirTx(uint8_t b, bool tx) {
#if ICS_DIR_MODE == 1
    gpio_set_level((gpio_num_t)ICS_ENP[b], tx ? ICS_EN_TX_LEVEL : !ICS_EN_TX_LEVEL);
#else
    (void)b; (void)tx;
#endif
}

class EspIcsBus : public la::ServoBus {
public:
    bool init() {
        bool ok = true;
        for (uint8_t b = 0; b < 2; ++b) {
            const uart_port_t p = ICS_PORT[b];
            uart_config_t c = {};
            c.baud_rate = (int)la::ICS_BAUD;
            c.data_bits = UART_DATA_8_BITS;
            c.parity    = UART_PARITY_EVEN;
            c.stop_bits = UART_STOP_BITS_1;
            c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            c.source_clk = UART_SCLK_DEFAULT;
            ok &= uart_driver_install(p, 256, 0, 0, nullptr, 0) == ESP_OK;
            ok &= uart_param_config(p, &c) == ESP_OK;
#if ICS_DIR_MODE == 2
            ok &= uart_set_pin(p, ICS_TXP[b], ICS_RXP[b], ICS_ENP[b], UART_PIN_NO_CHANGE) == ESP_OK;
            ok &= uart_set_mode(p, UART_MODE_RS485_HALF_DUPLEX) == ESP_OK;
#if ICS_RS485_INVERT_RTS
            ok &= uart_set_line_inverse(p, UART_SIGNAL_RTS_INV) == ESP_OK;
#endif
#else
            ok &= uart_set_pin(p, ICS_TXP[b], ICS_RXP[b], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK;
#endif
            gpio_set_pull_mode((gpio_num_t)ICS_RXP[b], GPIO_PULLUP_ONLY);
            uart_set_rx_full_threshold(p, 3);
            uart_set_rx_timeout(p, 2);
#if ICS_DIR_MODE == 1
            gpio_config_t io = {};
            io.pin_bit_mask = 1ULL << ICS_ENP[b];
            io.mode = GPIO_MODE_OUTPUT;
            ok &= gpio_config(&io) == ESP_OK;
            icsDirTx(b, false);
#endif
        }
        return ok;
    }

    // Kirim R+L bersamaan, tunggu TX selesai (polling), pindah ke RX, early-exit saat reply valid.
    void exchange(la::IcsExchange& x) override {
        for (uint8_t b = 0; b < 2; ++b) { uart_flush_input(ICS_PORT[b]); icsDirTx(b, true); x.got[b] = false; }
        for (uint8_t b = 0; b < 2; ++b) uart_write_bytes(ICS_PORT[b], x.tx[b], 3);
        for (uint8_t b = 0; b < 2; ++b) { uart_wait_tx_idle_polling(ICS_PORT[b]); icsDirTx(b, false); }

        uint8_t rx[2][12];
        uint8_t n[2] = {0, 0};
        const int64_t deadline = esp_timer_get_time() + ICS_REPLY_TIMEOUT_US;
        while (!(x.got[0] && x.got[1]) && esp_timer_get_time() < deadline) {
            for (uint8_t b = 0; b < 2; ++b) {
                if (x.got[b]) continue;
                size_t avail = 0;
                if (uart_get_buffered_data_len(ICS_PORT[b], &avail) != ESP_OK || avail == 0) continue;
                const size_t room = sizeof(rx[b]) - n[b];
                if (room == 0) continue;
                const int r = uart_read_bytes(ICS_PORT[b], rx[b] + n[b], (uint32_t)(avail < room ? avail : room), 0);
                if (r > 0) n[b] = (uint8_t)(n[b] + r);
                x.got[b] = la::ics::parseReply(rx[b], n[b], x.tx[b], &x.pos[b]);
            }
        }
    }
};

// ============================================================================
//  Inbox & platform
// ============================================================================
QueueHandle_t g_cmdQ = nullptr;
QueueHandle_t g_keyQ = nullptr;

class QueueInbox : public la::Inbox {
public:
    bool nextKey(char& c) override { return xQueueReceive(g_keyQ, &c, 0) == pdTRUE; }
    bool nextCommand(la::Command& c) override { return xQueueReceive(g_cmdQ, &c, 0) == pdTRUE; }
};

int64_t platNow(void*) { return esp_timer_get_time(); }
void platLog(void*, const char* line) { logPrintf("%s", line); }

la::Controller   g_ctrl;
EspIcsBus        g_bus;
QueueInbox       g_inbox;
TaskHandle_t     g_task = nullptr;
la::Telemetry    g_tele{};
la::Telemetry    g_teleWork{};
portMUX_TYPE     g_teleMux = portMUX_INITIALIZER_UNLOCKED;

la::ImuInput readImu() {
    la::ImuInput in{};
    in.sensorOk = imu_is_ok();
    in.rateHz = imu_rate_hz();
    ImuSample s;
    if (imu_get_latest(&s)) { in.hasSample = true; in.roll = s.roll; in.pitch = s.pitch; in.tUs = s.tUs; in.seq = s.seq; }
    return in;
}

void publish(bool force) {
    static int64_t last = 0;
    const int64_t now = esp_timer_get_time();
    if (!force && now - last < (int64_t)TELE_PUBLISH_MS * 1000) return;
    last = now;
    g_ctrl.snapshot(g_teleWork);
    g_teleWork.ctrlStackFree = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);
    portENTER_CRITICAL(&g_teleMux);
    g_tele = g_teleWork;
    portEXIT_CRITICAL(&g_teleMux);
}

void heartbeat() {
    const uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    const bool on = (g_ctrl.mode() == Mode::FALLEN) ? ((ms / 100) & 1) : ((ms / 500) & 1);
    gpio_set_level((gpio_num_t)PIN_LED1, on ? 1 : 0);
}

void controlTask(void*) {
    if (!g_bus.init()) logPrintf("GAGAL inisialisasi UART ICS!\r\n");   // ISR UART ikut di core1
    for (int i = 0; i < la::BOOT_SYNC_CYCLES; ++i) { g_ctrl.bootFreeCycle(g_bus); vTaskDelay(pdMS_TO_TICKS(10)); }
    g_ctrl.bootFinish(BOOT_SYNC_FROM_FEEDBACK != 0);
    publish(true);

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(la::CONTROL_PERIOD_MS);
    const int64_t periodUs = (int64_t)la::CONTROL_PERIOD_MS * 1000;
    for (;;) {
        const int64_t t0 = esp_timer_get_time();
        gpio_set_level((gpio_num_t)PIN_T2, 1);
        g_ctrl.step(readImu(), g_bus, g_inbox);
        heartbeat();
        gpio_set_level((gpio_num_t)PIN_T2, 0);
        publish(false);
        if (esp_timer_get_time() - t0 >= periodUs) {
            g_ctrl.noteOverrun();
            lastWake = xTaskGetTickCount();
            vTaskDelay(1);
        } else {
            xTaskDelayUntil(&lastWake, period);
        }
    }
}

}  // namespace

// ============================================================================
//  API
// ============================================================================
bool control_queue_cmd(const RobotCommand& c) { return g_cmdQ && xQueueSend(g_cmdQ, &c, 0) == pdTRUE; }
la::MotionStore& control_motions() { return g_ctrl.motions(); }
void control_set_blackbox(la::BlackboxRing* ring) { g_ctrl.setBlackbox(ring); }
bool control_push_key(char c)                 { return g_keyQ && xQueueSend(g_keyQ, &c, 0) == pdTRUE; }

TelemetrySnapshot telemetry_copy() {
    TelemetrySnapshot t;
    portENTER_CRITICAL(&g_teleMux);
    t = g_tele;
    portEXIT_CRITICAL(&g_teleMux);
    return t;
}

void control_init(const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS]) {
    la::Platform p{ platNow, platLog, nullptr };
    la::Controller::Options o;
    o.autoCalibrate = IMU_AUTO_CALIBRATE != 0;
    o.autoRearm     = AUTO_REARM_AFTER_FALL != 0;
    o.swcontLegacy  = SWCONT_LEGACY != 0;
    g_ctrl.init(p, cfg, trim, o);

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LED1) | (1ULL << PIN_T2);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)PIN_LED1, 0);
    gpio_set_level((gpio_num_t)PIN_T2, 0);

    g_cmdQ = xQueueCreate(32, sizeof(la::Command));
    g_keyQ = xQueueCreate(64, sizeof(char));
}

bool control_start() {
    if (!g_cmdQ || !g_keyQ) { logPrintf("GAGAL membuat queue kontrol!\r\n"); return false; }
    return xTaskCreatePinnedToCore(controlTask, "WalkCtrl", STACK_CONTROL, nullptr, PRIO_CONTROL,
                                   &g_task, CORE_CONTROL) == pdPASS;
}
