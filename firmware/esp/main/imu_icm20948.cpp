// =============================================================================
//  IMU task (core0): transport I2C ESP-IDF (i2c_master) untuk la::Icm20948
//  Register & fusi Mahony ada di core/src/icm20948.cpp (dipakai juga STM32).
//  Transaksi I2C berbasis interrupt -> task blocking, CPU bebas; core1 tidak menunggu I2C.
// =============================================================================
#include "shared.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "leumpang/icm20948.h"

namespace {

constexpr int I2C_TIMEOUT_MS = 5;

i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_dev = nullptr;
TaskHandle_t s_task = nullptr;
la::Icm20948 s_icm;

ImuSample    s_latest{};
bool         s_have = false;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     s_ok = false;
volatile uint16_t s_rateHz = 0;

bool opWrite(void*, uint8_t reg, uint8_t val) {
    const uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, I2C_TIMEOUT_MS) == ESP_OK;
}
bool opRead(void*, uint8_t reg, uint8_t* d, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, d, n, I2C_TIMEOUT_MS) == ESP_OK;
}
void opDelay(void*, uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }

bool busInit() {
    if (s_bus) return true;
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = (gpio_num_t)IMU_SDA;
    bc.scl_io_num = (gpio_num_t)IMU_SCL;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    bc.flags.enable_internal_pullup = true;      // tetap pasang pull-up eksternal 2k2..4k7
    return i2c_new_master_bus(&bc, &s_bus) == ESP_OK;
}

bool chipInit() {
    if (!busInit()) { logPrintf("IMU: i2c_new_master_bus gagal\r\n"); return false; }
    if (s_dev) { i2c_master_bus_rm_device(s_dev); s_dev = nullptr; }
    uint16_t addr = 0;
    if (i2c_master_probe(s_bus, 0x69, 20) == ESP_OK) addr = 0x69;
    else if (i2c_master_probe(s_bus, 0x68, 20) == ESP_OK) addr = 0x68;
    else { logPrintf("IMU: ICM-20948 tidak menjawab di 0x68/0x69 (SDA=%d SCL=%d)\r\n", IMU_SDA, IMU_SCL); return false; }
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = addr;
    dc.scl_speed_hz = IMU_I2C_HZ;
    if (i2c_master_bus_add_device(s_bus, &dc, &s_dev) != ESP_OK) return false;

    la::I2cOps ops;
    ops.write = opWrite; ops.read = opRead; ops.delayMs = opDelay;
    s_icm.params.kp2 = IMU_MAHONY_KP2; s_icm.params.ki2 = IMU_MAHONY_KI2;
    s_icm.params.gateLo = IMU_ACC_GATE_LO; s_icm.params.gateHi = IMU_ACC_GATE_HI;
    const la::Icm20948::Err e = s_icm.init(ops, ICM_INT_GPIO >= 0);
    if (e == la::Icm20948::Err::WHO_AM_I) { logPrintf("IMU: WHO_AM_I=0x%02X (harus 0xEA)\r\n", s_icm.whoAmI()); return false; }
    if (e != la::Icm20948::Err::OK) { logPrintf("IMU: konfigurasi register gagal\r\n"); return false; }
    logPrintf("IMU: ICM-20948 OK @0x%02X, raw 562 Hz, fusi Mahony %u Hz\r\n",
              (unsigned)addr, (unsigned)(1000000u / IMU_PERIOD_US));
    return true;
}

#if ICM_INT_GPIO >= 0
void IRAM_ATTR intIsr(void*) {
    BaseType_t woken = pdFALSE;
    if (s_task) vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#endif

void imuTask(void*) {
    for (;;) {
        s_ok = false;
        while (!chipInit()) vTaskDelay(pdMS_TO_TICKS(1000));
        if (s_icm.calibrateGyroBias()) {
            float x, y, z; s_icm.biasDps(x, y, z);
            logPrintf("IMU: bias gyro %.2f %.2f %.2f deg/s\r\n", (double)x, (double)y, (double)z);
        } else {
            logPrintf("IMU: robot bergerak saat kalibrasi bias gyro, bias = 0\r\n");
        }
        la::ImuRaw r;
        while (!s_icm.read(r)) vTaskDelay(pdMS_TO_TICKS(5));
        s_icm.fusionInit(r);
        s_ok = true;

        int64_t lastUs = esp_timer_get_time(), rateT0 = lastUs;
        uint32_t rateN = 0, seq = 0;
        uint8_t errs = 0;
        TickType_t lastWake = xTaskGetTickCount();
        const TickType_t per = pdMS_TO_TICKS(IMU_PERIOD_US / 1000) ? pdMS_TO_TICKS(IMU_PERIOD_US / 1000) : 1;
        for (;;) {
#if ICM_INT_GPIO >= 0
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            (void)lastWake; (void)per;
#else
            xTaskDelayUntil(&lastWake, per);
#endif
            if (!s_icm.read(r)) {
                if (++errs >= 20) {
                    logPrintf("IMU: I2C error beruntun, reset bus + init ulang\r\n");
                    s_ok = false;
                    i2c_master_bus_reset(s_bus);
                    break;
                }
                continue;
            }
            errs = 0;
            const int64_t now = esp_timer_get_time();
            const float dt = clampf((float)(now - lastUs) * 1e-6f, 1e-4f, 0.02f);
            lastUs = now;
            s_icm.fusionUpdate(r, dt);

            ImuSample s;
            s_icm.quat(s.qw, s.qx, s.qy, s.qz);
            s.roll = s_icm.roll(); s.pitch = s_icm.pitch();
            s_icm.corrected(r, s.gx, s.gy, s.gz);
            s.tUs = now; s.seq = ++seq;
            portENTER_CRITICAL(&s_mux);
            s_latest = s; s_have = true;
            portEXIT_CRITICAL(&s_mux);

            ++rateN;
            if (now - rateT0 >= 1000000) { s_rateHz = (uint16_t)rateN; rateN = 0; rateT0 = now; }
        }
    }
}

}  // namespace

bool imu_start() {
#if ICM_INT_GPIO >= 0
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << ICM_INT_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io);
    gpio_install_isr_service(0);
#endif
    const bool ok = xTaskCreatePinnedToCore(imuTask, "IMU", STACK_IMU, nullptr, PRIO_IMU, &s_task, CORE_SYSTEM) == pdPASS;
#if ICM_INT_GPIO >= 0
    if (ok) gpio_isr_handler_add((gpio_num_t)ICM_INT_GPIO, intIsr, nullptr);
#endif
    return ok;
}

bool imu_get_latest(ImuSample* out) {
    bool have;
    portENTER_CRITICAL(&s_mux);
    have = s_have;
    if (have) *out = s_latest;
    portEXIT_CRITICAL(&s_mux);
    return have && s_ok;
}
bool     imu_is_ok()   { return s_ok; }
uint16_t imu_rate_hz() { return s_ok ? s_rateHz : 0; }
