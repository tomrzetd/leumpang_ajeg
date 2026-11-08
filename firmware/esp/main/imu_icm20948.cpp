// =============================================================================
//  ICM-20948 (I2C, driver i2c_master IDF >= 5.2) + fusi Mahony 6-sumbu, 500 Hz
//  Task di core0: transaksi I2C berbasis interrupt (task blocking, CPU bebas),
//  sehingga core1 tidak lagi menunggu bus I2C seperti di V3 (DMP FIFO di loop kontrol).
// =============================================================================
#include "shared.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

namespace {

// ---- Register (bank 0 kecuali disebut) ------------------------------------
constexpr uint8_t REG_BANK_SEL     = 0x7F;
constexpr uint8_t B0_WHO_AM_I      = 0x00;   // = 0xEA
constexpr uint8_t B0_USER_CTRL     = 0x03;
constexpr uint8_t B0_PWR_MGMT_1    = 0x06;
constexpr uint8_t B0_PWR_MGMT_2    = 0x07;
[[maybe_unused]] constexpr uint8_t B0_INT_PIN_CFG   = 0x0F;
[[maybe_unused]] constexpr uint8_t B0_INT_ENABLE_1  = 0x11;
constexpr uint8_t B0_ACCEL_XOUT_H  = 0x2D;   // 0x2D..0x38 = accel XYZ + gyro XYZ
constexpr uint8_t B2_GYRO_SMPLRT_DIV  = 0x00;
constexpr uint8_t B2_GYRO_CONFIG_1    = 0x01;
constexpr uint8_t B2_ODR_ALIGN_EN     = 0x09;
constexpr uint8_t B2_ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t B2_ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t B2_ACCEL_CONFIG     = 0x14;
constexpr uint8_t WHO_AM_I_VAL     = 0xEA;

constexpr float GYRO_LSB_PER_DPS = 32.8f;    // FS +-1000 dps
constexpr float ACC_LSB_PER_G    = 8192.0f;  // FS +-4 g
constexpr float DEG2RAD          = 0.01745329252f;
constexpr int   I2C_TIMEOUT_MS   = 5;

i2c_master_bus_handle_t s_bus = nullptr;
i2c_master_dev_handle_t s_dev = nullptr;
int8_t       s_bank = -1;
TaskHandle_t s_task = nullptr;

ImuSample    s_latest{};
bool         s_have = false;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool     s_ok = false;
volatile uint16_t s_rateHz = 0;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float s_ix = 0, s_iy = 0, s_iz = 0;           // integral Mahony
float s_bgx = 0, s_bgy = 0, s_bgz = 0;        // bias gyro (rad/s)

esp_err_t wr(uint8_t reg, uint8_t val) {
    const uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, I2C_TIMEOUT_MS);
}
esp_err_t rd(uint8_t reg, uint8_t* out, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, I2C_TIMEOUT_MS);
}
esp_err_t bank(uint8_t b) {
    if (s_bank == (int8_t)b) return ESP_OK;
    const esp_err_t e = wr(REG_BANK_SEL, (uint8_t)(b << 4));
    s_bank = (e == ESP_OK) ? (int8_t)b : -1;
    return e;
}

struct Raw { float ax, ay, az, gx, gy, gz; };   // g, rad/s

bool readRaw(Raw* r) {
    uint8_t d[12];
    if (bank(0) != ESP_OK || rd(B0_ACCEL_XOUT_H, d, sizeof(d)) != ESP_OK) return false;
    auto s16 = [&](int i) { return (int16_t)((uint16_t)d[i] << 8 | d[i + 1]); };
    r->ax = s16(0) / ACC_LSB_PER_G;  r->ay = s16(2) / ACC_LSB_PER_G;  r->az = s16(4) / ACC_LSB_PER_G;
    r->gx = s16(6)  / GYRO_LSB_PER_DPS * DEG2RAD;
    r->gy = s16(8)  / GYRO_LSB_PER_DPS * DEG2RAD;
    r->gz = s16(10) / GYRO_LSB_PER_DPS * DEG2RAD;
    return true;
}

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
    s_bank = -1;

    uint8_t who = 0;
    if (bank(0) != ESP_OK || rd(B0_WHO_AM_I, &who, 1) != ESP_OK || who != WHO_AM_I_VAL) {
        logPrintf("IMU: WHO_AM_I=0x%02X (harus 0xEA)\r\n", who);
        return false;
    }
    bool ok = wr(B0_PWR_MGMT_1, 0x80) == ESP_OK;              // reset
    vTaskDelay(pdMS_TO_TICKS(100));
    s_bank = -1;
    ok &= bank(0) == ESP_OK;
    ok &= wr(B0_PWR_MGMT_1, 0x01) == ESP_OK;                  // auto clock, bangun
    vTaskDelay(pdMS_TO_TICKS(30));
    ok &= wr(B0_PWR_MGMT_2, 0x00) == ESP_OK;                  // accel + gyro ON
    ok &= wr(B0_USER_CTRL, 0x00) == ESP_OK;                   // DMP/FIFO/I2C master OFF
#if ICM_INT_GPIO >= 0
    ok &= wr(B0_INT_PIN_CFG, 0x10) == ESP_OK;                 // clear on any read, push-pull, aktif HIGH
    ok &= wr(B0_INT_ENABLE_1, 0x01) == ESP_OK;                // RAW_DATA_0_RDY
#endif
    ok &= bank(2) == ESP_OK;
    ok &= wr(B2_GYRO_SMPLRT_DIV, 1) == ESP_OK;                // 1125/(1+1) = 562,5 Hz
    ok &= wr(B2_GYRO_CONFIG_1, (3 << 3) | (2 << 1) | 1) == ESP_OK;  // DLPF ~120 Hz, +-1000 dps
    ok &= wr(B2_ACCEL_SMPLRT_DIV_1, 0) == ESP_OK;
    ok &= wr(B2_ACCEL_SMPLRT_DIV_2, 1) == ESP_OK;             // 562,5 Hz
    ok &= wr(B2_ACCEL_CONFIG, (3 << 3) | (1 << 1) | 1) == ESP_OK;   // DLPF ~50 Hz, +-4 g
    ok &= wr(B2_ODR_ALIGN_EN, 1) == ESP_OK;
    ok &= bank(0) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!ok) logPrintf("IMU: konfigurasi register gagal\r\n");
    else     logPrintf("IMU: ICM-20948 OK @0x%02X, raw 562 Hz, fusi Mahony %u Hz\r\n",
                       (unsigned)addr, (unsigned)(1000000u / IMU_PERIOD_US));
    return ok;
}

// Bias gyro saat diam (0,4 s). Bila robot bergerak, bias = 0 dan Ki menangani sisanya.
void gyroBiasCal() {
    float sx = 0, sy = 0, sz = 0, mx = 0;
    int n = 0;
    Raw r;
    for (int i = 0; i < 200; ++i) {
        if (readRaw(&r)) {
            sx += r.gx; sy += r.gy; sz += r.gz; ++n;
            const float m = fabsf(r.gx) + fabsf(r.gy) + fabsf(r.gz);
            if (m > mx) mx = m;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (n > 100 && mx < 0.15f) {
        const float fn = (float)n; s_bgx = sx / fn; s_bgy = sy / fn; s_bgz = sz / fn;
        logPrintf("IMU: bias gyro %.2f %.2f %.2f deg/s\r\n",
                  (double)(s_bgx / DEG2RAD), (double)(s_bgy / DEG2RAD), (double)(s_bgz / DEG2RAD));
    } else {
        s_bgx = s_bgy = s_bgz = 0;
        logPrintf("IMU: robot bergerak saat kalibrasi bias gyro, bias = 0\r\n");
    }
}

void quatFromAccel(const Raw& r) {
    const float roll  = atan2f(r.ay, r.az);
    const float pitch = atan2f(-r.ax, sqrtf(r.ay * r.ay + r.az * r.az));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
    s_ix = s_iy = s_iz = 0;
}

void mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    // Koreksi accel hanya saat percepatan ~1 g (abaikan hentakan kaki / impak).
    if (an > IMU_ACC_GATE_LO && an < IMU_ACC_GATE_HI) {
        const float inv = 1.0f / an;
        ax *= inv; ay *= inv; az *= inv;
        const float vx = q1 * q3 - q0 * q2;
        const float vy = q0 * q1 + q2 * q3;
        const float vz = q0 * q0 - 0.5f + q3 * q3;
        const float ex = ay * vz - az * vy;
        const float ey = az * vx - ax * vz;
        const float ez = ax * vy - ay * vx;
        if (IMU_MAHONY_KI2 > 0.0f) {
            s_ix += IMU_MAHONY_KI2 * ex * dt; s_iy += IMU_MAHONY_KI2 * ey * dt; s_iz += IMU_MAHONY_KI2 * ez * dt;
            gx += s_ix; gy += s_iy; gz += s_iz;
        }
        gx += IMU_MAHONY_KP2 * ex; gy += IMU_MAHONY_KP2 * ey; gz += IMU_MAHONY_KP2 * ez;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    const float a = q0, b = q1, c = q2;
    q0 += (-b * gx - c * gy - q3 * gz);
    q1 += ( a * gx + c * gz - q3 * gy);
    q2 += ( a * gy - b * gz + q3 * gx);
    q3 += ( a * gz + b * gy - c * gx);
    const float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (n > 1e-6f && isFiniteVal(n)) { const float i = 1.0f / n; q0 *= i; q1 *= i; q2 *= i; q3 *= i; }
    else { q0 = 1; q1 = q2 = q3 = 0; }
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
        // ---- inisialisasi (ulang) ----
        s_ok = false;
        while (!chipInit()) vTaskDelay(pdMS_TO_TICKS(1000));
        gyroBiasCal();
        Raw r;
        while (!readRaw(&r)) vTaskDelay(pdMS_TO_TICKS(5));
        quatFromAccel(r);
        s_ok = true;

        int64_t lastUs = esp_timer_get_time();
        int64_t rateT0 = lastUs;
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
            if (!readRaw(&r)) {
                if (++errs >= 20) {                      // ~40 ms gagal beruntun -> pulihkan bus
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
            const float gx = r.gx - s_bgx, gy = r.gy - s_bgy, gz = r.gz - s_bgz;
            mahony(gx, gy, gz, r.ax, r.ay, r.az, dt);

            ImuSample s;
            s.qw = q0; s.qx = q1; s.qy = q2; s.qz = q3;
            s.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
            s.pitch = asinf(clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
            s.gx = gx; s.gy = gy; s.gz = gz;
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
    gpio_install_isr_service(0);                       // ESP_ERR_INVALID_STATE bila sudah ada: aman
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
