// IMU ICM-20948 di I2C1 (polling HAL) - sampling + Mahony di ISR TIM7 500 Hz
#include "board.h"
#include "app.h"
#include "leumpang/icm20948.h"

namespace {
I2C_HandleTypeDef s_i2c;
uint16_t          s_addr = 0;
la::Icm20948      s_icm;
volatile bool     s_ok = false;
volatile uint32_t s_seq = 0;
volatile float    s_roll = 0, s_pitch = 0;
volatile int64_t  s_tUs = 0;
volatile uint16_t s_rateHz = 0;
int64_t           s_lastUs = 0, s_rateT0 = 0;
uint32_t          s_rateN = 0;
uint8_t           s_errs = 0;

bool opWrite(void*, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&s_i2c, (uint16_t)(s_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 5) == HAL_OK;
}
bool opRead(void*, uint8_t reg, uint8_t* d, size_t n) {
    return HAL_I2C_Mem_Read(&s_i2c, (uint16_t)(s_addr << 1), reg, I2C_MEMADD_SIZE_8BIT, d, (uint16_t)n, 5) == HAL_OK;
}
void opDelay(void*, uint32_t ms) { HAL_Delay(ms); }
}  // namespace

bool imu_init() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitTypeDef g = {};
    g.Pin = IMU_SCL_PIN | IMU_SDA_PIN; g.Mode = GPIO_MODE_AF_OD; g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = IMU_AF;
    HAL_GPIO_Init(IMU_SCL_PORT, &g);
    s_i2c.Instance = IMU_I2C;
    s_i2c.Init.Timing = IMU_I2C_TIMING;
    s_i2c.Init.OwnAddress1 = 0;
    s_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    s_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    s_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    s_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&s_i2c) != HAL_OK) return false;
    if (HAL_I2C_IsDeviceReady(&s_i2c, 0x69 << 1, 2, 10) == HAL_OK) s_addr = 0x69;
    else if (HAL_I2C_IsDeviceReady(&s_i2c, 0x68 << 1, 2, 10) == HAL_OK) s_addr = 0x68;
    else { la_log_main("IMU: ICM-20948 tidak menjawab di 0x68/0x69 (PB6/PB7)\r\n"); return false; }
    la::I2cOps ops; ops.write = opWrite; ops.read = opRead; ops.delayMs = opDelay;
    const la::Icm20948::Err e = s_icm.init(ops, false);
    if (e != la::Icm20948::Err::OK) { la_log_main("IMU: init gagal (%d) WHO_AM_I=0x%02X\r\n", (int)e, s_icm.whoAmI()); return false; }
    if (s_icm.calibrateGyroBias()) { float x, y, z; s_icm.biasDps(x, y, z);
        la_log_main("IMU: OK @0x%02X, bias gyro %.2f %.2f %.2f deg/s\r\n", s_addr, (double)x, (double)y, (double)z); }
    else la_log_main("IMU: OK @0x%02X, robot bergerak saat kalibrasi bias (bias=0)\r\n", s_addr);
    la::ImuRaw r;
    if (!s_icm.read(r)) return false;
    s_icm.fusionInit(r);
    s_lastUs = s_rateT0 = la_micros();
    s_ok = true;
    return true;
}

void imu_isr_tick() {
    if (!s_ok && s_errs == 0) return;
    la::ImuRaw r;
    if (!s_icm.read(r)) {
        if (++s_errs >= 20) s_ok = false;       // ~40 ms gagal -> data basi; kontrol akan RESET bila berjalan
        return;
    }
    s_errs = 0; s_ok = true;
    const int64_t now = la_micros();
    const float dt = la::clampf((float)(now - s_lastUs) * 1e-6f, 1e-4f, 0.02f);
    s_lastUs = now;
    s_icm.fusionUpdate(r, dt);
    s_roll = s_icm.roll(); s_pitch = s_icm.pitch(); s_tUs = now;
    s_seq = s_seq + 1;
    if (++s_rateN, now - s_rateT0 >= 1000000) { s_rateHz = (uint16_t)s_rateN; s_rateN = 0; s_rateT0 = now; }
}

bool imu_latest(la::ImuInput& in) {
    NVIC_DisableIRQ(TIM7_IRQn);                  // salinan konsisten dari ISR prioritas lebih tinggi
    in.sensorOk = s_ok; in.hasSample = s_ok && s_seq != 0;
    in.roll = s_roll; in.pitch = s_pitch; in.tUs = s_tUs; in.seq = s_seq; in.rateHz = s_ok ? s_rateHz : 0;
    NVIC_EnableIRQ(TIM7_IRQn);
    return in.hasSample;
}
