#include "leumpang/icm20948.h"

#include <math.h>

namespace la {

namespace {
constexpr uint8_t REG_BANK_SEL = 0x7F;
constexpr uint8_t B0_WHO_AM_I = 0x00, B0_USER_CTRL = 0x03, B0_PWR_MGMT_1 = 0x06, B0_PWR_MGMT_2 = 0x07;
constexpr uint8_t B0_INT_PIN_CFG = 0x0F, B0_INT_ENABLE_1 = 0x11, B0_ACCEL_XOUT_H = 0x2D;
constexpr uint8_t B2_GYRO_SMPLRT_DIV = 0x00, B2_GYRO_CONFIG_1 = 0x01, B2_ODR_ALIGN_EN = 0x09;
constexpr uint8_t B2_ACCEL_SMPLRT_DIV_1 = 0x10, B2_ACCEL_SMPLRT_DIV_2 = 0x11, B2_ACCEL_CONFIG = 0x14;
constexpr float GYRO_LSB_PER_DPS = 32.8f;      // +-1000 dps
constexpr float ACC_LSB_PER_G    = 8192.0f;    // +-4 g
constexpr float DEG2RAD          = 0.01745329252f;
}  // namespace

bool Icm20948::bank(uint8_t b) {
    if (bank_ == (int8_t)b) return true;
    const bool ok = wr(REG_BANK_SEL, (uint8_t)(b << 4));
    bank_ = ok ? (int8_t)b : -1;
    return ok;
}

Icm20948::Err Icm20948::init(const I2cOps& ops, bool enableDataReadyInt) {
    ops_ = ops; bank_ = -1; who_ = 0;
    if (!bank(0) || !rd(B0_WHO_AM_I, &who_, 1)) return Err::NO_ACK;
    if (who_ != WHO_AM_I_VAL) return Err::WHO_AM_I;
    bool ok = wr(B0_PWR_MGMT_1, 0x80);             // reset
    delay(100);
    bank_ = -1;
    ok &= bank(0);
    ok &= wr(B0_PWR_MGMT_1, 0x01);                  // auto clock
    delay(30);
    ok &= wr(B0_PWR_MGMT_2, 0x00);                  // accel + gyro ON
    ok &= wr(B0_USER_CTRL, 0x00);                   // DMP/FIFO/I2C master OFF
    if (enableDataReadyInt) {
        ok &= wr(B0_INT_PIN_CFG, 0x10);             // clear on any read, push-pull, aktif HIGH
        ok &= wr(B0_INT_ENABLE_1, 0x01);            // RAW_DATA_0_RDY
    }
    ok &= bank(2);
    ok &= wr(B2_GYRO_SMPLRT_DIV, 1);                // 562,5 Hz
    ok &= wr(B2_GYRO_CONFIG_1, (3 << 3) | (2 << 1) | 1);
    ok &= wr(B2_ACCEL_SMPLRT_DIV_1, 0);
    ok &= wr(B2_ACCEL_SMPLRT_DIV_2, 1);
    ok &= wr(B2_ACCEL_CONFIG, (3 << 3) | (1 << 1) | 1);
    ok &= wr(B2_ODR_ALIGN_EN, 1);
    ok &= bank(0);
    delay(50);
    return ok ? Err::OK : Err::CONFIG;
}

bool Icm20948::read(ImuRaw& r) {
    uint8_t d[12];
    if (!bank(0) || !rd(B0_ACCEL_XOUT_H, d, sizeof(d))) return false;
    auto s16 = [&](int i) { return (int16_t)((uint16_t)d[i] << 8 | d[i + 1]); };
    r.ax = s16(0) / ACC_LSB_PER_G;  r.ay = s16(2) / ACC_LSB_PER_G;  r.az = s16(4) / ACC_LSB_PER_G;
    r.gx = s16(6)  / GYRO_LSB_PER_DPS * DEG2RAD;
    r.gy = s16(8)  / GYRO_LSB_PER_DPS * DEG2RAD;
    r.gz = s16(10) / GYRO_LSB_PER_DPS * DEG2RAD;
    return true;
}

bool Icm20948::calibrateGyroBias(int n) {
    float sx = 0, sy = 0, sz = 0, mx = 0;
    int got = 0;
    ImuRaw r;
    for (int i = 0; i < n; ++i) {
        if (read(r)) {
            sx += r.gx; sy += r.gy; sz += r.gz; ++got;
            const float m = fabsf(r.gx) + fabsf(r.gy) + fabsf(r.gz);
            if (m > mx) mx = m;
        }
        delay(2);
    }
    if (got > n / 2 && mx < 0.15f) {
        const float g = (float)got;
        bgx_ = sx / g; bgy_ = sy / g; bgz_ = sz / g;
        return true;
    }
    bgx_ = bgy_ = bgz_ = 0;
    return false;
}

void Icm20948::biasDps(float& x, float& y, float& z) const {
    x = bgx_ / DEG2RAD; y = bgy_ / DEG2RAD; z = bgz_ / DEG2RAD;
}

void Icm20948::fusionInit(const ImuRaw& r) {
    const float rl = atan2f(r.ay, r.az);
    const float pt = atan2f(-r.ax, sqrtf(r.ay * r.ay + r.az * r.az));
    const float cr = cosf(rl * 0.5f), sr = sinf(rl * 0.5f);
    const float cp = cosf(pt * 0.5f), sp = sinf(pt * 0.5f);
    q0_ = cr * cp; q1_ = sr * cp; q2_ = cr * sp; q3_ = -sr * sp;
    ix_ = iy_ = iz_ = 0;
}

void Icm20948::fusionUpdate(const ImuRaw& r, float dt) {
    float gx = r.gx - bgx_, gy = r.gy - bgy_, gz = r.gz - bgz_;
    float ax = r.ax, ay = r.ay, az = r.az;
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > params.gateLo && an < params.gateHi) {
        const float inv = 1.0f / an;
        ax *= inv; ay *= inv; az *= inv;
        const float vx = q1_ * q3_ - q0_ * q2_;
        const float vy = q0_ * q1_ + q2_ * q3_;
        const float vz = q0_ * q0_ - 0.5f + q3_ * q3_;
        const float ex = ay * vz - az * vy;
        const float ey = az * vx - ax * vz;
        const float ez = ax * vy - ay * vx;
        if (params.ki2 > 0.0f) {
            ix_ += params.ki2 * ex * dt; iy_ += params.ki2 * ey * dt; iz_ += params.ki2 * ez * dt;
            gx += ix_; gy += iy_; gz += iz_;
        }
        gx += params.kp2 * ex; gy += params.kp2 * ey; gz += params.kp2 * ez;
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    const float a = q0_, b = q1_, c = q2_;
    q0_ += (-b * gx - c * gy - q3_ * gz);
    q1_ += ( a * gx + c * gz - q3_ * gy);
    q2_ += ( a * gy - b * gz + q3_ * gx);
    q3_ += ( a * gz + b * gy - c * gx);
    const float n = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    if (n > 1e-6f && isFiniteVal(n)) { const float i = 1.0f / n; q0_ *= i; q1_ *= i; q2_ *= i; q3_ *= i; }
    else { q0_ = 1; q1_ = q2_ = q3_ = 0; }
}

float Icm20948::roll() const {
    return atan2f(2.0f * (q0_ * q1_ + q2_ * q3_), 1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
}
float Icm20948::pitch() const {
    return asinf(clampf(2.0f * (q0_ * q2_ - q3_ * q1_), -1.0f, 1.0f));
}

}  // namespace la
