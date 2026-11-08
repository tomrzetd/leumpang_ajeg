// =============================================================================
//  ICM-20948 (raw accel + gyro lewat I2C) + fusi Mahony 6-sumbu - portabel
//  Dipakai firmware ESP-IDF dan STM32. Transport I2C disediakan HAL lewat I2cOps.
//  Konfigurasi sensor: gyro +-1000 dps DLPF ~120 Hz, accel +-4 g DLPF ~50 Hz, ODR 562,5 Hz.
// =============================================================================
#pragma once
#include "leumpang/types.h"

namespace la {

struct I2cOps {
    void* ctx = nullptr;
    bool (*write)(void* ctx, uint8_t reg, uint8_t val) = nullptr;
    bool (*read)(void* ctx, uint8_t reg, uint8_t* d, size_t n) = nullptr;
    void (*delayMs)(void* ctx, uint32_t ms) = nullptr;
};

struct ImuRaw { float ax, ay, az, gx, gy, gz; };   // g, rad/s

struct MahonyParams {
    float kp2 = 1.0f;          // 2*Kp
    float ki2 = 0.02f;         // 2*Ki
    float gateLo = 0.80f;      // |a| di luar [lo, hi] g -> koreksi accel diabaikan (impak kaki)
    float gateHi = 1.20f;
};

class Icm20948 {
public:
    enum class Err : uint8_t { OK, NO_ACK, WHO_AM_I, CONFIG };
    static constexpr uint8_t WHO_AM_I_VAL = 0xEA;

    Err  init(const I2cOps& ops, bool enableDataReadyInt);
    bool read(ImuRaw& r);                         // tanpa koreksi bias
    // Bias gyro saat diam (n sampel, jeda 2 ms). false = robot bergerak -> bias 0.
    bool calibrateGyroBias(int n = 200);
    void biasDps(float& x, float& y, float& z) const;
    uint8_t whoAmI() const { return who_; }

    // ---- fusi ----
    void  fusionInit(const ImuRaw& r);            // quaternion dari accel (tanpa transien)
    void  fusionUpdate(const ImuRaw& r, float dt);  // r dengan bias dikoreksi otomatis
    float roll() const;                            // rad, frame sensor
    float pitch() const;
    void  quat(float& w, float& x, float& y, float& z) const { w = q0_; x = q1_; y = q2_; z = q3_; }
    void  corrected(const ImuRaw& r, float& gx, float& gy, float& gz) const {
        gx = r.gx - bgx_; gy = r.gy - bgy_; gz = r.gz - bgz_; }
    MahonyParams params;

private:
    bool wr(uint8_t reg, uint8_t v) { return ops_.write(ops_.ctx, reg, v); }
    bool rd(uint8_t reg, uint8_t* d, size_t n) { return ops_.read(ops_.ctx, reg, d, n); }
    bool bank(uint8_t b);
    void delay(uint32_t ms) { if (ops_.delayMs) ops_.delayMs(ops_.ctx, ms); }

    I2cOps  ops_{};
    int8_t  bank_ = -1;
    uint8_t who_ = 0;
    float   q0_ = 1, q1_ = 0, q2_ = 0, q3_ = 0;
    float   ix_ = 0, iy_ = 0, iz_ = 0;
    float   bgx_ = 0, bgy_ = 0, bgz_ = 0;
};

}  // namespace la
