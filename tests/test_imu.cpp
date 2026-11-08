// Tes driver ICM-20948 (dengan sensor tiruan) + fusi Mahony
#include "leumpang/icm20948.h"
#include "check.h"
#include <cmath>
#include <map>

using namespace la;

struct FakeIcm {           // register bank-aware
    uint8_t bank = 0; std::map<int, uint8_t> regs; int16_t ax = 0, ay = 0, az = 8192, gx = 0, gy = 0, gz = 0;
    bool present = true;
};
static FakeIcm F;

int main() {
    I2cOps ops;
    ops.write = [](void*, uint8_t r, uint8_t v) { if (!F.present) return false; if (r == 0x7F) F.bank = v >> 4; else F.regs[F.bank * 256 + r] = v; return true; };
    ops.read = [](void*, uint8_t r, uint8_t* d, size_t n) {
        if (!F.present) return false;
        if (F.bank == 0 && r == 0x00) { d[0] = 0xEA; return true; }
        if (F.bank == 0 && r == 0x2D && n == 12) {
            const int16_t v[6] = {F.ax, F.ay, F.az, F.gx, F.gy, F.gz};
            for (int i = 0; i < 6; ++i) { d[2 * i] = (uint8_t)((uint16_t)v[i] >> 8); d[2 * i + 1] = (uint8_t)v[i]; }
            return true;
        }
        return false;
    };
    ops.delayMs = [](void*, uint32_t) {};
    Icm20948 imu;
    F.present = false; CHECK(imu.init(ops, false) == Icm20948::Err::NO_ACK, "sensor tidak ada -> NO_ACK");
    F.present = true;  CHECK(imu.init(ops, false) == Icm20948::Err::OK, "init OK (WHO_AM_I 0xEA)");
    CHECK(F.regs[2 * 256 + 0x01] == ((3 << 3) | (2 << 1) | 1) && F.regs[2 * 256 + 0x14] == ((3 << 3) | (1 << 1) | 1),
          "GYRO_CONFIG_1 / ACCEL_CONFIG di bank 2 sesuai (+-1000 dps, +-4 g)");
    F.gx = 33;      // ~1 dps bias
    CHECK(imu.calibrateGyroBias(), "kalibrasi bias saat diam");
    float bx, by, bz; imu.biasDps(bx, by, bz);
    CHECK(fabsf(bx - 33 / 32.8f) < 0.01f, "bias gyro X terukur ~1 dps");
    F.gx = 0;
    // pitch 15 deg statis
    F.ax = (int16_t)lroundf(-sinf(15 * 0.0174533f) * 8192); F.az = (int16_t)lroundf(cosf(15 * 0.0174533f) * 8192);
    ImuRaw r; imu.read(r); imu.fusionInit(r);
    CHECK(fabsf(imu.pitch() * 57.2958f - 15) < 0.1f, "init fusi dari accel: pitch 15 deg");
    // integrasi gyro + bias dikoreksi: putar 30 deg/s sumbu X selama 1 s
    F.ax = 0; F.az = 8192; imu.read(r); imu.fusionInit(r);
    for (int i = 0; i < 500; ++i) {
        const float a = (i + 1) * 0.002f * 30 * 0.0174533f;
        ImuRaw s{0, sinf(a), cosf(a), 30 * 0.0174533f + 33 / 32.8f * 0.0174533f, 0, 0};   // gyro mentah = benar + bias
        imu.fusionUpdate(s, 0.002f);
    }
    CHECK(fabsf(imu.roll() * 57.2958f - 30) < 0.6f, "integrasi gyro 30 deg/s x 1 s -> roll 30 deg");
    return checkSummary("test_imu");
}
