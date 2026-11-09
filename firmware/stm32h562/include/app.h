// Antarmuka antar-modul firmware STM32H562
#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
#include "leumpang/types.h"
#include "leumpang/ics.h"
#include "leumpang/motion.h"
#include "leumpang/blackbox.h"
#include "leumpang/files.h"
#include "leumpang/profile.h"

// clock.cpp
void     la_clock_init();                 // PLL 250 MHz, HSI48+CRS (USB), PLL1Q 100 MHz (SDMMC)
void     la_timebase_init();              // TIM2 32-bit 1 MHz + perpanjangan 64-bit
int64_t  la_micros();

// ics_bus.cpp
class Stm32IcsBus : public la::ServoBus {
public:
    void init();
    void exchange(la::IcsExchange& x) override;
    int  transact(uint8_t bus, const uint8_t* tx, uint8_t txLen, uint8_t* rx, uint8_t rxMax,
                  uint8_t want, uint32_t timeoutUs) override;
};

// imu.cpp
bool     imu_init();                      // di main sebelum TIM7 dinyalakan
void     imu_isr_tick();                  // dari TIM7 500 Hz
bool     imu_latest(la::ImuInput& in);    // aman dipanggil dari TIM6 (TIM7 di-mask singkat)

// storage.cpp (main loop saja)
bool     storage_mount();                                       // sebelum config dimuat
void     storage_begin(la::MotionStore& store, la::BlackboxRing* ring); // muat motion, buka blackbox
bool     storage_ok();
la::FileOps* storage_files();                                     // layanan berkas LAP (nullptr tanpa SD)
const la::RobotProfile* storage_profile();                        // /config/robot.json (nullptr = bawaan)
void     storage_service(uint32_t nowMs);
void     storage_save_motion(int slot, const uint8_t* d, size_t n);
bool     storage_load_cfg(la::RuntimeConfig* cfg, int16_t trim[2][la::SERVO_SLOTS]);
bool     storage_save_cfg(const la::RuntimeConfig& cfg, const int16_t trim[2][la::SERVO_SLOTS]);

// main.cpp
void     la_log_main(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
#endif

#ifdef __cplusplus
extern "C" {
#endif
void la_tim2_overflow_isr(void);
void la_tim6_isr(void);
void la_tim7_isr(void);
extern volatile uint32_t g_tim2_hi;
#ifdef __cplusplus
}
#endif
