// =============================================================================
//  Leumpang Ajeg - firmware ESP (S3 / P4): KONFIGURASI
//  (konstanta robot/gait ada di core/include/leumpang/types.h)
//  Semua makro #ifndef dapat di-override lewat build flag (-D...).
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
//  Pin per board (dipilih otomatis dari target ESP-IDF)
// ---------------------------------------------------------------------------
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32P4
#include "board_esp32p4.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "board_esp32s3.h"
#else
#error "Target belum didukung: pakai esp32s3 atau esp32p4"
#endif

// ---------------------------------------------------------------------------
//  Arah bus ICS (EN driver half-duplex)
//   0 = tanpa pin arah (adaptor resistor/dioda, echo TX terlihat di RX)
//   1 = GPIO EN dikendalikan software (default, sesuai diagram: EN_R=6, EN_L=7)
//   2 = EN dikendalikan hardware UART mode RS485 (pin RTS) - latensi terkecil;
//       WAJIB verifikasi polaritas EN dengan logic analyzer sebelum dipakai.
// ---------------------------------------------------------------------------
#ifndef ICS_DIR_MODE
#define ICS_DIR_MODE 1
#endif
#ifndef ICS_EN_TX_LEVEL
#define ICS_EN_TX_LEVEL 1        // level EN saat TX (HIGH = TX untuk 74HC126 / Kondo)
#endif
#ifndef ICS_RS485_INVERT_RTS
#define ICS_RS485_INVERT_RTS 0   // mode 2: balik polaritas RTS bila EN terbalik
#endif

// ---------------------------------------------------------------------------
//  Servo ICS 3.5
// ---------------------------------------------------------------------------
#ifndef ICS_REPLY_TIMEOUT_US
#define ICS_REPLY_TIMEOUT_US 300                 // batas tunggu reply per slot (early-exit)
#endif
#ifndef BOOT_SYNC_FROM_FEEDBACK
#define BOOT_SYNC_FROM_FEEDBACK 1                // baca posisi aktual sebelum ramp boot
#endif

// ---------------------------------------------------------------------------
//  Waktu & task
// ---------------------------------------------------------------------------
constexpr uint32_t LOCAL_UI_PERIOD_MS = 100;
constexpr uint32_t TELE_PUBLISH_MS    = 50;      // snapshot telemetry 20 Hz
constexpr uint32_t STACK_LAP          = 6144;
constexpr int      PRIO_LAP           = 4;
constexpr uint8_t  RGB_BRIGHTNESS     = 40;

constexpr int CORE_CONTROL = 1;                  // core1 eksklusif untuk kontrol
constexpr int CORE_SYSTEM  = 0;                  // Wi-Fi, web, IMU, UI, logger

constexpr int PRIO_CONTROL = 20;
constexpr int PRIO_IMU     = 17;                 // core0, di bawah lwIP(18)/Wi-Fi(23)
constexpr int PRIO_LOGGER  = 2;
constexpr int PRIO_CONSOLE = 3;
constexpr int PRIO_UI      = 2;
constexpr int PRIO_SERVICE = 1;

constexpr uint32_t STACK_CONTROL = 8192;
constexpr uint32_t STACK_IMU     = 4096;
constexpr uint32_t STACK_LOGGER  = 3072;
constexpr uint32_t STACK_CONSOLE = 6144;
constexpr uint32_t STACK_UI      = 6144;
constexpr uint32_t STACK_SERVICE = 4096;
constexpr uint32_t STACK_HTTPD   = 8192;

// ---------------------------------------------------------------------------
//  IMU (ICM-20948 raw accel+gyro, fusi Mahony di ESP32)
// ---------------------------------------------------------------------------
constexpr uint32_t IMU_I2C_HZ       = 400000;
constexpr uint32_t IMU_PERIOD_US    = 2000;      // 500 Hz (ODR sensor 562,5 Hz)
constexpr float    IMU_MAHONY_KP2   = 1.0f;      // 2*Kp
constexpr float    IMU_MAHONY_KI2   = 0.02f;     // 2*Ki (bias gyro sisa)
constexpr float    IMU_ACC_GATE_LO  = 0.80f;     // |a| [g] di luar rentang -> koreksi accel diabaikan
constexpr float    IMU_ACC_GATE_HI  = 1.20f;
#ifndef IMU_AUTO_CALIBRATE
#define IMU_AUTO_CALIBRATE 1
#endif
// ---------------------------------------------------------------------------
//  Keselamatan
// ---------------------------------------------------------------------------
#ifndef AUTO_REARM_AFTER_FALL
#define AUTO_REARM_AFTER_FALL 0
#endif
#ifndef SWCONT_LEGACY
#define SWCONT_LEGACY 0
#endif


// ---------------------------------------------------------------------------
//  LCD ST7789 (esp_lcd + DMA)
//  Modul 2.0" = 240x320. Modul 1.69" = 240x280 -> LCD_W 280 + LCD_GAP_X 20.
// ---------------------------------------------------------------------------
#ifndef LCD_PCLK_HZ
#define LCD_PCLK_HZ (40 * 1000 * 1000)   // 80 MHz hanya bila SCK/MOSI/CS di pin IOMUX FSPI
#endif
constexpr int  LCD_W = 320, LCD_H = 240;          // landscape
constexpr bool LCD_SWAP_XY = true, LCD_MIRROR_X = true, LCD_MIRROR_Y = false;
constexpr bool LCD_INVERT  = true;                // kebanyakan modul ST7789 IPS
constexpr int  LCD_GAP_X = 0, LCD_GAP_Y = 0;

// ---------------------------------------------------------------------------
//  Jaringan
// ---------------------------------------------------------------------------
#ifndef APP_USE_MDNS
#define APP_USE_MDNS 1
#endif
constexpr char WIFI_AP_PASS[] = "KHRrobot2026";   // >= 8 char; GANTI sebelum dipakai
constexpr char DASH_USER[]    = "admin";
constexpr char DASH_PASS[]    = "robot2026";      // GANTI sebelum dipakai
constexpr char MDNS_NAME[]    = "leumpang-ajeg";
constexpr bool IMU_SWAP_DEFAULT = false;
constexpr uint8_t WIFI_AP_CHANNEL = 6;

// ---------------------------------------------------------------------------
//  SD card / storage
// ---------------------------------------------------------------------------
constexpr int      PRIO_STORAGE  = 1;
constexpr uint32_t STACK_STORAGE = 6144;
constexpr uint32_t BBX_ROTATE_BYTES = 50u * 1024 * 1024;   // file blackbox baru tiap 50 MB
#ifndef LA_BLACKBOX_DEFAULT
#define LA_BLACKBOX_DEFAULT 1             // rekam blackbox bila SD terpasang
#endif

// ---------------------------------------------------------------------------
//  Guard compile-time pin
// ---------------------------------------------------------------------------
constexpr size_t BOARD_PIN_COUNT = sizeof(BOARD_USED_PINS) / sizeof(BOARD_USED_PINS[0]);
constexpr bool boardPinsUnique() {
    for (size_t i = 0; i < BOARD_PIN_COUNT; ++i)
        for (size_t j = i + 1; j < BOARD_PIN_COUNT; ++j)
            if (BOARD_USED_PINS[i] == BOARD_USED_PINS[j]) return false;
    return true;
}
constexpr bool boardPinsAvoidReserved() {
    for (size_t i = 0; i < BOARD_PIN_COUNT; ++i) if (boardPinReserved(BOARD_USED_PINS[i])) return false;
    return true;
}
static_assert(boardPinsUnique(), "GPIO conflict: dua fungsi memakai pin yang sama");
static_assert(boardPinsAvoidReserved(), "Memakai GPIO yang dicadangkan board (USB/flash/PSRAM/strapping/C6/TF)");
static_assert(ICS_DIR_MODE >= 0 && ICS_DIR_MODE <= 2, "ICS_DIR_MODE harus 0/1/2");
static_assert(sizeof(WIFI_AP_PASS) - 1 >= 8, "WPA2 butuh password AP >= 8 karakter");
