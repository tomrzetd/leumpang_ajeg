// =============================================================================
//  Leumpang Ajeg - firmware ESP32-S3: KONFIGURASI BOARD
//  (konstanta robot/gait ada di core/include/leumpang/types.h)
//  Semua makro #ifndef dapat di-override lewat build flag (-D...).
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
//  Pin ESP32-S3 N16R8 (sama dengan diagram wiring V3)
//  Dicadangkan: GPIO19/20 (USB D-/D+), GPIO26..32 (flash), GPIO33..37 (Octal PSRAM),
//  strapping GPIO0/3/45/46.
// ---------------------------------------------------------------------------
constexpr int PIN_LED1 = 2;      // heartbeat sederhana (opsional)
constexpr int PIN_RGB  = 4;      // WS2812B status
constexpr int PIN_T2   = 5;      // pin ukur jitter: HIGH selama satu siklus kontrol

constexpr int BUS_R_TX = 17, BUS_R_RX = 16, BUS_R_EN = 6;   // Serial1 kanan
constexpr int BUS_L_TX = 21, BUS_L_RX = 18, BUS_L_EN = 7;   // Serial2 kiri

constexpr int IMU_SDA = 8, IMU_SCL = 9;
#ifndef ICM_INT_GPIO
#define ICM_INT_GPIO -1          // diagram: INT = NC. Isi mis. 12 bila INT disambung.
#endif

constexpr int LCD_SCK = 10, LCD_MOSI = 11, LCD_CS = 13, LCD_DC = 14, LCD_RST = 15;

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
constexpr uint32_t STACK_CONSOLE = 3072;
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
//  Guard compile-time pin
// ---------------------------------------------------------------------------
constexpr int V4_USED_PINS[] = {
    PIN_LED1, PIN_RGB, PIN_T2, BUS_R_TX, BUS_R_RX, BUS_L_TX, BUS_L_RX,
    IMU_SDA, IMU_SCL, LCD_SCK, LCD_MOSI, LCD_CS, LCD_DC, LCD_RST,
#if ICS_DIR_MODE != 0
    BUS_R_EN, BUS_L_EN,
#endif
#if ICM_INT_GPIO >= 0
    ICM_INT_GPIO,
#endif
};
constexpr size_t V4_PIN_COUNT = sizeof(V4_USED_PINS) / sizeof(V4_USED_PINS[0]);
constexpr bool v4PinsUnique() {
    for (size_t i = 0; i < V4_PIN_COUNT; ++i)
        for (size_t j = i + 1; j < V4_PIN_COUNT; ++j)
            if (V4_USED_PINS[i] == V4_USED_PINS[j]) return false;
    return true;
}
constexpr bool v4PinsAvoidReserved() {
    for (size_t i = 0; i < V4_PIN_COUNT; ++i) {
        const int p = V4_USED_PINS[i];
        if (p == 0 || p == 3 || p == 19 || p == 20 || p == 45 || p == 46) return false;
        if (p >= 26 && p <= 37) return false;
        if (p > 48 || p < 0) return false;
        if (p >= 22 && p <= 25) return false;    // tidak ada di ESP32-S3
    }
    return true;
}
static_assert(v4PinsUnique(), "GPIO conflict: dua fungsi memakai pin yang sama");
static_assert(v4PinsAvoidReserved(), "Memakai GPIO USB/flash/PSRAM/strapping yang dicadangkan");
static_assert(ICS_DIR_MODE >= 0 && ICS_DIR_MODE <= 2, "ICS_DIR_MODE harus 0/1/2");
static_assert(sizeof(WIFI_AP_PASS) - 1 >= 8, "WPA2 butuh password AP >= 8 karakter");
