// =============================================================================
//  Board: ESP32-S3 N16R8 (sesuai diagram wiring V3) + slot SD tambahan
//  Dicadangkan: GPIO19/20 (USB), 26..32 (flash), 33..37 (Octal PSRAM),
//  strapping 0/3/45/46.
// =============================================================================
#pragma once
#include "sdkconfig.h"

#define LA_BOARD_NAME   "leumpang-s3"
#define LA_BOARD_ID     1                 // la::lap::BOARD_ESP32S3

constexpr int PIN_LED1 = 2;
constexpr int PIN_RGB  = 4;
constexpr int PIN_T2   = 5;

constexpr int BUS_R_TX = 17, BUS_R_RX = 16, BUS_R_EN = 6;
constexpr int BUS_L_TX = 21, BUS_L_RX = 18, BUS_L_EN = 7;

constexpr int IMU_SDA = 8, IMU_SCL = 9;
#ifndef ICM_INT_GPIO
#define ICM_INT_GPIO -1                   // opsional: 1 atau 12 bila INT disambung
#endif

// ---- Leumpang Radio (Fase 4B) ----
//  S3 hanya punya 3 UART (1/2 = bus ICS). Radio memakai UART0 bila konsol dipindah ke
//  USB-Serial-JTAG (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y); selain itu radio nonaktif.
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#ifndef LA_RADIO_TX
#define LA_RADIO_TX 48
#endif
#ifndef LA_RADIO_RX
#define LA_RADIO_RX 12
#endif
#define LA_RADIO_UART 0
#else
#define LA_RADIO_TX -1
#define LA_RADIO_RX -1
#define LA_RADIO_UART -1
#endif

constexpr int LCD_SCK = 10, LCD_MOSI = 11, LCD_CS = 13, LCD_DC = 14, LCD_RST = 15;

// ---- SD card (modul microSD, SDMMC lewat GPIO matrix) -----------------------
//  1-bit: CLK, CMD, D0. 4-bit (LA_SD_4BIT=1): + D1, D2, D3.
//  Modul microSD wajib punya pull-up 10k di CMD & D0..D3 (kebanyakan sudah).
#ifndef LA_SD_ENABLE
#define LA_SD_ENABLE 1
#endif
#ifndef LA_SD_4BIT
#define LA_SD_4BIT 0
#endif
constexpr int SD_CLK = 39, SD_CMD = 38, SD_D0 = 40, SD_D1 = 41, SD_D2 = 42, SD_D3 = 47;
constexpr int SD_PWR_EN = -1;             // tanpa pin daya
constexpr int SD_LDO_CHAN = -1;           // tanpa LDO internal

constexpr int BOARD_USED_PINS[] = {
    PIN_LED1, PIN_RGB, PIN_T2, BUS_R_TX, BUS_R_RX, BUS_L_TX, BUS_L_RX, BUS_R_EN, BUS_L_EN,
    IMU_SDA, IMU_SCL, LCD_SCK, LCD_MOSI, LCD_CS, LCD_DC, LCD_RST,
#if LA_RADIO_TX >= 0
    LA_RADIO_TX, LA_RADIO_RX,
#endif
#if LA_SD_ENABLE
    SD_CLK, SD_CMD, SD_D0,
#if LA_SD_4BIT
    SD_D1, SD_D2, SD_D3,
#endif
#endif
#if ICM_INT_GPIO >= 0
    ICM_INT_GPIO,
#endif
};
constexpr bool boardPinReserved(int p) {
    return p == 0 || p == 3 || p == 19 || p == 20 || p == 45 || p == 46 ||
           (p >= 22 && p <= 25) || (p >= 26 && p <= 37) || p < 0 || p > 48;
}
