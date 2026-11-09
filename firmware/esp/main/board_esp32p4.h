// =============================================================================
//  Board: DFRobot FireBeetle 2 ESP32-P4 (DFR1172, chip v1.0, 360 MHz)
//  Fungsi on-board (wiki DFRobot):
//    ESP32-C6 (Wi-Fi via esp_hosted SDIO): D0..D3=14..17, CLK=18, CMD=19, EN=54, WAKEUP=6
//    TF card (SDMMC slot 0, IOMUX): D0..D3=39..42, CLK=43, CMD=44, EN=45
//    LED=3, BOOT=35, I2C SDA=7 SCL=8, UART0 TX=37 RX=38, MIC CLK=12 DATA=9
//  Pin header yang dipakai robot: 20,22,23,28,29,30,32,33,48,49,50,51,52 (+7/8)
//  VERIFIKASI ulang dengan pinout cetak board Anda sebelum menyambung.
// =============================================================================
#pragma once

#define LA_BOARD_NAME   "leumpang-p4"
#define LA_BOARD_ID     2                 // la::lap::BOARD_ESP32P4

constexpr int PIN_LED1 = 3;               // LED on-board
constexpr int PIN_RGB  = 51;
constexpr int PIN_T2   = 52;

constexpr int BUS_R_TX = 20, BUS_R_RX = 22, BUS_R_EN = 23;
constexpr int BUS_L_TX = 48, BUS_L_RX = 49, BUS_L_EN = 50;

constexpr int IMU_SDA = 7, IMU_SCL = 8;   // header I2C board
#ifndef ICM_INT_GPIO
#define ICM_INT_GPIO -1
#endif

// ---- Leumpang Radio (Fase 4B): UART3 ke board radio (ESP32 klasik / Pico 2 W) ----
//  P4 TX -> RX radio, P4 RX <- TX radio, GND bersama. 3,3 V. VERIFIKASI pin header board Anda.
#ifndef LA_RADIO_TX
#define LA_RADIO_TX 46
#endif
#ifndef LA_RADIO_RX
#define LA_RADIO_RX 47
#endif
#define LA_RADIO_UART 3

constexpr int LCD_SCK = 28, LCD_MOSI = 29, LCD_CS = 30, LCD_DC = 32, LCD_RST = 33;

// ---- SD card: slot TF on-board ---------------------------------------------
#ifndef LA_SD_ENABLE
#define LA_SD_ENABLE 1
#endif
#ifndef LA_SD_4BIT
#define LA_SD_4BIT 1
#endif
constexpr int SD_CLK = 43, SD_CMD = 44, SD_D0 = 39, SD_D1 = 40, SD_D2 = 41, SD_D3 = 42;   // IOMUX slot 0
constexpr int SD_PWR_EN = 45;             // EN daya kartu; polaritas dicoba otomatis (aktif-LOW dulu)
constexpr int SD_LDO_CHAN = 4;            // LDO internal untuk domain IO SD (seperti P4-Function-EV)

constexpr int BOARD_USED_PINS[] = {
    PIN_RGB, PIN_T2, BUS_R_TX, BUS_R_RX, BUS_L_TX, BUS_L_RX, BUS_R_EN, BUS_L_EN,
    IMU_SDA, IMU_SCL, LCD_SCK, LCD_MOSI, LCD_CS, LCD_DC, LCD_RST,
#if LA_RADIO_TX >= 0
    LA_RADIO_TX, LA_RADIO_RX,
#endif
#if ICM_INT_GPIO >= 0
    ICM_INT_GPIO,
#endif
};
constexpr bool boardPinReserved(int p) {
    return (p >= 14 && p <= 19) || p == 54 || p == 6 ||        // ESP32-C6 SDIO
           (p >= 39 && p <= 45) ||                              // TF card
           p == 24 || p == 25 ||                                // USB-Serial-JTAG
           (p >= 34 && p <= 38) ||                              // strapping + UART0
           p == 9 || p == 12 || p == 3 ||                       // MIC, LED on-board
           p < 0 || p > 54;
}
