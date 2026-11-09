// =============================================================================
//  Leumpang Radio (Fase 4B): status stik (PS4/PS5/...) dari board radio ke robot.
//  Board radio (ESP32 klasik atau Pico 2 W, Bluepad32) mengirim LAP PAD 50 Hz lewat UART;
//  robot membalas PAD_FEEDBACK (warna lightbar per mode, getar saat jatuh/E-STOP).
//  Header ini dipakai core, firmware radio, simulator, dan tes.
// =============================================================================
#pragma once
#include <stdint.h>

namespace la {

// Tombol (bit) - nama PlayStation; stik lain dipetakan Bluepad32 ke posisi yang sama.
enum PadButton : uint32_t {
    PAD_CROSS    = 1u << 0,  PAD_CIRCLE  = 1u << 1,  PAD_SQUARE  = 1u << 2,  PAD_TRIANGLE = 1u << 3,
    PAD_L1       = 1u << 4,  PAD_R1      = 1u << 5,  PAD_L2      = 1u << 6,  PAD_R2       = 1u << 7,
    PAD_SHARE    = 1u << 8,  PAD_OPTIONS = 1u << 9,  PAD_L3      = 1u << 10, PAD_R3       = 1u << 11,
    PAD_PS       = 1u << 12, PAD_TOUCH   = 1u << 13,
    PAD_UP       = 1u << 16, PAD_DOWN    = 1u << 17, PAD_RIGHT   = 1u << 18, PAD_LEFT     = 1u << 19,
};
enum PadFlag : uint8_t { PAD_CONNECTED = 1u << 0 };

// Satu laporan. Sumbu -127..127, atas/kanan = positif. Pemicu analog 0..255.
struct PadState {
    uint8_t  flags;
    uint8_t  battery;          // 0..255 dari Bluepad32 (0 = tidak diketahui)
    uint32_t buttons;
    int8_t   lx, ly, rx, ry;
    uint8_t  l2, r2;
    uint8_t  seq;
};
constexpr uint8_t PAD_BODY_LEN = 13;

inline void padEncode(const PadState& p, uint8_t* b) {
    b[0] = p.flags; b[1] = p.battery;
    for (int i = 0; i < 4; ++i) b[2 + i] = (uint8_t)(p.buttons >> (8 * i));
    b[6] = (uint8_t)p.lx; b[7] = (uint8_t)p.ly; b[8] = (uint8_t)p.rx; b[9] = (uint8_t)p.ry;
    b[10] = p.l2; b[11] = p.r2; b[12] = p.seq;
}
inline bool padDecode(const uint8_t* b, uint8_t len, PadState& p) {
    if (len < PAD_BODY_LEN) return false;
    p.flags = b[0]; p.battery = b[1];
    p.buttons = (uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24);
    p.lx = (int8_t)b[6]; p.ly = (int8_t)b[7]; p.rx = (int8_t)b[8]; p.ry = (int8_t)b[9];
    p.l2 = b[10]; p.r2 = b[11]; p.seq = b[12];
    return true;
}

// Balasan robot -> radio: body r, g, b, rumbleMs/10, weak, strong, playerLeds
struct PadFeedback {
    uint8_t r, g, b;
    uint8_t rumble10ms;        // 0 = tanpa getar
    uint8_t weak, strong;
    uint8_t leds;              // bit 0..3 LED pemain (DS4 tidak punya; DS5 ada)
};
constexpr uint8_t PAD_FEEDBACK_LEN = 7;

// Batas waktu: tanpa PAD selama ini = stik dianggap hilang (gait berhenti, bukan E-STOP).
constexpr uint32_t PAD_TIMEOUT_MS   = 250;
// Setelah tersambung, PS/touchpad diabaikan sebentar: tombol PS dipakai untuk menyambung DS4.
constexpr uint32_t PAD_ARM_DELAY_MS = 1000;
constexpr int      PAD_DEADZONE     = 20;      // dari 127

}  // namespace la
