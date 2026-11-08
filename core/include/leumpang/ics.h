// =============================================================================
//  Protokol servo Kondo ICS 3.5 + antarmuka HAL bus servo
// =============================================================================
#pragma once
#include "leumpang/types.h"

namespace la {
namespace ics {

// Frame posisi: [0x80|id, pos>>7, pos&0x7F]
inline void buildPosition(uint8_t* f, uint8_t id, uint16_t pos) {
    f[0] = (uint8_t)(0x80 | (id & 0x1F));
    f[1] = (uint8_t)((pos >> 7) & 0x7F);
    f[2] = (uint8_t)(pos & 0x7F);
}

// Reply valid harus tepat di offset 0 (tanpa echo) atau 3 (setelah echo yang IDENTIK
// dengan frame terkirim). Reply = cmd dengan bit7 = 0, lalu posisi 14-bit.
inline bool parseReply(const uint8_t* buf, size_t n, const uint8_t* sent, int16_t* pos) {
    size_t off = 0;
    if (n >= 3 && buf[0] == sent[0] && buf[1] == sent[1] && buf[2] == sent[2]) off = 3;
    if (n < off + 3) return false;
    if (buf[off] != (uint8_t)(sent[0] & 0x7F)) return false;
    if ((buf[off + 1] & 0x80) || (buf[off + 2] & 0x80)) return false;
    const int16_t v = (int16_t)(((uint16_t)buf[off + 1] << 7) | buf[off + 2]);
    if (v < ICS_FB_MIN || v > ICS_FB_MAX) return false;
    *pos = v;
    return true;
}

}  // namespace ics

// Satu "slot": frame untuk bus kanan & kiri dikirim bersamaan, reply dikumpulkan.
struct IcsExchange {
    uint8_t tx[SERVO_BUSES][3];
    bool    got[SERVO_BUSES];
    int16_t pos[SERVO_BUSES];
};

// HAL: implementasi per board (UART ESP-IDF, USART HAL STM32, model simulasi).
// Wajib: kirim tx[0] ke bus R dan tx[1] ke bus L, tunggu reply dengan batas waktu
// (~300 us), isi got/pos memakai ics::parseReply. Tidak boleh memblokir > 1 ms.
class ServoBus {
public:
    virtual ~ServoBus() = default;
    virtual void exchange(IcsExchange& x) = 0;
};

}  // namespace la
