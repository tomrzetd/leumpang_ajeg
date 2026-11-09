// =============================================================================
//  Protokol servo Kondo ICS 3.5 + antarmuka HAL bus servo
// =============================================================================
#pragma once
#include "leumpang/types.h"
#include <string.h>

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

// ---- Fase 4A: perintah parameter & ID (ICS 3.5) ------------------------------
//  READ   tx [0xA0|id, sc]          rx [0x20|id, sc, v]   (sc=0 EEPROM: rx [0x20|id, 0, 64 byte])
//  WRITE  tx [0xC0|id, sc, v]       rx [0x40|id, sc, v]   (RAM; hilang saat servo mati)
//  ID R   tx [0xFF, 0, 0, 0]        rx [0xE0|id]          (HANYA satu servo di bus!)
//  ID W   tx [0xE0|id, 1, 1, 1]     rx [0xE0|id]
enum Sc : uint8_t { SC_EEPROM = 0, SC_STRETCH = 1, SC_SPEED = 2, SC_CURRENT = 3, SC_TEMP = 4 };
constexpr uint8_t EEPROM_LEN = 64;
constexpr uint8_t MAX_ID     = 31;

inline uint8_t buildRead(uint8_t* f, uint8_t id, uint8_t sc) {
    f[0] = (uint8_t)(0xA0 | (id & 0x1F)); f[1] = sc; return 2;
}
inline uint8_t buildWrite(uint8_t* f, uint8_t id, uint8_t sc, uint8_t v) {
    f[0] = (uint8_t)(0xC0 | (id & 0x1F)); f[1] = sc; f[2] = (uint8_t)(v & 0x7F); return 3;
}
inline uint8_t buildIdRead(uint8_t* f) { f[0] = 0xFF; f[1] = 0; f[2] = 0; f[3] = 0; return 4; }
inline uint8_t buildIdWrite(uint8_t* f, uint8_t id) {
    f[0] = (uint8_t)(0xE0 | (id & 0x1F)); f[1] = 1; f[2] = 1; f[3] = 1; return 4;
}
inline uint8_t readReplyLen(uint8_t sc) { return sc == SC_EEPROM ? (uint8_t)(2 + EEPROM_LEN) : 3; }

// Rentang nilai tulis yang sah per sub-perintah (EEPROM tidak ditulis lewat alat ini).
inline bool paramWritable(uint8_t sc, uint8_t v) {
    switch (sc) {
        case SC_STRETCH: case SC_SPEED: case SC_TEMP: return v >= 1 && v <= 127;
        case SC_CURRENT: return v >= 1 && v <= 63;
        default: return false;
    }
}

// Echo half-duplex: bila awal buffer identik dengan frame terkirim, reply mulai sesudahnya.
inline size_t echoOffset(const uint8_t* buf, size_t n, const uint8_t* sent, size_t sentLen) {
    return (n >= sentLen && memcmp(buf, sent, sentLen) == 0) ? sentLen : 0;
}
// Reply lengkap? (dipakai HAL untuk early-exit transaksi)
inline bool replyComplete(const uint8_t* buf, size_t n, const uint8_t* sent, size_t sentLen, size_t want) {
    return n - echoOffset(buf, n, sent, sentLen) >= want;
}
// Reply READ: out menerima 1 byte (sc 1..4) atau 64 byte (EEPROM).
inline bool parseRead(const uint8_t* buf, size_t n, const uint8_t* sent, uint8_t sc, uint8_t* out) {
    const size_t off = echoOffset(buf, n, sent, 2);
    const size_t want = readReplyLen(sc);
    if (n < off + want) return false;
    const uint8_t* r = buf + off;
    if (r[0] != (uint8_t)(sent[0] & 0x7F) || r[1] != sc) return false;
    for (size_t i = 2; i < want; ++i) if (r[i] & 0x80) return false;
    memcpy(out, r + 2, want - 2);
    return true;
}
inline bool parseWrite(const uint8_t* buf, size_t n, const uint8_t* sent) {
    const size_t off = echoOffset(buf, n, sent, 3);
    if (n < off + 3) return false;
    const uint8_t* r = buf + off;
    return r[0] == (uint8_t)(sent[0] & 0x7F) && r[1] == sent[1] && r[2] == sent[2];
}
inline bool parseId(const uint8_t* buf, size_t n, const uint8_t* sent, uint8_t* id) {
    const size_t off = echoOffset(buf, n, sent, 4);
    if (n < off + 1 || (buf[off] & 0xE0) != 0xE0) return false;
    if (sent[0] != 0xFF && buf[off] != sent[0]) return false;   // tulis ID: reply = ID baru
    *id = (uint8_t)(buf[off] & 0x1F);
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
    // Fase 4A (mode TOOL): satu transaksi di satu bus. Kirim tx, kumpulkan byte (termasuk echo)
    // sampai ics::replyComplete(rx, n, tx, txLen, want) atau timeoutUs habis.
    // Kembalikan jumlah byte di rx; -1 = HAL tidak mendukung.
    virtual int transact(uint8_t bus, const uint8_t* tx, uint8_t txLen, uint8_t* rx, uint8_t rxMax,
                         uint8_t want, uint32_t timeoutUs) {
        (void)bus; (void)tx; (void)txLen; (void)rx; (void)rxMax; (void)want; (void)timeoutUs;
        return -1;
    }
};

}  // namespace la
