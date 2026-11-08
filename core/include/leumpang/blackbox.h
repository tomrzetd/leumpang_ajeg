// =============================================================================
//  Blackbox: rekaman telemetry 100 Hz untuk SD card (dan pemutaran ulang di twin)
//
//  Thread kontrol mendorong 1 record per siklus ke SpscRing (lock-free, tanpa blok).
//  Task penyimpanan prioritas rendah menguras ring ke file. Ring penuh = record
//  dibuang dan dihitung (blackboxDropped), kontrol TIDAK pernah menunggu SD.
//
//  File .lbb: header 16 B { u32 'LABB', u16 ver, u16 recSize, u32 startMs, u32 rsv }
//             lalu record 64 B berurutan (little-endian), lihat BlackboxRecord::encode.
// =============================================================================
#pragma once
#include <atomic>
#include <string.h>
#include "leumpang/types.h"

namespace la {

template <typename T, uint32_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0, "N harus pangkat 2");
public:
    bool push(const T& v) {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= N) { ++dropped_; return false; }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }
    bool pop(T& v) {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        v = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }
    uint32_t size() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
    uint32_t dropped() const { return dropped_; }
private:
    T buf_[N];
    std::atomic<uint32_t> head_{0}, tail_{0};
    uint32_t dropped_ = 0;     // hanya ditulis produsen
};

constexpr uint32_t BLACKBOX_MAGIC    = 0x4242414C;   // "LABB"
constexpr uint16_t BLACKBOX_VER      = 1;
constexpr size_t   BLACKBOX_REC_SIZE = 64;

struct BlackboxRecord {
    uint32_t ms;
    uint16_t mode;
    int16_t  rollCdeg, pitchCdeg;       // centi-derajat
    uint16_t loopUs;
    int16_t  cmd[SERVO_BUSES][SERVO_SLOTS];
    uint32_t fbValidBits;               // bit (bus*11+slot)
    uint8_t  flags;                     // bit0 imuFresh, bit1 jiku kiri, bit2 intent aktif
    uint8_t  fwct;
    uint16_t autoHx10;

    void encode(uint8_t* o) const {      // tepat 64 B
        size_t i = 0;
        auto u8  = [&](uint8_t v)  { o[i++] = v; };
        auto u16 = [&](uint16_t v) { o[i++] = (uint8_t)v; o[i++] = (uint8_t)(v >> 8); };
        auto u32 = [&](uint32_t v) { u16((uint16_t)v); u16((uint16_t)(v >> 16)); };
        u32(ms); u16(mode); u16((uint16_t)rollCdeg); u16((uint16_t)pitchCdeg); u16(loopUs);
        for (int b = 0; b < SERVO_BUSES; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) u16((uint16_t)cmd[b][s]);
        u32(fbValidBits); u8(flags); u8(fwct); u16(autoHx10);
        while (i < BLACKBOX_REC_SIZE) o[i++] = 0;
    }
};

using BlackboxRing = SpscRing<BlackboxRecord, 256>;   // 2,5 s penyangga pada 100 Hz

inline void blackboxHeader(uint8_t* o, uint32_t startMs) {
    memset(o, 0, 16);
    const uint32_t m = BLACKBOX_MAGIC;
    memcpy(o, &m, 4);                    // target semua little-endian
    o[4] = (uint8_t)BLACKBOX_VER; o[5] = 0;
    o[6] = (uint8_t)BLACKBOX_REC_SIZE; o[7] = 0;
    memcpy(o + 8, &startMs, 4);
}

}  // namespace la
