#include "leumpang/motion.h"
#include "leumpang/lap.h"   // crc16

#include <string.h>

namespace la {

const char* motionErrorName(MotionError e) {
    switch (e) {
        case MotionError::OK:       return "OK";
        case MotionError::SIZE:     return "ukuran salah";
        case MotionError::MAGIC:    return "bukan LAM1";
        case MotionError::VERSION:  return "versi tidak didukung";
        case MotionError::FRAMES:   return "jumlah frame 1..64";
        case MotionError::BAD_CRC:      return "CRC salah";
        case MotionError::DURATION: return "durasi frame 20..10000 ms";
        case MotionError::INTERP:   return "interpolasi tidak dikenal";
    }
    return "?";
}

static inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline void wr16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

MotionError motionDecode(const uint8_t* d, size_t n, Motion& m) {
    if (n < MOTION_HDR_BYTES + 2) return MotionError::SIZE;
    const uint32_t magic = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    if (magic != MOTION_MAGIC) return MotionError::MAGIC;
    if (d[4] != 1) return MotionError::VERSION;
    const uint8_t nf = d[5];
    if (nf < 1 || nf > MOTION_MAX_FRAMES) return MotionError::FRAMES;
    const size_t need = MOTION_HDR_BYTES + (size_t)nf * MOTION_FRAME_BYTES + 2;
    if (n != need) return MotionError::SIZE;
    if (lap::crc16(d, n - 2) != rd16(d + n - 2)) return MotionError::BAD_CRC;
    m.nFrames = nf;
    m.flags = rd16(d + 6);
    memcpy(m.name, d + 8, 16); m.name[16] = 0;
    const uint8_t* p = d + MOTION_HDR_BYTES;
    for (int i = 0; i < nf; ++i, p += MOTION_FRAME_BYTES) {
        MotionFrame& f = m.f[i];
        f.durMs = rd16(p);
        f.interp = p[2];
        if (f.durMs < MOTION_MIN_FRAME_MS || f.durMs > MOTION_MAX_FRAME_MS) return MotionError::DURATION;
        if (f.interp > MI_STEP) return MotionError::INTERP;
        for (int b = 0; b < SERVO_BUSES; ++b) for (int s = 0; s < SERVO_SLOTS; ++s)
            f.joint[b][s] = (int16_t)rd16(p + 4 + 2 * (b * SERVO_SLOTS + s));
    }
    return MotionError::OK;
}

size_t motionEncode(const Motion& m, uint8_t* o) {
    memset(o, 0, MOTION_HDR_BYTES);
    const uint32_t magic = MOTION_MAGIC;
    o[0] = (uint8_t)magic; o[1] = (uint8_t)(magic >> 8); o[2] = (uint8_t)(magic >> 16); o[3] = (uint8_t)(magic >> 24);
    o[4] = 1; o[5] = m.nFrames;
    wr16(o + 6, m.flags);
    memcpy(o + 8, m.name, strnlen(m.name, 16));
    uint8_t* p = o + MOTION_HDR_BYTES;
    for (int i = 0; i < m.nFrames; ++i, p += MOTION_FRAME_BYTES) {
        const MotionFrame& f = m.f[i];
        wr16(p, f.durMs); p[2] = f.interp; p[3] = 0;
        for (int b = 0; b < SERVO_BUSES; ++b) for (int s = 0; s < SERVO_SLOTS; ++s)
            wr16(p + 4 + 2 * (b * SERVO_SLOTS + s), (uint16_t)f.joint[b][s]);
    }
    const size_t n = MOTION_HDR_BYTES + (size_t)m.nFrames * MOTION_FRAME_BYTES;
    wr16(o + n, lap::crc16(o, n));
    return n + 2;
}

// ---- MotionStore ----------------------------------------------------------
bool MotionStore::beginWrite(int slot) {
    if (!validSlot(slot)) return false;
    uint8_t s = st_[slot].load(std::memory_order_acquire);
    while (s != PLAYING) {
        if (st_[slot].compare_exchange_weak(s, WRITING, std::memory_order_acq_rel)) return true;
    }
    return false;
}
void MotionStore::commit(int slot, bool ok) {
    if (validSlot(slot)) st_[slot].store(ok ? READY : EMPTY, std::memory_order_release);
}
bool MotionStore::load(int slot, const uint8_t* d, size_t n, MotionError* err) {
    if (!beginWrite(slot)) { if (err) *err = MotionError::OK; return false; }
    const MotionError e = motionDecode(d, n, m_[slot]);
    if (err) *err = e;
    commit(slot, e == MotionError::OK);
    return e == MotionError::OK;
}
const Motion* MotionStore::acquire(int slot) {
    if (!validSlot(slot)) return nullptr;
    uint8_t expect = READY;
    return st_[slot].compare_exchange_strong(expect, PLAYING, std::memory_order_acq_rel) ? &m_[slot] : nullptr;
}
void MotionStore::release(int slot) {
    if (validSlot(slot)) {
        uint8_t expect = PLAYING;
        st_[slot].compare_exchange_strong(expect, READY, std::memory_order_acq_rel);
    }
}

}  // namespace la
