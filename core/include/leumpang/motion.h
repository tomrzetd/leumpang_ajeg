// =============================================================================
//  Template gerak (Motion) - format LAM1 + penyimpanan antar-thread
//
//  Format biner LAM1 (little-endian), dipakai di LAP MOTION_* dan file SD:
//     u32 magic 'LAM1' | u8 ver(1) | u8 nFrames | u16 flags | char name[16]
//     nFrames x { u16 durMs | u8 interp | u8 rsv | i16 joint[2][11] }   (48 B)
//     u16 crc16 (CCITT-FALSE atas semua byte sebelumnya)
//  joint = offset servo (ruang "src" yang sama dengan ServoPositions/telemetry kin),
//  urutan bus R slot 0..10 lalu bus L slot 0..10. center/sign/trim ditambahkan controller.
// =============================================================================
#pragma once
#include <atomic>
#include "leumpang/types.h"

namespace la {

constexpr int      MOTION_MAX_FRAMES = 64;
constexpr int      MOTION_SLOTS      = 4;
constexpr uint32_t MOTION_MAGIC      = 0x314D414C;      // "LAM1"
constexpr size_t   MOTION_HDR_BYTES  = 24;
constexpr size_t   MOTION_FRAME_BYTES = 48;
constexpr size_t   MOTION_MAX_BYTES  = MOTION_HDR_BYTES + MOTION_MAX_FRAMES * MOTION_FRAME_BYTES + 2;
constexpr uint16_t MOTION_MIN_FRAME_MS = 20;
constexpr uint16_t MOTION_MAX_FRAME_MS = 10000;

enum MotionFlags : uint16_t { MF_RETURN_HOME = 1u << 0 };   // selesai -> RESET ke HOME
enum MotionInterp : uint8_t { MI_COSINE = 0, MI_LINEAR = 1, MI_STEP = 2 };

struct MotionFrame {
    uint16_t durMs;
    uint8_t  interp;
    int16_t  joint[SERVO_BUSES][SERVO_SLOTS];
};

struct Motion {
    char        name[17];
    uint8_t     nFrames;
    uint16_t    flags;
    MotionFrame f[MOTION_MAX_FRAMES];
};

enum class MotionError : uint8_t { OK, SIZE, MAGIC, VERSION, FRAMES, BAD_CRC, DURATION, INTERP };
const char* motionErrorName(MotionError e);

// Dekode + validasi struktur (bukan batas sendi; itu dicek controller saat play).
MotionError motionDecode(const uint8_t* d, size_t n, Motion& m);
// Enkode ke buffer (>= MOTION_MAX_BYTES). Return panjang.
size_t motionEncode(const Motion& m, uint8_t* out);

// Slot motion yang dibagi thread komunikasi (penulis) dan thread kontrol (pembaca).
//  EMPTY -> WRITING (penulis) -> READY -> PLAYING (kontrol) -> READY
class MotionStore {
public:
    enum State : uint8_t { EMPTY = 0, WRITING = 1, READY = 2, PLAYING = 3 };

    // --- sisi penulis (LAP / loader SD) ---
    bool beginWrite(int slot);                       // false bila slot sedang PLAYING
    Motion* writable(int slot) { return &m_[slot]; }
    void commit(int slot, bool ok);                  // ok -> READY, gagal -> EMPTY
    bool load(int slot, const uint8_t* d, size_t n, MotionError* err = nullptr);

    // --- sisi kontrol ---
    const Motion* acquire(int slot);                 // READY -> PLAYING, nullptr bila tidak siap
    void release(int slot);                          // PLAYING -> READY

    State state(int slot) const { return (State)st_[slot].load(std::memory_order_acquire); }
    static bool validSlot(int s) { return s >= 0 && s < MOTION_SLOTS; }

private:
    Motion m_[MOTION_SLOTS]{};
    std::atomic<uint8_t> st_[MOTION_SLOTS]{};
};

}  // namespace la
