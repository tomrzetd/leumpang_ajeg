// =============================================================================
//  Leumpang Studio - dokumen gerak (keyframe) <-> LAM1, pratinjau identik dengan
//  pemutar firmware (Controller::motionStep), dan penyederhanaan rekaman kamera.
// =============================================================================
#pragma once
#include "kinematics.h"
#include "leumpang/motion.h"

#include <string>
#include <vector>

namespace la::studio {

struct Keyframe {
    uint16_t ms = 500;
    uint8_t  interp = MI_COSINE;
    int16_t  j[SERVO_BUSES][SERVO_SLOTS]{};
};

struct MotionDoc {
    std::string name = "gerak";
    bool returnHome = true;
    std::vector<Keyframe> frames;

    uint32_t totalMs() const;
    // Pose pada waktu t (ms sejak mulai), mulai dari pose 'start' (pose robot saat PLAY; READY = HOME).
    void sample(uint32_t tMs, const JointSrc start, JointSrc out) const;
    // Validasi untuk firmware (jumlah frame, durasi, batas sendi dengan trim). Pesan galat Bahasa Indonesia.
    bool validate(const int16_t trim[SERVO_BUSES][SERVO_SLOTS], std::string* err) const;
    bool toMotion(Motion& m, std::string* err) const;
    std::vector<uint8_t> toLam(std::string* err) const;
    static MotionDoc fromMotion(const Motion& m);
    static bool fromLam(const uint8_t* data, size_t len, MotionDoc& out, std::string* err);
};

// Satu sampel rekaman (kamera / robot) pada waktu tMs.
struct PoseSample { uint32_t tMs; int16_t j[SERVO_BUSES][SERVO_SLOTS]; };

// Ramer-Douglas-Peucker multi-sendi: pilih keyframe sesedikit mungkin sehingga interpolasi linear
// antar-keyframe menyimpang <= tolCounts dari rekaman. Toleransi dinaikkan otomatis bila
// hasilnya > maxFrames. Frame keluaran memakai interp LINEAR.
std::vector<Keyframe> simplify(const std::vector<PoseSample>& rec, int tolCounts, int maxFrames = MOTION_MAX_FRAMES);

}  // namespace la::studio
