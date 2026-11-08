// =============================================================================
//  Leumpang Studio - kinematika maju (FK) digital twin. Tanpa Qt, diuji di host.
//  Model (link, sumbu, mesh) dibangkitkan tools/gen_model.py -> robot_model_gen.h.
// =============================================================================
#pragma once
#include "robot_model_gen.h"
#include "leumpang/types.h"

#include <cmath>

namespace la::studio {

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    float norm() const { return std::sqrt(dot(*this)); }
    Vec3 unit() const { const float n = norm(); return n > 1e-9f ? *this * (1.0f / n) : Vec3{}; }
};

struct Quat {                                    // w + xi + yj + zk
    float w = 1, x = 0, y = 0, z = 0;
    static Quat axisAngle(const Vec3& a, float rad);
    Quat operator*(const Quat& q) const;
    Vec3 rotate(const Vec3& v) const;
    Quat conj() const { return {w, -x, -y, -z}; }
};

struct Pose { Vec3 p; Quat q; };                   // posisi (mm) + orientasi di frame dunia

// Nilai sendi dalam ruang src (offset hitungan, sama dengan 'kin' telemetry & joint LAM1).
using JointSrc = int16_t[SERVO_BUSES][SERVO_SLOTS];

// HOME_POSE core dalam bentuk [bus][slot].
void homeSrc(JointSrc out);
// Hitungan servo mentah <-> src, memakai profil JOINTS core (center, sign) dan trim.
int16_t rawToSrc(uint8_t bus, uint8_t slot, int16_t raw, int16_t trim);
int16_t srcToRaw(uint8_t bus, uint8_t slot, int16_t src, int16_t trim);
bool    srcWithinLimits(uint8_t bus, uint8_t slot, int16_t src, int16_t trim);
const char* jointName(uint8_t bus, uint8_t slot);
// Cari (bus, slot) dari nama sendi core ("R ELBOW"), -1 bila tidak ada.
bool jointByName(const char* name, uint8_t& bus, uint8_t& slot);

// FK: pose tiap link relatif frame pelvis.
void forward(const JointSrc src, Pose out[gen::LINK_COUNT]);

// Pose dunia: pelvis diputar roll (sumbu X) lalu pitch (sumbu Y) dalam radian, lalu digeser
// vertikal agar titik sol terendah z = 0. Tanda roll/pitch IMU robot bisa dibalik di studio.
struct WorldOpts { float rollRad = 0, pitchRad = 0; bool ground = true; };
void world(const JointSrc src, const WorldOpts& o, Pose out[gen::LINK_COUNT]);

// Empat sudut sol kaki (frame dunia, dari pose link *_foot).
void soleCorners(const Pose& foot, bool rightFoot, Vec3 out[4]);

int linkIndex(const char* name);

}  // namespace la::studio
