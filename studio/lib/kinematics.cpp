#include "kinematics.h"
#include "leumpang/controller.h"

#include <cstring>
#include <initializer_list>

namespace la::studio {

Quat Quat::axisAngle(const Vec3& a, float rad) {
    const Vec3 u = a.unit();
    const float s = std::sin(rad * 0.5f);
    return {std::cos(rad * 0.5f), u.x * s, u.y * s, u.z * s};
}
Quat Quat::operator*(const Quat& q) const {
    return {w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w};
}
Vec3 Quat::rotate(const Vec3& v) const {
    const Vec3 u{x, y, z};
    const Vec3 t = u.cross(v) * 2.0f;
    return v + t * w + u.cross(t);
}

// ServoPositions -> [bus][slot] (urutan jointSrc di controller.cpp)
void homeSrc(JointSrc out) {
    const ServoPositions& h = HOME_POSE;
    for (int b = 0; b < 2; ++b) {
        out[b][0] = b == 0 ? h.waist : h.head;
        out[b][1] = h.U0[b]; out[b][2] = h.U1[b]; out[b][3] = h.U2[b]; out[b][4] = h.E[b];
        out[b][5] = h.K2[b]; out[b][6] = h.K1[b]; out[b][7] = h.K0[b]; out[b][8] = h.H[b];
        out[b][9] = h.A0[b]; out[b][10] = h.A1[b];
    }
}

int16_t rawToSrc(uint8_t b, uint8_t s, int16_t raw, int16_t trim) {
    const JointDef& j = JOINTS[b][s];
    return (int16_t)((int32_t)j.sign * ((int32_t)raw - j.center) - trim);
}
int16_t srcToRaw(uint8_t b, uint8_t s, int16_t src, int16_t trim) {
    const JointDef& j = JOINTS[b][s];
    int32_t v = (int32_t)j.center + (int32_t)j.sign * ((int32_t)src + trim);
    return (int16_t)(v < j.lo ? j.lo : (v > j.hi ? j.hi : v));
}
bool srcWithinLimits(uint8_t b, uint8_t s, int16_t src, int16_t trim) {
    const JointDef& j = JOINTS[b][s];
    const int32_t v = (int32_t)j.center + (int32_t)j.sign * ((int32_t)src + trim);
    return v >= j.lo && v <= j.hi;
}
const char* jointName(uint8_t b, uint8_t s) { return JOINT_NAMES[b][s]; }
bool jointByName(const char* name, uint8_t& bus, uint8_t& slot) {
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        if (!strcmp(JOINT_NAMES[b][s], name)) { bus = b; slot = s; return true; }
    return false;
}

int linkIndex(const char* name) {
    for (int i = 0; i < gen::LINK_COUNT; ++i) if (!strcmp(gen::LINKS[i].name, name)) return i;
    return -1;
}

void forward(const JointSrc src, Pose out[gen::LINK_COUNT]) {
    for (int i = 0; i < gen::LINK_COUNT; ++i) {
        const gen::LinkDef& L = gen::LINKS[i];
        Pose parent;                                           // identitas untuk root
        if (L.parent >= 0) parent = out[L.parent];             // parent selalu sebelum anak
        Pose& o = out[i];
        o.p = parent.p + parent.q.rotate(Vec3{L.ox, L.oy, L.oz});
        o.q = parent.q;
        if (L.bus >= 0) {
            const float ang = (float)src[L.bus][L.slot] / gen::CHG_SVA;
            o.q = parent.q * Quat::axisAngle(Vec3{L.ax, L.ay, L.az}, ang);
        }
    }
}

void soleCorners(const Pose& foot, bool rightFoot, Vec3 out[4]) {
    // sesuai gen_model.py: pelat 96 x 58, pusat (9, +-5), bawah sol SOLE_Z
    const float oy = rightFoot ? -5.0f : 5.0f;
    const float xs[2] = {9.0f - 48.0f, 9.0f + 48.0f}, ys[2] = {oy - 29.0f, oy + 29.0f};
    int k = 0;
    for (float x : xs) for (float y : ys) out[k++] = foot.p + foot.q.rotate(Vec3{x, y, gen::SOLE_Z});
}

void world(const JointSrc src, const WorldOpts& o, Pose out[gen::LINK_COUNT]) {
    forward(src, out);
    const Quat base = Quat::axisAngle(Vec3{1, 0, 0}, o.rollRad) * Quat::axisAngle(Vec3{0, 1, 0}, o.pitchRad);
    for (int i = 0; i < gen::LINK_COUNT; ++i) { out[i].p = base.rotate(out[i].p); out[i].q = base * out[i].q; }
    if (!o.ground) return;
    static const int rf = linkIndex("r_foot"), lf = linkIndex("l_foot");
    float minZ = 1e9f;
    Vec3 c[4];
    for (int f : {rf, lf}) {
        soleCorners(out[f], f == rf, c);
        for (const Vec3& v : c) minZ = v.z < minZ ? v.z : minZ;
    }
    for (int i = 0; i < gen::LINK_COUNT; ++i) out[i].p.z -= minZ;
}

}  // namespace la::studio
