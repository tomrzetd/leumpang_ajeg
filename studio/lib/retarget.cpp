#include "retarget.h"

#include <algorithm>
#include <cstring>

namespace la::studio {

static constexpr float kPiF = 3.14159265f;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static int16_t toCounts(float rad) { return (int16_t)std::lround(clampf(rad * gen::CHG_SVA, -30000.0f, 30000.0f)); }

Vec3 segmentDir(const Keypoint& a, const Keypoint& b, float refLenPx) {
    if (refLenPx < 1.0f) return {};
    // kamera menghadap orang: kiri tubuh = +x gambar, atas = -y gambar
    float lat = (b.x - a.x) / refLenPx, up = -(b.y - a.y) / refLenPx;
    const float planar = std::sqrt(lat * lat + up * up);
    if (planar > 1.0f) { lat /= planar; up /= planar; }
    const float fwd = std::sqrt(std::max(0.0f, 1.0f - lat * lat - up * up));
    return Vec3{fwd, lat, up}.unit();
}

bool shoulderAngles(const Vec3& d, int robotBus, float& pitch, float& roll) {
    // FK lengan atas (gen_model): arah = (cos r cos p, +-sin r, cos r sin p); kanan: lateral = -sin r
    const float outward = robotBus == 0 ? -d.y : d.y;
    roll = std::asin(clampf(outward, -1.0f, 1.0f));
    const float h = std::sqrt(d.x * d.x + d.z * d.z);
    if (h < 1e-3f) { pitch = 0; return std::fabs(outward) > 0.9f; }
    pitch = std::atan2(d.z, d.x);
    return true;
}

bool Retargeter::solveArm(const Keypoint kp[KP_COUNT], bool humanRight, int robotBus, float shoulderW,
                          JointSrc pose) const {
    const Keypoint& sh = kp[humanRight ? R_SHOULDER : L_SHOULDER];
    const Keypoint& el = kp[humanRight ? R_ELBOW : L_ELBOW];
    const Keypoint& wr = kp[humanRight ? R_WRIST : L_WRIST];
    if (sh.score < opts.minScore || el.score < opts.minScore) return false;

    Vec3 du = segmentDir(sh, el, opts.upperRatio * shoulderW);
    const bool haveFore = wr.score >= opts.minScore;
    Vec3 df = haveFore ? segmentDir(el, wr, opts.foreRatio * shoulderW) : du;
    if (opts.mirror) { du.y = -du.y; df.y = -df.y; }      // cermin di bidang sagital

    float pitch = 0, roll = 0;
    if (!shoulderAngles(du, robotBus, pitch, roll)) return false;
    pose[robotBus][1] = toCounts(pitch);
    pose[robotBus][2] = toCounts(roll);

    // siku: besar tekukan dari sudut antar segmen; arah tekuk dipilih lewat yaw lengan atas (U2)
    const float bend = haveFore ? std::acos(clampf(du.dot(df), -1.0f, 1.0f)) : 0.0f;
    pose[robotBus][4] = toCounts(-clampf(bend, 0.0f, 150.0f * kPiF / 180.0f));
    pose[robotBus][3] = 0;
    if (haveFore && bend > 10.0f * kPiF / 180.0f) {
        static const int fa[2] = {linkIndex("r_forearm"), linkIndex("l_forearm")};
        Pose P[gen::LINK_COUNT];
        float best = 1e9f; int16_t bestYaw = 0;
        for (int deg = -90; deg <= 90; deg += 5) {
            pose[robotBus][3] = toCounts((float)deg * kPiF / 180.0f);
            forward(pose, P);
            const Vec3 f = P[fa[robotBus]].q.rotate(Vec3{1, 0, 0});
            const float e = (f - df).norm();
            if (e < best) { best = e; bestYaw = pose[robotBus][3]; }
        }
        pose[robotBus][3] = bestYaw;
    }
    return true;
}

RetargetResult Retargeter::solve(const Keypoint kp[KP_COUNT], const JointSrc base) const {
    RetargetResult r;
    memcpy(r.src, base, sizeof(r.src));
    const Keypoint &ls = kp[L_SHOULDER], &rs = kp[R_SHOULDER];
    if (ls.score < opts.minScore || rs.score < opts.minScore) return r;
    const float shoulderW = std::hypot(ls.x - rs.x, ls.y - rs.y);
    if (shoulderW < 8.0f) return r;

    for (int humanSide = 0; humanSide < 2; ++humanSide) {                   // 0 = kanan manusia
        const bool humanRight = humanSide == 0;
        const int robotBus = opts.mirror ? (humanRight ? 1 : 0) : (humanRight ? 0 : 1);
        if (solveArm(kp, humanRight, robotBus, shoulderW, r.src)) {
            r.armOk[robotBus] = true;
            for (int s = 1; s <= 4; ++s) r.mask |= 1u << (robotBus * SERVO_SLOTS + s);
        }
    }
    if (opts.head && kp[NOSE].score >= opts.minScore) {
        const float midX = 0.5f * (ls.x + rs.x);
        float off = (kp[NOSE].x - midX) / (0.5f * shoulderW);              // + = menoleh ke kiri tubuh
        if (opts.mirror) off = -off;
        r.src[1][0] = toCounts(std::asin(clampf(off * 0.9f, -0.9f, 0.9f)));
        r.mask |= 1u << (1 * SERVO_SLOTS + 0);
    }
    // jaga dalam batas sendi (trim 0; robot tetap memeriksa ulang)
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        if (r.mask & (1u << (b * SERVO_SLOTS + s))) r.src[b][s] = rawToSrc(b, s, srcToRaw(b, s, r.src[b][s], 0), 0);
    return r;
}

bool Retargeter::calibrateTPose(const Keypoint kp[KP_COUNT]) {
    for (int k : {L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST})
        if (kp[k].score < opts.minScore) return false;
    const float w = std::hypot(kp[L_SHOULDER].x - kp[R_SHOULDER].x, kp[L_SHOULDER].y - kp[R_SHOULDER].y);
    if (w < 8.0f) return false;
    auto len = [&](int a, int b) { return std::hypot(kp[a].x - kp[b].x, kp[a].y - kp[b].y); };
    opts.upperRatio = 0.5f * (len(L_SHOULDER, L_ELBOW) + len(R_SHOULDER, R_ELBOW)) / w;
    opts.foreRatio  = 0.5f * (len(L_ELBOW, L_WRIST) + len(R_ELBOW, R_WRIST)) / w;
    return opts.upperRatio > 0.2f && opts.foreRatio > 0.2f;
}

void JointFilter::apply(JointSrc v, uint32_t mask, float dt) {
    auto alpha = [dt](float fc) { const float tau = 1.0f / (2.0f * kPiF * fc); return 1.0f / (1.0f + tau / dt); };
    if (dt <= 0) dt = 1.0f / 30.0f;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
        const bool on = mask & (1u << (b * SERVO_SLOTS + s));
        const float x = (float)v[b][s];
        if (!init_ || !on) { x_[b][s] = x; dx_[b][s] = 0; continue; }
        const float dxRaw = (x - x_[b][s]) / dt;
        dx_[b][s] += alpha(dCutoffHz) * (dxRaw - dx_[b][s]);
        const float fc = minCutoffHz + beta * std::fabs(dx_[b][s]);
        x_[b][s] += alpha(fc) * (x - x_[b][s]);
        v[b][s] = (int16_t)std::lround(x_[b][s]);
    }
    init_ = true;
}

}  // namespace la::studio
