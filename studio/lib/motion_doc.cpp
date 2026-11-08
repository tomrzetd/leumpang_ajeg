#include "motion_doc.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace la::studio {

static constexpr float kPiF = 3.14159265f;

uint32_t MotionDoc::totalMs() const {
    uint32_t t = 0;
    for (const Keyframe& f : frames) t += f.ms;
    return t;
}

void MotionDoc::sample(uint32_t tMs, const JointSrc start, JointSrc out) const {
    int16_t prev[SERVO_BUSES][SERVO_SLOTS];
    memcpy(prev, start, sizeof(prev));
    uint32_t t0 = 0;
    for (const Keyframe& f : frames) {
        const uint32_t d = f.ms ? f.ms : 1;
        if (tMs < t0 + d) {
            const float t = (float)(tMs - t0) / (float)d;
            float a = t;                                           // sama dengan Controller::motionStep
            if (f.interp == MI_COSINE) a = 0.5f * (1.0f - std::cos(kPiF * t));
            else if (f.interp == MI_STEP) a = 1.0f;
            for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s)
                out[b][s] = (int16_t)((float)prev[b][s] + ((float)f.j[b][s] - (float)prev[b][s]) * a);
            return;
        }
        memcpy(prev, f.j, sizeof(prev));
        t0 += d;
    }
    memcpy(out, prev, sizeof(prev));
}

bool MotionDoc::validate(const int16_t trim[SERVO_BUSES][SERVO_SLOTS], std::string* err) const {
    char buf[160];
    auto fail = [&](const char* m) { if (err) *err = m; return false; };
    if (frames.empty()) return fail("belum ada keyframe");
    if ((int)frames.size() > MOTION_MAX_FRAMES) {
        snprintf(buf, sizeof buf, "terlalu banyak keyframe (%zu, maks %d)", frames.size(), MOTION_MAX_FRAMES);
        return fail(buf);
    }
    for (size_t i = 0; i < frames.size(); ++i) {
        const Keyframe& f = frames[i];
        if (f.ms < MOTION_MIN_FRAME_MS || f.ms > MOTION_MAX_FRAME_MS) {
            snprintf(buf, sizeof buf, "frame %zu: durasi %u ms di luar %u..%u", i + 1, (unsigned)f.ms,
                     (unsigned)MOTION_MIN_FRAME_MS, (unsigned)MOTION_MAX_FRAME_MS);
            return fail(buf);
        }
        for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
            if (!srcWithinLimits(b, s, f.j[b][s], trim ? trim[b][s] : 0)) {
                snprintf(buf, sizeof buf, "frame %zu: %s = %d di luar batas sendi", i + 1, jointName(b, s), f.j[b][s]);
                return fail(buf);
            }
    }
    return true;
}

bool MotionDoc::toMotion(Motion& m, std::string* err) const {
    if (!validate(nullptr, err)) return false;
    memset(&m, 0, sizeof(m));
    snprintf(m.name, sizeof(m.name), "%s", name.c_str());
    m.nFrames = (uint8_t)frames.size();
    m.flags = returnHome ? MF_RETURN_HOME : 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        m.f[i].durMs = frames[i].ms;
        m.f[i].interp = frames[i].interp;
        memcpy(m.f[i].joint, frames[i].j, sizeof(m.f[i].joint));
    }
    return true;
}

std::vector<uint8_t> MotionDoc::toLam(std::string* err) const {
    Motion m;
    if (!toMotion(m, err)) return {};
    std::vector<uint8_t> out(MOTION_MAX_BYTES);
    out.resize(motionEncode(m, out.data()));
    return out;
}

MotionDoc MotionDoc::fromMotion(const Motion& m) {
    MotionDoc d;
    d.name = m.name;
    d.returnHome = (m.flags & MF_RETURN_HOME) != 0;
    for (int i = 0; i < m.nFrames; ++i) {
        Keyframe k;
        k.ms = m.f[i].durMs; k.interp = m.f[i].interp;
        memcpy(k.j, m.f[i].joint, sizeof(k.j));
        d.frames.push_back(k);
    }
    return d;
}

bool MotionDoc::fromLam(const uint8_t* data, size_t len, MotionDoc& out, std::string* err) {
    Motion m;
    const MotionError e = motionDecode(data, len, m);
    if (e != MotionError::OK) { if (err) *err = std::string("LAM1 tidak valid: ") + motionErrorName(e); return false; }
    out = fromMotion(m);
    return true;
}

// ---------------------------------------------------------------------------
static int deviation(const PoseSample& a, const PoseSample& b, const PoseSample& p) {
    const float span = (float)(b.tMs - a.tMs);
    const float u = span > 0 ? (float)(p.tMs - a.tMs) / span : 0.0f;
    int worst = 0;
    for (int bb = 0; bb < 2; ++bb) for (int s = 0; s < SERVO_SLOTS; ++s) {
        const float lin = (float)a.j[bb][s] + ((float)b.j[bb][s] - (float)a.j[bb][s]) * u;
        const int d = (int)std::lround(std::fabs(lin - (float)p.j[bb][s]));
        worst = d > worst ? d : worst;
    }
    return worst;
}

static void rdp(const std::vector<PoseSample>& r, size_t i0, size_t i1, int tol, std::vector<char>& keep) {
    if (i1 <= i0 + 1) return;
    int worst = -1; size_t idx = i0;
    for (size_t i = i0 + 1; i < i1; ++i) {
        const int d = deviation(r[i0], r[i1], r[i]);
        if (d > worst) { worst = d; idx = i; }
    }
    if (worst > tol) { keep[idx] = 1; rdp(r, i0, idx, tol, keep); rdp(r, idx, i1, tol, keep); }
}

std::vector<Keyframe> simplify(const std::vector<PoseSample>& rec, int tol, int maxFrames) {
    std::vector<Keyframe> out;
    if (rec.size() < 2) return out;
    if (tol < 1) tol = 1;
    std::vector<char> keep;
    for (int iter = 0; iter < 24; ++iter) {
        keep.assign(rec.size(), 0);
        keep.front() = keep.back() = 1;
        rdp(rec, 0, rec.size() - 1, tol, keep);
        // jaga durasi frame <= MOTION_MAX_FRAME_MS: sisipkan titik antara bila celah terlalu panjang
        size_t last = 0;
        for (size_t i = 1; i < rec.size(); ++i) {
            if (rec[i].tMs - rec[last].tMs > MOTION_MAX_FRAME_MS) keep[i - 1] = 1;
            if (keep[i]) last = i; else if (keep[i - 1]) last = i - 1;
        }
        int n = 0;
        for (size_t i = 1; i < rec.size(); ++i) n += keep[i];
        if (n + 1 <= maxFrames) break;          // +1 = keyframe awal
        tol = tol * 3 / 2 + 1;
    }
    // keyframe pertama = sampel 0 dicapai dalam 500 ms (dari pose robot saat PLAY)
    Keyframe k0; k0.ms = 500; k0.interp = MI_COSINE; memcpy(k0.j, rec[0].j, sizeof(k0.j));
    out.push_back(k0);
    size_t prev = 0;
    for (size_t i = 1; i < rec.size(); ++i) {
        if (!keep[i]) continue;
        Keyframe k;
        uint32_t d = rec[i].tMs - rec[prev].tMs;
        k.ms = (uint16_t)std::clamp<uint32_t>(d, MOTION_MIN_FRAME_MS, MOTION_MAX_FRAME_MS);
        k.interp = MI_LINEAR;
        memcpy(k.j, rec[i].j, sizeof(k.j));
        out.push_back(k);
        prev = i;
    }
    if ((int)out.size() > maxFrames) out.resize((size_t)maxFrames);
    return out;
}

}  // namespace la::studio
