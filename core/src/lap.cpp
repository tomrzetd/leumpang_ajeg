#include "leumpang/lap.h"

namespace la {
namespace lap {

uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc) {
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)((uint16_t)d[i] << 8);
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t cobsEncode(const uint8_t* in, size_t n, uint8_t* out) {
    size_t r = 0, codeIdx = 0, w = 1;
    uint8_t code = 1;
    while (r < n) {
        if (in[r] == 0) {
            out[codeIdx] = code; codeIdx = w++; code = 1;
        } else {
            out[w++] = in[r]; ++code;
            if (code == 0xFF) { out[codeIdx] = code; codeIdx = w++; code = 1; }
        }
        ++r;
    }
    out[codeIdx] = code;
    return w;
}

size_t cobsDecode(const uint8_t* in, size_t n, uint8_t* out) {
    size_t r = 0, w = 0;
    while (r < n) {
        const uint8_t code = in[r];
        if (code == 0 || r + code > n) return 0;
        ++r;
        for (uint8_t i = 1; i < code; ++i) {
            if (in[r] == 0) return 0;
            out[w++] = in[r++];
        }
        if (code != 0xFF && r < n) out[w++] = 0;
    }
    return w;
}

size_t encodeFrame(const Packet& p, uint8_t* out, size_t cap) {
    if (p.len > MAX_BODY) return 0;
    uint8_t raw[MAX_RAW];
    raw[0] = p.type; raw[1] = p.seq;
    memcpy(raw + 2, p.body, p.len);
    const size_t n = (size_t)p.len + 2;
    const uint16_t c = crc16(raw, n);
    raw[n] = (uint8_t)c; raw[n + 1] = (uint8_t)(c >> 8);
    const size_t need = (n + 2) + (n + 2) / 254 + 2;
    if (cap < need) return 0;
    const size_t m = cobsEncode(raw, n + 2, out);
    out[m] = 0;
    return m + 1;
}

bool Decoder::feed(uint8_t b, Packet& out) {
    if (b != 0) {
        if (n_ < sizeof(buf_)) buf_[n_++] = b;
        else overflow_ = true;
        return false;
    }
    // delimiter
    const size_t n = n_;
    const bool ovf = overflow_;
    n_ = 0; overflow_ = false;
    if (n == 0) return false;
    if (ovf) { ++overflows; return false; }
    uint8_t raw[MAX_FRAME];
    const size_t m = cobsDecode(buf_, n, raw);
    if (m < 4 || m > MAX_RAW) { ++cobsErrors; return false; }
    const uint16_t got = (uint16_t)(raw[m - 2] | (raw[m - 1] << 8));
    if (crc16(raw, m - 2) != got) { ++crcErrors; return false; }
    out.type = raw[0]; out.seq = raw[1]; out.len = (uint8_t)(m - 4);
    memcpy(out.body, raw + 2, out.len);
    ++frames;
    return true;
}

void fillTelemetry(const Telemetry& t, uint32_t savedGen, TelemetryMsg& m) {
    m.ms = t.ms; m.mode = (uint16_t)t.mode;
    m.flags = (uint16_t)((t.imuFreshFlag ? TF_IMU_FRESH : 0) | (t.imuInit ? TF_IMU_INIT : 0) |
                         (t.calDone ? TF_CAL_DONE : 0) | (t.cfgGen != savedGen ? TF_PERSIST_PENDING : 0) |
                         (t.jikuasi ? TF_JIKU_LEFT : 0));
    m.rollDeg = t.roll * kRad2Deg; m.pitchDeg = t.pitch * kRad2Deg;
    m.imuAgeMs = (int16_t)clampi(t.imuAgeMs, -1, 32767);
    m.imuHz = t.imuHz; m.fwct = t.fwct; m.autoH = t.autoH;
    m.loopAvgUs = (uint16_t)clampi((int32_t)t.loopAvgUs, 0, 65535);
    m.loopMaxUs = (uint16_t)clampi((int32_t)t.loopMaxUs, 0, 65535);
    m.servoPhaseUs = (uint16_t)clampi((int32_t)t.servoPhaseUs, 0, 65535);
    m.overruns = t.loopOverruns;
    uint8_t ok = 0;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) if (t.fbValid[b][s] && t.fbAgeMs[b][s] < 700) ++ok;
    m.fbOk = ok; m.cfgGen = t.cfgGen;
}

void fillServoState(const Telemetry& t, ServoStateMsg& m) {
    memcpy(m.cmd, t.cmd, sizeof(m.cmd)); memcpy(m.fb, t.fb, sizeof(m.fb));
    memcpy(m.trim, t.trim, sizeof(m.trim)); memcpy(m.valid, t.fbValid, sizeof(m.valid));
}

void fillGaitState(const Telemetry& t, GaitStateMsg& m) {
    m.fwct = t.fwct; m.fwctEnd = t.fwctEnd; m.dxi = t.dxi; m.dyi = t.dyi; m.dxis = t.dxis; m.dyis = t.dyis;
    m.autoH = t.autoH; m.fh = t.fh; m.swx = t.swx; m.swy = t.swy; m.swMax = t.swMax; m.waist = t.waist;
    m.jikuasi = t.jikuasi;
    static_assert(sizeof(ServoPositions) == sizeof(m.kin), "ServoPositions harus 22 x int16");
    memcpy(m.kin, &t.kin, sizeof(m.kin));
}

void fillConfig(const RuntimeConfig& c, uint32_t cfgGen, uint32_t savedGen, ConfigMsg& m) {
    m.v[(int)ParamId::FALL_DEG]          = c.fallAngleRad * kRad2Deg;
    m.v[(int)ParamId::REARM_DEG]         = c.rearmAngleRad * kRad2Deg;
    m.v[(int)ParamId::IMU_STALE_MS]      = (float)c.imuStaleUs / 1000.0f;
    m.v[(int)ParamId::SERVO_STEP]        = (float)c.servoMaxStep;
    m.v[(int)ParamId::TRIGGER_ROLL_DEG]  = c.triggerRollRad * kRad2Deg;
    m.v[(int)ParamId::TRIGGER_PITCH_DEG] = c.triggerPitchRad * kRad2Deg;
    m.v[(int)ParamId::FOOT_LIFT]         = c.footLiftMax;
    m.v[(int)ParamId::SWAY_BASE]         = c.swayBase;
    m.v[(int)ParamId::SWAY_GAIN]         = c.swayGain;
    m.v[(int)ParamId::SIGN_ROLL]         = (float)c.imuRollSign;
    m.v[(int)ParamId::SIGN_PITCH]        = (float)c.imuPitchSign;
    m.v[(int)ParamId::IMU_SWAP]          = c.imuSwap ? 1.0f : 0.0f;
    m.cfgGen = cfgGen; m.savedGen = savedGen;
}

}  // namespace lap
}  // namespace la
