// =============================================================================
//  leumpang-core Controller - port 1:1 dari firmware/esp32s3 V4 control.cpp
//  (rumus UVC/IK/state machine identik; diverifikasi tests/test_parity.cpp)
// =============================================================================
#include "leumpang/controller.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace la {

// ============================================================================
//  Mode & config
// ============================================================================
const char* modeName(Mode m) {
    switch (m) {
        case Mode::RAW_ANGLES: return "RAW_ANGLES";
        case Mode::FALLEN:     return "FALLEN/FREE";
        case Mode::RESET:      return "RESET/HOME";
        case Mode::READY:      return "READY";
        case Mode::BALANCE:    return "BALANCE";
        case Mode::WALK_INIT:  return "WALK_INIT";
        case Mode::WALK_HOLD:  return "WALK_HOLD";
        case Mode::WALK_STEP:  return "WALK_STEP";
        case Mode::WALK_END:   return "WALK_END";
        case Mode::POSE_HOLD:  return "POSE_HOLD";
        case Mode::ANGLE_DISP: return "ANGLE_DISP";
        case Mode::MIRROR_DBG: return "MIRROR_DBG";
        case Mode::WALK_CONT:  return "WALK_CONT";
        case Mode::MOTION_PLAY:return "MOTION_PLAY";
        case Mode::LIVE_POSE:  return "LIVE_POSE";
    }
    return "UNKNOWN";
}
const char* modeShortName(Mode m) {
    switch (m) {
        case Mode::RAW_ANGLES: return "RAW";
        case Mode::FALLEN:     return "FALLEN/FREE";
        case Mode::RESET:      return "RESET/HOME";
        case Mode::READY:      return "READY";
        case Mode::BALANCE:    return "BALANCE";
        case Mode::WALK_INIT:  return "WALK INIT";
        case Mode::WALK_HOLD:  return "WALK HOLD";
        case Mode::WALK_STEP:  return "WALK STEP";
        case Mode::WALK_END:   return "WALK END";
        case Mode::POSE_HOLD:  return "POSE HOLD";
        case Mode::ANGLE_DISP: return "ANGLE";
        case Mode::MIRROR_DBG: return "MIRROR";
        case Mode::WALK_CONT:  return "WALK CONT";
        case Mode::MOTION_PLAY:return "MOTION";
        case Mode::LIVE_POSE:  return "LIVE POSE";
    }
    return "UNKNOWN";
}
bool modeIsGait(Mode m) {
    return m == Mode::WALK_INIT || m == Mode::WALK_HOLD || m == Mode::WALK_STEP || m == Mode::WALK_END ||
           m == Mode::WALK_CONT;
}
bool modePersistSafe(Mode m) {
    return m == Mode::READY || m == Mode::FALLEN || m == Mode::ANGLE_DISP || m == Mode::RAW_ANGLES;
}

RuntimeConfig runtimeConfigDefault() {
    RuntimeConfig c{};
    c.fallAngleRad = FALL_ANGLE_RAD; c.rearmAngleRad = REARM_ANGLE_RAD;
    c.imuStaleUs = (uint32_t)IMU_STALE_US; c.servoMaxStep = SERVO_MAX_STEP;
    c.triggerRollRad = 0.033f; c.triggerPitchRad = 0.044f;
    c.footLiftMax = 35.0f; c.swayBase = 17.0f; c.swayGain = 17.0f;
    c.imuRollSign = +1; c.imuPitchSign = +1; c.imuSwap = false;
    c.gaitEnable = false; c.gaitStepMax = 10.0f; c.gaitYawMaxRad = 5.0f / kRad2Deg;
    c.obstacleStopMm = 250.0f;
    return c;
}

void runtimeConfigSanitize(RuntimeConfig& c) {
    const RuntimeConfig d = runtimeConfigDefault();
    auto fin = [](float v, float def) { return isFiniteVal(v) ? v : def; };
    c.fallAngleRad    = clampf(fin(c.fallAngleRad, d.fallAngleRad), 8.0f / kRad2Deg, 60.0f / kRad2Deg);
    c.rearmAngleRad   = clampf(fin(c.rearmAngleRad, d.rearmAngleRad), 3.0f / kRad2Deg, 30.0f / kRad2Deg);
    c.imuStaleUs      = (uint32_t)clampi((int32_t)c.imuStaleUs, 20000, 500000);
    c.servoMaxStep    = (int16_t)clampi(c.servoMaxStep, 20, 1000);
    c.triggerRollRad  = clampf(fin(c.triggerRollRad, d.triggerRollRad), 0.2f / kRad2Deg, 15.0f / kRad2Deg);
    c.triggerPitchRad = clampf(fin(c.triggerPitchRad, d.triggerPitchRad), 0.2f / kRad2Deg, 15.0f / kRad2Deg);
    c.footLiftMax     = clampf(fin(c.footLiftMax, d.footLiftMax), 0.0f, 60.0f);
    c.swayBase        = clampf(fin(c.swayBase, d.swayBase), 0.0f, 45.0f);
    c.swayGain        = clampf(fin(c.swayGain, d.swayGain), 0.0f, 45.0f);
    c.imuRollSign     = (c.imuRollSign < 0) ? -1 : +1;
    c.imuPitchSign    = (c.imuPitchSign < 0) ? -1 : +1;
    c.imuSwap         = c.imuSwap ? true : false;
    if (c.rearmAngleRad >= c.fallAngleRad) c.rearmAngleRad = 0.5f * c.fallAngleRad;
    c.gaitEnable      = c.gaitEnable ? true : false;
    c.gaitStepMax     = clampf(fin(c.gaitStepMax, d.gaitStepMax), 0.0f, 25.0f);
    c.gaitYawMaxRad   = clampf(fin(c.gaitYawMaxRad, d.gaitYawMaxRad), 0.0f, 15.0f / kRad2Deg);
    c.obstacleStopMm  = clampf(fin(c.obstacleStopMm, d.obstacleStopMm), 0.0f, 1500.0f);
}

static inline int16_t f2i16(float v, int16_t fallback = 0) {
    if (!isFiniteVal(v)) return fallback;
    return (int16_t)clampf(v, -32000.0f, 32000.0f);
}

// ============================================================================
//  Profil sendi KHR-3HV. lo/hi WAJIB dipersempit sesuai batas mekanik.
// ============================================================================
const ServoPositions HOME_POSE = {
    {661, 661}, {0, 0}, {0, 0}, {1322, 1322}, {661, 661}, {0, 0},
    {-2700, -2760}, {0, 0}, {0, 0}, {0, 0}, 0, 0
};

const JointDef JOINTS[SERVO_BUSES][SERVO_SLOTS] = {
    {   // bus R
        {  0, 7500, +1, ICS_POS_MIN, ICS_POS_MAX }, {  1, 4735, -1, ICS_POS_MIN, ICS_POS_MAX },
        {  2,10110, -1, ICS_POS_MIN, ICS_POS_MAX }, {  3, 7500, -1, ICS_POS_MIN, ICS_POS_MAX },
        {  4, 4800, +1, ICS_POS_MIN, ICS_POS_MAX }, {  5, 7500, -1, ICS_POS_MIN, ICS_POS_MAX },
        {  6, 7440, -1, ICS_POS_MIN, ICS_POS_MAX }, {  7, 7450, -1, ICS_POS_MIN, ICS_POS_MAX },
        {  8, 9320, -1, ICS_POS_MIN, ICS_POS_MAX }, {  9, 7920, +1, ICS_POS_MIN, ICS_POS_MAX },
        { 10, 7440, +1, ICS_POS_MIN, ICS_POS_MAX },
    },
    {   // bus L
        {  0, 7500, +1, ICS_POS_MIN, ICS_POS_MAX }, {  1, 9260, +1, ICS_POS_MIN, ICS_POS_MAX },
        {  2, 4850, +1, ICS_POS_MIN, ICS_POS_MAX }, {  3, 7500, +1, ICS_POS_MIN, ICS_POS_MAX },
        {  4,10150, -1, ICS_POS_MIN, ICS_POS_MAX }, {  5, 7410, +1, ICS_POS_MIN, ICS_POS_MAX },
        {  6, 7560, +1, ICS_POS_MIN, ICS_POS_MAX }, {  7, 7500, +1, ICS_POS_MIN, ICS_POS_MAX },
        {  8, 5650, +1, ICS_POS_MIN, ICS_POS_MAX }, {  9, 7100, -1, ICS_POS_MIN, ICS_POS_MAX },
        { 10, 7500, -1, ICS_POS_MIN, ICS_POS_MAX },
    },
};

const char* const JOINT_NAMES[SERVO_BUSES][SERVO_SLOTS] = {
    {"WAIST", "R SHOULDER P", "R SHOULDER R", "R SHOULDER Y", "R ELBOW", "R HIP Y", "R HIP R",
     "R HIP P", "R KNEE", "R ANKLE P", "R ANKLE R"},
    {"HEAD", "L SHOULDER P", "L SHOULDER R", "L SHOULDER Y", "L ELBOW", "L HIP Y", "L HIP R",
     "L HIP P", "L KNEE", "L ANKLE P", "L ANKLE R"},
};

int16_t* Controller::jointSrc(uint8_t b, uint8_t s) {
    ServoPositions& v = servos_;
    switch (s) {
        case 0:  return b == 0 ? &v.waist : &v.head;
        case 1:  return &v.U0[b];
        case 2:  return &v.U1[b];
        case 3:  return &v.U2[b];
        case 4:  return &v.E[b];
        case 5:  return &v.K2[b];
        case 6:  return &v.K1[b];
        case 7:  return &v.K0[b];
        case 8:  return &v.H[b];
        case 9:  return &v.A0[b];
        default: return &v.A1[b];
    }
}

// ============================================================================
//  Init, log
// ============================================================================
void Controller::logf(const char* fmt, ...) {
    if (!plat_.log) return;
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    plat_.log(plat_.ctx, buf);
}

void Controller::init(const Platform& p, const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS],
                      const Options& opt) {
    plat_ = p;
    opt_ = opt;
    cfg_ = cfg;
    runtimeConfigSanitize(cfg_);
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        trim_[b][s] = (int16_t)clampi(trim[b][s], -TRIM_LIMIT, TRIM_LIMIT);
    walk_ = WalkingState();
    servos_ = HOME_POSE;
    walk_.mode = Mode::RESET; walk_.motCt = 100;
    walk_.fhMax = cfg_.footLiftMax; walk_.swMax = 25; walk_.fwctUp = 1;
    initWalkState();
    primeOutputs();
    imuCalReset();
    cal_.done = !opt_.autoCalibrate; cal_.lastSeq = 0;
}

// ============================================================================
//  Servo
// ============================================================================
int16_t Controller::jointTarget(uint8_t b, uint8_t s) {
    const JointDef& j = JOINTS[b][s];
    const int32_t v = (int32_t)j.center + (int32_t)j.sign * ((int32_t)(*jointSrc(b, s)) + (int32_t)trim_[b][s]);
    return (int16_t)clampi(v, j.lo, j.hi);
}
void Controller::primeOutputs() {
    for (uint8_t b = 0; b < 2; ++b)
        for (uint8_t s = 0; s < SERVO_SLOTS; ++s) lastOut_[b][s] = jointTarget(b, s);
}

void Controller::sendSlot(ServoBus& bus, uint8_t slot, bool freeTorque) {
    IcsExchange x{};
    for (uint8_t b = 0; b < 2; ++b)
        ics::buildPosition(x.tx[b], JOINTS[b][slot].id,
                           freeTorque ? ICS_POS_FREE : (uint16_t)clampi(lastOut_[b][slot], ICS_POS_MIN, ICS_POS_MAX));
    bus.exchange(x);
    const int64_t t = now();
    for (uint8_t b = 0; b < 2; ++b) {
        ServoFeedback& fb = fb_[b][slot];
        if (x.got[b]) { fb.pos = x.pos[b]; fb.lastUs = t; fb.okCount++; fb.valid = true; }
        else {
            fb.missCount++;
            if (fb.lastUs == 0 || (t - fb.lastUs) > SERVO_FB_VALID_US) fb.valid = false;
        }
    }
}

void Controller::sendAllServoPositions(ServoBus& bus) {
    const int64_t t0 = now();
    for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        for (uint8_t b = 0; b < 2; ++b) {
            const int16_t tgt = jointTarget(b, s), last = lastOut_[b][s];
            lastOut_[b][s] = (int16_t)(last + clampi((int32_t)tgt - last, -cfg_.servoMaxStep, cfg_.servoMaxStep));
        }
        sendSlot(bus, s, false);
    }
    servoPhaseUs_ = (uint32_t)(now() - t0);
}
void Controller::sendAllServoFree(ServoBus& bus) {
    const int64_t t0 = now();
    for (uint8_t s = 0; s < SERVO_SLOTS; ++s) sendSlot(bus, s, true);
    servoPhaseUs_ = (uint32_t)(now() - t0);
}

uint8_t Controller::syncStateFromServoFeedback() {
    uint8_t synced = 0;
    const int64_t t = now();
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const ServoFeedback& fb = fb_[b][s];
        const JointDef& j = JOINTS[b][s];
        if (!fb.valid || fb.lastUs == 0 || (t - fb.lastUs) > SERVO_FB_VALID_US) continue;
        const int16_t raw = (int16_t)clampi(fb.pos, j.lo, j.hi);
        lastOut_[b][s] = raw;
        const int32_t offs = (int32_t)j.sign * ((int32_t)raw - (int32_t)j.center) - trim_[b][s];
        *jointSrc(b, s) = (int16_t)clampi(offs, -32000, 32000);
        ++synced;
    }
    return synced;
}

void Controller::bootFreeCycle(ServoBus& bus) { sendAllServoFree(bus); }

void Controller::bootFinish(bool syncFromFeedback) {
    if (syncFromFeedback) {
        const uint8_t n = syncStateFromServoFeedback();
        logf("Boot sync: %u/22 posisi servo terbaca%s\r\n", (unsigned)n,
             n < 22 ? " (sisanya diasumsikan HOME - cek jalur RX/EN!)" : "");
    }
    startReset("boot", 100);
}

// ============================================================================
//  Gerak
// ============================================================================
void Controller::movSv(int16_t* cur, int16_t target) {
    if (walk_.motCt <= 0) *cur = target;
    else *cur = (int16_t)(*cur + (int16_t)((target - *cur) / walk_.motCt));
}
void Controller::rampTo(const ServoPositions& t) {
    for (int s = 0; s < 2; ++s) {
        movSv(&servos_.K0[s], t.K0[s]); movSv(&servos_.K1[s], t.K1[s]);
        movSv(&servos_.K2[s], t.K2[s]); movSv(&servos_.H[s],  t.H[s]);
        movSv(&servos_.A0[s], t.A0[s]); movSv(&servos_.A1[s], t.A1[s]);
        movSv(&servos_.U0[s], t.U0[s]); movSv(&servos_.U1[s], t.U1[s]);
        movSv(&servos_.U2[s], t.U2[s]); movSv(&servos_.E[s],  t.E[s]);
    }
    movSv(&servos_.head, t.head);
    movSv(&servos_.waist, t.waist);
}
void Controller::initWalkState() {
    WalkingState& w = walk_;
    w.dxi = w.dyi = w.dxis = w.dyis = w.dxib = w.dyib = 0;
    w.landF = w.landB = 0;
    w.fwctEnd = 18; w.walkCt = 0; w.fwctUp = 1; w.fwct = 1;
    w.autoH = HEIGHT; w.sw = w.swx = w.swy = 0;
    w.fh = 0; w.waist = 0;
    w.fallCt = 0; w.uprightCt = 0;
}

// ============================================================================
//  IMU sisi kontrol
// ============================================================================
bool Controller::imuFresh() const {
    return imuLastUs_ != 0 && (now() - imuLastUs_) < (int64_t)cfg_.imuStaleUs;
}
void Controller::imuPull(const ImuInput& in) {
    imuOk_ = in.sensorOk;
    imuHz_ = in.sensorOk ? in.rateHz : 0;
    if (!in.hasSample || !in.sensorOk || in.seq == imuSrcSeq_) return;
    imuSrcSeq_ = in.seq;
    float r = in.roll, p = in.pitch;
    if (cfg_.imuSwap) { const float t = r; r = p; p = t; }
    rawRoll_  = r * (float)cfg_.imuRollSign;
    rawPitch_ = p * (float)cfg_.imuPitchSign;
    roll_     = rawRoll_  - offRoll_;
    pitch_    = rawPitch_ - offPitch_;
    imuLastUs_ = in.tUs;
    imuSeq_++;
}
void Controller::imuCalReset() {
    cal_.n = 0; cal_.sumR = cal_.sumP = 0;
    cal_.minR = cal_.minP = 1.0e9f; cal_.maxR = cal_.maxP = -1.0e9f;
}
void Controller::imuCalStep() {
    if (!opt_.autoCalibrate || cal_.done) return;
    if (!imuFresh()) { imuCalReset(); return; }
    if (imuSeq_ == cal_.lastSeq) return;
    cal_.lastSeq = imuSeq_;
    cal_.sumR += rawRoll_; cal_.sumP += rawPitch_;
    if (rawRoll_  < cal_.minR) cal_.minR = rawRoll_;
    if (rawRoll_  > cal_.maxR) cal_.maxR = rawRoll_;
    if (rawPitch_ < cal_.minP) cal_.minP = rawPitch_;
    if (rawPitch_ > cal_.maxP) cal_.maxP = rawPitch_;
    if (++cal_.n < IMU_CAL_SAMPLES) return;
    if ((cal_.maxR - cal_.minR) <= IMU_CAL_MAX_SPREAD && (cal_.maxP - cal_.minP) <= IMU_CAL_MAX_SPREAD) {
        offRoll_ = cal_.sumR / (float)cal_.n; offPitch_ = cal_.sumP / (float)cal_.n;
        roll_ = rawRoll_ - offRoll_; pitch_ = rawPitch_ - offPitch_;
        cal_.done = true;
        logf("IMU kalibrasi OK: offset R=%.2f P=%.2f deg\r\n", (double)(offRoll_ * kRad2Deg), (double)(offPitch_ * kRad2Deg));
    } else {
        logf("IMU kalibrasi diulang: robot bergerak/bergetar\r\n");
        imuCalReset();
    }
}
void Controller::imuCalRestart() { cal_.done = false; offRoll_ = offPitch_ = 0; imuCalReset(); }

// ============================================================================
//  Keselamatan
// ============================================================================
static bool isWalkingMode(Mode m) { return m == Mode::BALANCE || modeIsGait(m); }
static bool fallCheckMode(Mode m) {
    return isWalkingMode(m) || m == Mode::READY || m == Mode::POSE_HOLD || m == Mode::MIRROR_DBG ||
           m == Mode::MOTION_PLAY || m == Mode::LIVE_POSE;
}

void Controller::enterFallen(const char* why) {
    if (walk_.mode == Mode::FALLEN) return;
    if (walk_.mode == Mode::MOTION_PLAY && motSlot_ >= 0) { motions_.release(motSlot_); motSlot_ = -1; mot_ = nullptr; }
    gaitOfsActive_ = false; intentDeadlineUs_ = 0;
    walk_.mode = Mode::FALLEN;
    walk_.fallCt = 0; walk_.uprightCt = 0; freeTick_ = 0;
    logf("\r\n!!! %s - SERVO FREE. Tegakkan robot lalu tekan 'r'.\r\n", why);
}
void Controller::startReset(const char* why, int16_t cycles) {
    if (walk_.mode == Mode::MOTION_PLAY && motSlot_ >= 0) { motions_.release(motSlot_); motSlot_ = -1; mot_ = nullptr; }
    gaitOfsActive_ = false; intentDeadlineUs_ = 0;
    walk_.motCt = cycles;
    walk_.mode  = Mode::RESET;
    logf("RESET: %s\r\n", why);
}
bool Controller::manualReset(const char* why) {
    if (walk_.mode == Mode::FALLEN) {
        if (!imuFresh() || fabsf(roll_) >= cfg_.rearmAngleRad || fabsf(pitch_) >= cfg_.rearmAngleRad) {
            logf("RESET ditolak: robot FALLEN belum tegak / IMU tidak fresh\r\n");
            return false;
        }
        const uint8_t n = syncStateFromServoFeedback();
        logf("Re-arm sync: %u/22 feedback servo dipakai sebagai baseline\r\n", (unsigned)n);
    }
    startReset(why);
    return true;
}
void Controller::detAng() {
    if (!imuFresh()) {
        if (isWalkingMode(walk_.mode)) startReset("data IMU basi");
        return;
    }
    const bool tilted = fabsf(pitch_) >= cfg_.fallAngleRad || fabsf(roll_) >= cfg_.fallAngleRad;
    if (!tilted) { walk_.fallCt = 0; return; }
    if (++walk_.fallCt >= FALL_CONFIRM_CYCLES) enterFallen("FALL DETECTED");
}
bool Controller::walkStateFinite() const {
    const WalkingState& w = walk_;
    const float v[] = { w.dxi, w.dyi, w.dxis, w.dyis, w.dxib, w.dyib, w.autoH, w.fh, w.fhMax, w.sw,
                        w.swx, w.swy, w.swMax, w.waist, w.fwct, w.fwctEnd, w.fwctUp };
    for (float x : v) if (!isFiniteVal(x)) return false;
    return true;
}

// ============================================================================
//  UVC & kinematika
// ============================================================================
void Controller::uvcSub() {
    WalkingState& w = walk_;
    if (w.fwct <= w.landF) {
        const float k = w.dyi / fmaxf(11.0f - w.fwct, 1.0f);
        w.dyi -= k; w.dyis += k;
        if (w.dxi > RST_F_LEG)       { w.dxi -= RST_F_LEG; w.dxis -= RST_F_LEG; }
        else if (w.dxi < -RST_F_LEG) { w.dxi += RST_F_LEG; w.dxis += RST_F_LEG; }
        else                         { w.dxis -= w.dxi; w.dxi = 0; }
    }
    w.dyis = clampf(w.dyis, -70.0f, 70.0f);
    w.dxis = clampf(w.dxis, -70.0f, 70.0f);
    if (HEIGHT > w.autoH) w.autoH += (HEIGHT - w.autoH) * 0.07f;
    else                  w.autoH = HEIGHT;
    const float supportRoll = (w.jikuasi == 0) ? -roll_ : roll_;
    if (w.fwct > w.fwctEnd - w.landB && supportRoll > 0)
        w.autoH -= (fabsf(w.dyi - w.dyib) + fabsf(w.dxi - w.dxib)) * 0.02f;
    w.autoH = clampf(w.autoH, 140.0f, HEIGHT);
}
void Controller::uvcSub2() {
    WalkingState& w = walk_;
    const float den = fmaxf(w.fwctEnd - w.fwct + 1.0f, 1.0f);
    const float k1 = w.dyi / den; w.dyi -= k1;
    const float k0 = w.dxi / den; w.dxi -= k0;
    w.waist -= w.waist / den;
    servos_.K2[0] = f2i16(w.waist,  servos_.K2[0]);
    servos_.K2[1] = f2i16(-w.waist, servos_.K2[1]);
    if (w.fwct <= w.landF) { w.dyis += k1; w.dxis -= k0; }
    else { w.dyis -= w.dyis / den; w.dxis -= w.dxis / den; }
    w.autoH += (HEIGHT - w.autoH) / den;
    w.dyis = clampf(w.dyis, -70.0f, 70.0f);
    w.dxis = clampf(w.dxis, -70.0f, 70.0f);
}
void Controller::uvc() {
    WalkingState& w = walk_;
    const float rollt  = (w.jikuasi == 0) ? -roll_ : roll_;
    const float pitcht = pitch_;
    if (w.fwct > w.landF && w.fwct <= w.fwctEnd - w.landB) {
        float k  = atan2f(w.dyi - w.sw, w.autoH);
        float kl = w.autoH / cosf(k);
        float ks = k + rollt;
        w.dyi   = kl * sinf(ks) + w.sw;
        w.autoH = kl * cosf(ks);
        k  = atan2f(w.dxi, w.autoH);
        kl = w.autoH / cosf(k);
        ks = k + pitcht;
        w.dxi   = kl * sinf(ks);
        w.autoH = kl * cosf(ks);
        w.dyi = clampf(w.dyi, 0.0f, 45.0f);
        w.dxi = clampf(w.dxi, -45.0f, 45.0f);
        w.dyis = w.dyi;
        w.dxis = -w.dxi;
    }
}
void Controller::footUp() {
    WalkingState& w = walk_;
    const float span = (float)(w.fwctEnd - w.landF - w.landB);
    if (w.fwct > w.landF && w.fwct <= (w.fwctEnd - w.landB) && span >= 1.0f) {
        const float t = (w.fwct - w.landF) / span;
        w.fh = w.fhMax * 0.5f * (1.0f - cosf(2.0f * kPi * t));
    } else w.fh = 0;
}
void Controller::swCont() {
    WalkingState& w = walk_;
    const float k = w.swMax * sinf(kPi * w.fwct / fmaxf(w.fwctEnd, 1.0f));
    float t;
    if (opt_.swcontLegacy) {
        t = atanf((fabsf(w.dxi) + fabsf(21.5f * sinf(w.fwct))) / (w.dyi + 21.5f * cosf(w.fwct) - w.fwct));
    } else {
        const float theta = w.waist / CHG_SVA;
        t = atan2f(fabsf(w.dxi) + fabsf(21.5f * sinf(theta)), w.dyi + 21.5f * cosf(theta));
    }
    w.swx = (w.dxi > 0) ? k * sinf(t) : -k * sinf(t);
    w.swy = k * cosf(t);
}
void Controller::armCont() {
    servos_.U1[0] = f2i16(clampf(510.0f * walk_.dyis / 70.0f, 0.0f, 1000.0f), servos_.U1[0]);
    servos_.U1[1] = servos_.U1[0];
}
void Controller::footCont(float x, float y, float h, int s) {
    const float temp1 = sqrtf(y * y + h * h) - LEG_OFFSET;
    float k = sqrtf(x * x + temp1 * temp1);
    if (k > OFS_L) k = OFS_L;
    k = fmaxf(k, 1.0f);
    const float x_angle = CHG_SVA * asinf(clampf(x / k, -1.0f, 1.0f));
    float knee = CHG_SVA * acosf(clampf(k / LEG_ACOS_DEN, -1.0f, 1.0f));
    knee = clampf(knee, 0.0f, 1800.0f);
    const float a = KNEE_LPF_ALPHA;
    knee = (servos_.H[s] / 2.0f) * (1.0f - a) + knee * a;
    servos_.H[s]  = f2i16(knee * 2.0f, servos_.H[s]);
    servos_.K0[s] = f2i16(knee + x_angle, servos_.K0[s]);
    servos_.A0[s] = f2i16(knee - x_angle, servos_.A0[s]);
    float k1 = CHG_SVA * atan2f(y, fmaxf(h, 1.0f));
    k1 = (float)servos_.K1[s] * (1.0f - a) + k1 * a;
    servos_.K1[s] = f2i16(k1,  servos_.K1[s]);
    servos_.A1[s] = f2i16(-k1, servos_.A1[s]);
}
static float maxReachHeight(float x, float y) {
    const float xx = clampf(x, -OFS_L, OFS_L);
    const float radial = sqrtf(fmaxf(OFS_L * OFS_L - xx * xx, 0.0f)) + LEG_OFFSET;
    return sqrtf(fmaxf(radial * radial - y * y, 0.0f));
}
void Controller::feetCont1(float x0, float y0, float x1, float y1, int s) {
    WalkingState& w = walk_;
    float wk = 0;
    if (s == 1) {
        float wt;
        if (w.jikuasi == 0) { wt = 0.5f * atan2f(x0,  y0 + 21.5f); wk = fabsf(15.0f * x0 / 45.0f); }
        else                { wt = 0.5f * atan2f(-x1, y1 + 21.5f); wk = fabsf(15.0f * x1 / 45.0f); }
        w.waist = clampf(wt * CHG_SVA, -HIP_YAW_MAX, HIP_YAW_MAX);
        servos_.K2[0] = f2i16(w.waist,  servos_.K2[0]);
        servos_.K2[1] = f2i16(-w.waist, servos_.K2[1]);
    }
    const float lim0 = maxReachHeight(x0, y0 - wk) + ((w.jikuasi == 1) ? w.fh : 0.0f);
    const float lim1 = maxReachHeight(x1, y1 - wk) + ((w.jikuasi == 0) ? w.fh : 0.0f);
    const float safeH = clampf(fminf(lim0, lim1), 140.0f, HEIGHT);
    if (w.autoH > safeH) w.autoH = safeH;
    if (w.jikuasi == 0) {
        footCont(x0, y0 - wk, w.autoH, 0);
        footCont(x1, y1 - wk, w.autoH - w.fh, 1);
    } else {
        footCont(x0, y0 - wk, w.autoH - w.fh, 0);
        footCont(x1, y1 - wk, w.autoH, 1);
    }
}
void Controller::feetCont2(int s) {
    const WalkingState& w = walk_;
    if (gaitOfsActive_) {                       // hanya WALK_CONT; mode lain identik V4
        if (w.jikuasi == 0)
            feetCont1(w.dxi - w.swx + gx_, w.dyi - w.swy, w.dxis - w.swx + gxs_, w.dyis + w.swy, s);
        else
            feetCont1(w.dxis - w.swx + gxs_, w.dyis + w.swy, w.dxi - w.swx + gx_, w.dyi - w.swy, s);
        // yaw: kaki tumpu +gyaw/2 -> -gyaw/2, kaki ayun sebaliknya (hitungan servo hip-yaw)
        const int sup = (w.jikuasi == 0) ? 0 : 1, swg = 1 - sup;
        servos_.K2[sup] = f2i16((float)servos_.K2[sup] + gyaw_ * CHG_SVA, servos_.K2[sup]);
        servos_.K2[swg] = f2i16((float)servos_.K2[swg] - gyaw_ * CHG_SVA, servos_.K2[swg]);
        return;
    }
    if (w.jikuasi == 0)
        feetCont1(w.dxi - w.swx, w.dyi - w.swy, w.dxis - w.swx, w.dyis + w.swy, s);
    else
        feetCont1(w.dxis - w.swx, w.dyis + w.swy, w.dxi - w.swx, w.dyi - w.swy, s);
}
void Controller::counterCont() {
    WalkingState& w = walk_;
    if (w.fwct >= w.fwctEnd) {
        w.jikuasi ^= 1;
        w.fwct = 0; w.fh = 0;
        float k = w.dyis; w.dyis = w.dyi; w.dyib = w.dyi; w.dyi = k;
        k = w.dxis; w.dxis = w.dxi; w.dxib = w.dxi; w.dxi = k;
    } else {
        w.fwct += w.fwctUp;
        if (w.fwct > w.fwctEnd) w.fwct = w.fwctEnd;
    }
}

// ============================================================================
//  State machine
// ============================================================================
void Controller::cmdGo() {
    if (walk_.mode != Mode::READY)          { logf("g: hanya dari mode 720\r\n"); return; }
    if (!imuFresh())                        { logf("g ditolak: data IMU tidak valid\r\n"); return; }
    if (opt_.autoCalibrate && !cal_.done)   { logf("g ditolak: kalibrasi IMU belum selesai\r\n"); return; }
    walk_.fwct = 1; walk_.mode = Mode::BALANCE;
    logf("  Go -> mode=730\r\n");
}

void Controller::walkControl() {
    WalkingState& w = walk_;
    switch (w.mode) {
        case Mode::RESET:
            rampTo(HOME_POSE);
            if (w.motCt > 0) w.motCt--;
            else {
                initWalkState();
                w.jikuasi = 1; footCont(0, 0, HEIGHT, 0);
                w.jikuasi = 0; footCont(0, 0, HEIGHT, 1);
                w.mode = Mode::READY;
                logf("mode=720 [Ready]\r\n");
            }
            break;
        case Mode::READY: imuCalStep(); break;
        case Mode::BALANCE:
            feetCont2(1);
            if (imuFresh() && (fabsf(roll_) > cfg_.triggerRollRad || fabsf(pitch_) > cfg_.triggerPitchRad)) {
                w.jikuasi = (roll_ > 0) ? 1 : 0;
                w.fwct = 1; w.mode = Mode::WALK_INIT;
                logf("mode=740 [Walk Init]\r\n");
            }
            break;
        case Mode::WALK_INIT:
            uvc(); uvcSub(); footUp(); feetCont2(1); armCont(); counterCont();
            if (w.fwct == 0) { w.mode = Mode::WALK_HOLD; logf("mode=750\r\n"); }
            break;
        case Mode::WALK_HOLD:
            feetCont2(1);
            if (w.fwct > 30) {
                w.fwct = 1;
                const float k = sqrtf(0.5f * w.dxis * w.dxis + w.dyis * w.dyis);
                w.swMax = cfg_.swayBase + cfg_.swayGain * k / 45.0f;
                w.mode = Mode::WALK_STEP;
                logf("mode=760 swMax=%.1f\r\n", (double)w.swMax);
            } else w.fwct++;
            break;
        case Mode::WALK_STEP:
            w.landF = 25; w.fwctEnd = (float)(w.landF + 25);
            uvcSub2(); footUp(); swCont(); feetCont2(0); armCont(); counterCont();
            if (w.fwct == 0) {
                w.landF = w.landB = 0;
                w.dxi = w.dyi = w.dxis = w.dyis = w.dxib = w.dyib = 0;
                w.fwctEnd = 18; w.walkCt = 0; w.fwctUp = 1; w.fwct = 1;
                w.autoH = HEIGHT; w.sw = w.swx = w.swy = 0; w.jikuasi = 1;
                w.mode = Mode::WALK_END; logf("mode=770\r\n");
            }
            break;
        case Mode::WALK_END:
            feetCont2(0);
            if (w.fwct > 50) { w.fwct = 1; w.mode = Mode::BALANCE; logf("mode=730\r\n"); }
            else w.fwct++;
            break;
        case Mode::POSE_HOLD: feetCont2(1); break;
        case Mode::ANGLE_DISP:
            if (++dispDiv_ >= 5) { dispDiv_ = 0;
                logf("P:%.2f R:%.2f deg\r\n", (double)(pitch_ * kRad2Deg), (double)(roll_ * kRad2Deg)); }
            break;
        case Mode::MIRROR_DBG:
            w.dxis = -w.dxi; w.dyis = w.dyi; feetCont2(1);
            break;
        case Mode::RAW_ANGLES:
            if (++dispDiv_ >= 5) { dispDiv_ = 0;
                logf("R:%d P:%d (x1000 rad)\r\n", (int)(roll_ * 1000.0f), (int)(pitch_ * 1000.0f)); }
            break;
        case Mode::WALK_CONT:   walkContStep(); break;
        case Mode::MOTION_PLAY: motionStep();   break;
        case Mode::LIVE_POSE:   livePoseStep(); break;
        case Mode::FALLEN:
            if (opt_.autoRearm) {
                if (imuFresh() && fabsf(roll_) < cfg_.rearmAngleRad && fabsf(pitch_) < cfg_.rearmAngleRad) {
                    if (++w.uprightCt >= REARM_STABLE_CYCLES) {
                        const uint8_t n = syncStateFromServoFeedback();
                        logf("Auto re-arm sync: %u/22 feedback servo\r\n", (unsigned)n);
                        startReset("robot tegak stabil, re-arm otomatis");
                    }
                } else w.uprightCt = 0;
            }
            break;
    }
}

// ============================================================================
//  Parameter, trim, input
// ============================================================================
void Controller::applyRuntimeParam(ParamId id, float v) {
    if (!isFiniteVal(v)) { logf("Parameter ditolak: NaN/Inf\r\n"); return; }
    const bool frameParam = id == ParamId::SIGN_ROLL || id == ParamId::SIGN_PITCH || id == ParamId::IMU_SWAP;
    if (frameParam && !(walk_.mode == Mode::READY || walk_.mode == Mode::FALLEN ||
                        walk_.mode == Mode::ANGLE_DISP || walk_.mode == Mode::RAW_ANGLES)) {
        logf("Parameter frame IMU ditolak: hanya di READY/FALLEN\r\n");
        return;
    }
    RuntimeConfig c = cfg_;
    switch (id) {
        case ParamId::FALL_DEG:          c.fallAngleRad = v / kRad2Deg; break;
        case ParamId::REARM_DEG:         c.rearmAngleRad = v / kRad2Deg; break;
        case ParamId::IMU_STALE_MS:      c.imuStaleUs = (uint32_t)(clampf(v, 20.0f, 500.0f) * 1000.0f); break;
        case ParamId::SERVO_STEP:        c.servoMaxStep = (int16_t)clampf(v, 20.0f, 1000.0f); break;
        case ParamId::TRIGGER_ROLL_DEG:  c.triggerRollRad = v / kRad2Deg; break;
        case ParamId::TRIGGER_PITCH_DEG: c.triggerPitchRad = v / kRad2Deg; break;
        case ParamId::FOOT_LIFT:         c.footLiftMax = v; break;
        case ParamId::SWAY_BASE:         c.swayBase = v; break;
        case ParamId::SWAY_GAIN:         c.swayGain = v; break;
        case ParamId::SIGN_ROLL:         c.imuRollSign = (v < 0) ? -1 : +1; break;
        case ParamId::SIGN_PITCH:        c.imuPitchSign = (v < 0) ? -1 : +1; break;
        case ParamId::IMU_SWAP:          c.imuSwap = v >= 0.5f; break;
        case ParamId::GAIT_ENABLE:       c.gaitEnable = v >= 0.5f; break;
        case ParamId::GAIT_STEP_MAX:     c.gaitStepMax = v; break;
        case ParamId::GAIT_YAW_MAX_DEG:  c.gaitYawMaxRad = v / kRad2Deg; break;
        case ParamId::OBSTACLE_STOP_MM:  c.obstacleStopMm = v; break;
        case ParamId::COUNT_:            return;
    }
    runtimeConfigSanitize(c);
    const bool frameChanged = c.imuRollSign != cfg_.imuRollSign || c.imuPitchSign != cfg_.imuPitchSign ||
                              c.imuSwap != cfg_.imuSwap;
    if (memcmp(&c, &cfg_, sizeof(c)) == 0) return;
    cfg_ = c;
    walk_.fhMax = cfg_.footLiftMax;
    if (frameChanged) { imuCalRestart(); cal_.done = !opt_.autoCalibrate; imuSrcSeq_ = 0; }
    ++cfgGen_;
}

bool Controller::setTrim(uint8_t bus, uint8_t slot, int32_t value) {
    if (bus > 1 || slot >= SERVO_SLOTS) return false;
    if (walk_.mode != Mode::READY) { logf("trim hanya di mode 720\r\n"); return false; }
    const int16_t v = (int16_t)clampi(value, -TRIM_LIMIT, TRIM_LIMIT);
    if (trim_[bus][slot] != v) { trim_[bus][slot] = v; ++cfgGen_; }
    return true;
}

static int trimSlot(char grp, uint8_t idx) {
    switch (grp) {
        case 'k': return idx == 0 ? 7 : idx == 1 ? 6 : idx == 2 ? 5 : -1;
        case 'a': return idx == 0 ? 9 : idx == 1 ? 10 : -1;
        case 'u': return idx == 0 ? 1 : idx == 1 ? 2 : idx == 2 ? 3 : -1;
        case 'h': return idx == 0 ? 8 : -1;
        case 'e': return idx == 0 ? 4 : -1;
        default:  return -1;
    }
}

bool Controller::isDebugSafe() const {
    const Mode m = walk_.mode;
    return m == Mode::READY || m == Mode::POSE_HOLD || m == Mode::ANGLE_DISP ||
           m == Mode::MIRROR_DBG || m == Mode::RAW_ANGLES;
}
void Controller::printHelp() {
    logf("r reset | g jalan (720->730) | x STOP servo free | c kalibrasi IMU | ? bantuan\r\n");
    logf("t sudut(deg) | y mirror-debug | z sudut x1000 | (t,y,z hanya dari mode 720)\r\n");
    logf("p<0-2>+/- parameter | k/u/a/h/e <r|l|b> <0-2> +/- trim permanen (mode 720) | spasi batal\r\n");
}

void Controller::handleKey(char c) {
    if (c == '\r' || c == '\n') return;
    logf("> %c\r\n", c);
    if (c == ' ') { keySt_ = KS_IDLE; return; }
    if (keySt_ == KS_IDLE) {
        switch (c) {
            case 'r': manualReset("serial r"); break;
            case 'g': cmdGo(); break;
            case 'x': enterFallen("STOP (x)"); break;
            case 'c':
                if (walk_.mode == Mode::READY) { imuCalRestart(); logf("Kalibrasi IMU dimulai (jaga robot diam)\r\n"); }
                else logf("c: hanya dari mode 720\r\n");
                break;
            case 't': if (isDebugSafe()) { walk_.mode = Mode::ANGLE_DISP; logf("  Angle Disp\r\n"); } else logf("t: hanya dari mode 720\r\n"); break;
            case 'y': if (isDebugSafe()) { walk_.mode = Mode::MIRROR_DBG; logf("  Debug\r\n"); }   else logf("y: hanya dari mode 720\r\n"); break;
            case 'z': if (isDebugSafe()) { walk_.mode = Mode::RAW_ANGLES; logf("  Raw angle\r\n"); } else logf("z: hanya dari mode 720\r\n"); break;
            case 'p': keySt_ = KS_PARAM; break;
            case 'k': case 'u': case 'a': case 'h': case 'e':
                trimGrp_ = c; trimSide_ = 0; trimIdx_ = 0; keySt_ = KS_TRIM;
                logf("trim %c: r/l/b, angka=sendi, +/- (30 hitungan), spasi=keluar\r\n", trimGrp_);
                break;
            case '?': printHelp(); break;
            default: break;
        }
    } else if (keySt_ == KS_PARAM) {
        if (c >= '0' && c <= '9') paramSel_ = (int8_t)(c - '0');
        else if (c == '+' || c == '-') {
            const float d = (c == '+') ? 1.0f : -1.0f;
            switch (paramSel_) {
                case 0: walk_.dxi = clampf(walk_.dxi + d, -45.0f, 45.0f); break;
                case 1: walk_.dyi = clampf(walk_.dyi + d, -45.0f, 45.0f); break;
                case 2: applyRuntimeParam(ParamId::SWAY_BASE, cfg_.swayBase + d); break;
                default: break;
            }
            logf("No:%d dx:%.0f dy:%.0f swayBase:%.1f\r\n", paramSel_, (double)walk_.dxi, (double)walk_.dyi,
                 (double)cfg_.swayBase);
        }
    } else {
        if (c == 'r') trimSide_ = 0; else if (c == 'l') trimSide_ = 1; else if (c == 'b') trimSide_ = 2;
        else if (c == 'k' || c == 'u' || c == 'a' || c == 'h' || c == 'e') { trimGrp_ = c; trimIdx_ = 0; }
        else if (c >= '0' && c <= '9') trimIdx_ = (uint8_t)(c - '0');
        else if (c == '+' || c == '-') {
            const int sl = trimSlot(trimGrp_, trimIdx_);
            if (sl < 0) { logf("sendi %c%d tidak ada\r\n", trimGrp_, trimIdx_); return; }
            const int16_t d = (c == '+') ? 30 : -30;
            for (uint8_t sd = 0; sd < 2; ++sd) {
                if (trimSide_ != 2 && trimSide_ != (int8_t)sd) continue;
                setTrim(sd, (uint8_t)sl, (int32_t)trim_[sd][sl] + d);
            }
            logf("trim %c%d R=%d L=%d (disimpan otomatis)\r\n", trimGrp_, trimIdx_, trim_[0][sl], trim_[1][sl]);
        }
    }
}

void Controller::handleCommand(const Command& c) {
    switch (c.type) {
        case CmdType::RESET: manualReset("remote command"); break;
        case CmdType::GO:    cmdGo(); break;
        case CmdType::STOP:  enterFallen("STOP (remote)"); break;
        case CmdType::CALIBRATE:
            if (walk_.mode == Mode::READY) { imuCalRestart(); logf("Kalibrasi IMU dimulai (remote)\r\n"); }
            else logf("Calibrate ditolak: hanya READY\r\n");
            break;
        case CmdType::SET_PARAM: applyRuntimeParam(c.param, c.value); break;
        case CmdType::SET_TRIM:  setTrim(c.bus, c.slot, (int32_t)lroundf(clampf(c.value, -30000.0f, 30000.0f))); break;
        case CmdType::GAIT_INTENT: gaitIntent(c); break;
        case CmdType::MOTION_PLAY: motionStart(c.slot); break;
        case CmdType::POSE_TARGET: poseTarget(c); break;
        case CmdType::PERCEPTION:
            obstacleFrontMm_ = (uint16_t)clampi((int32_t)c.value, 0, 0xFFFF);
            obstacleDeadlineUs_ = now() + (int64_t)clampi(c.ttlMs, 0, 1000) * 1000;
            break;
        case CmdType::MOTION_STOP:
            if (walk_.mode == Mode::MOTION_PLAY) motionEnd(false, "dihentikan");
            break;
    }
}

// ============================================================================
//  Siklus
// ============================================================================
void Controller::housekeeping() {
    const uint32_t ms = (uint32_t)(now() / 1000);
    if (ms - lastTeleLogMs_ >= TELE_LOG_MS) {
        lastTeleLogMs_ = ms;
        const int age = imuLastUs_ ? (int)((now() - imuLastUs_) / 1000) : -1;
        logf("TELE m=%u R=%.1f P=%.1f fwct=%.0f jiku=%d | loop avg=%uus max=%uus ics=%uus ovr=%u | imu=%dms %uHz cal=%d\r\n",
             (unsigned)walk_.mode, (double)(roll_ * kRad2Deg), (double)(pitch_ * kRad2Deg),
             (double)walk_.fwct, (int)walk_.jikuasi, (unsigned)loopAvgUs_, (unsigned)loopMaxUs_,
             (unsigned)servoPhaseUs_, (unsigned)loopOverruns_, age, (unsigned)imuHz_, (int)cal_.done);
        loopMaxUs_ = 0;
    }
}

void Controller::step(const ImuInput& imuIn, ServoBus& bus, Inbox& inbox) {
    const int64_t t0 = now();
    imuPull(imuIn);
    char k;
    for (uint8_t n = 0; n < 16 && inbox.nextKey(k); ++n) handleKey(k);
    Command c;
    for (uint8_t n = 0; n < 12 && inbox.nextCommand(c); ++n) handleCommand(c);
    if (fallCheckMode(walk_.mode)) detAng();
    walkControl();
    if (!walkStateFinite()) { initWalkState(); startReset("nilai NaN/Inf pada state jalan"); }

    if (walk_.mode == Mode::FALLEN) {
        if ((freeTick_++ % 10) == 0) sendAllServoFree(bus);    // ~10 Hz + feedback posisi
    } else {
        freeTick_ = 0;
        sendAllServoPositions(bus);
    }
    housekeeping();
    const uint32_t dt = (uint32_t)(now() - t0);
    loopAvgUs_ = (loopAvgUs_ * 15u + dt) / 16u;
    if (dt > loopMaxUs_) loopMaxUs_ = dt;
    if (bb_) blackboxPush(dt);
}

void Controller::snapshot(Telemetry& t) const {
    const int64_t nowUs = now();
    t.seq = ++teleSeq_; t.ms = (uint32_t)(nowUs / 1000); t.mode = walk_.mode;
    t.roll = roll_; t.pitch = pitch_; t.rawRoll = rawRoll_; t.rawPitch = rawPitch_;
    t.offRoll = offRoll_; t.offPitch = offPitch_;
    t.imuAgeMs = imuLastUs_ ? (int32_t)((nowUs - imuLastUs_) / 1000) : -1;
    t.imuHz = imuHz_;
    t.imuFreshFlag = imuFresh(); t.imuInit = imuOk_; t.calDone = cal_.done;
    const WalkingState& w = walk_;
    t.fwct = w.fwct; t.fwctEnd = w.fwctEnd; t.dxi = w.dxi; t.dyi = w.dyi;
    t.dxis = w.dxis; t.dyis = w.dyis; t.autoH = w.autoH; t.fh = w.fh;
    t.swx = w.swx; t.swy = w.swy; t.swMax = w.swMax; t.waist = w.waist;
    t.jikuasi = w.jikuasi; t.kin = servos_; t.cfg = cfg_; t.cfgGen = cfgGen_;
    t.loopAvgUs = loopAvgUs_; t.loopMaxUs = loopMaxUs_; t.loopOverruns = loopOverruns_;
    t.servoPhaseUs = servoPhaseUs_;
    t.ctrlStackFree = 0;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const ServoFeedback& fb = fb_[b][s];
        t.cmd[b][s] = lastOut_[b][s]; t.fb[b][s] = fb.pos; t.trim[b][s] = trim_[b][s];
        t.fbValid[b][s] = fb.valid ? 1 : 0; t.fbMiss[b][s] = fb.missCount;
        const int64_t age = fb.lastUs ? (nowUs - fb.lastUs) / 1000 : 65535;
        t.fbAgeMs[b][s] = (uint16_t)(age > 65535 ? 65535 : age);
    }
    t.intentActive = intentDeadlineUs_ > nowUs;
    t.obstacleBlock = obstacleBlocked();
    t.obstacleFrontMm = obstacleDeadlineUs_ > nowUs ? obstacleFrontMm_ : 0xFFFF;
    t.intentVx = gaitStride_; t.intentYawDeg = gaitYaw_ * kRad2Deg;
    t.motionSlot = motSlot_; t.motionFrame = motFrame_;
    t.blackboxDropped = bb_ ? bb_->dropped() : 0;
}

// ============================================================================
//  Fase 1: gait kontinu (EKSPERIMENTAL - validasi di robot tergantung + twin dulu)
//  Langkah UVC berulang (mekanik WALK_INIT) + offset langkah depan & yaw:
//    kaki tumpu x: +S/2 -> -S/2, kaki ayun x: -S/2 -> +S/2 selama satu langkah.
//  S dan yaw hanya berubah di batas langkah, maks 3 mm / 1 deg per langkah.
// ============================================================================
void Controller::gaitIntent(const Command& c) {
    const uint32_t ms = (uint32_t)(now() / 1000);
    if (!cfg_.gaitEnable) {
        if (ms - lastGaitRejectMs_ > 2000) { lastGaitRejectMs_ = ms; logf("GAIT_INTENT ditolak: gaitEnable=0\r\n"); }
        return;
    }
    if (walk_.mode != Mode::BALANCE && walk_.mode != Mode::WALK_CONT) {
        if (ms - lastGaitRejectMs_ > 2000) { lastGaitRejectMs_ = ms; logf("GAIT_INTENT ditolak: hanya dari BALANCE/WALK_CONT\r\n"); }
        return;
    }
    if (!isFiniteVal(c.value) || !isFiniteVal(c.yawDeg)) return;
    gaitTgtStride_ = clampf(c.value, -cfg_.gaitStepMax, cfg_.gaitStepMax);
    gaitTgtYaw_    = clampf(c.yawDeg / kRad2Deg, -cfg_.gaitYawMaxRad, cfg_.gaitYawMaxRad);
    intentDeadlineUs_ = now() + (int64_t)clampi(c.ttlMs, 0, 1000) * 1000;
    if (walk_.mode == Mode::BALANCE && c.ttlMs > 0) {
        initWalkState();
        walk_.jikuasi = (roll_ > 0) ? 1 : 0;
        walk_.fwct = 1;
        gaitStride_ = gaitYaw_ = 0; gx_ = gxs_ = gyaw_ = 0;
        gaitOfsActive_ = true;
        walk_.mode = Mode::WALK_CONT;
        logf("mode=800 [Walk Cont] vx=%.1f yaw=%.1f\r\n", (double)gaitTgtStride_, (double)(gaitTgtYaw_ * kRad2Deg));
    }
}

void Controller::gaitOffsets() {
    const float t = clampf(walk_.fwct / fmaxf(walk_.fwctEnd, 1.0f), 0.0f, 1.0f);
    gx_   = gaitStride_ * (0.5f - t);
    gxs_  = gaitStride_ * (t - 0.5f);
    gyaw_ = gaitYaw_ * (0.5f - t);
}

void Controller::walkContStep() {
    WalkingState& w = walk_;
    const bool expired = now() >= intentDeadlineUs_;
    if (expired) { gaitTgtStride_ = 0; gaitTgtYaw_ = 0; }
    uvc(); uvcSub(); footUp();
    gaitOffsets();
    feetCont2(1); armCont(); counterCont();
    if (w.fwct == 0) {                                   // batas langkah
        gaitStride_ += clampf(gaitTgtStride_ - gaitStride_, -3.0f, 3.0f);
        // Fase 3: refleks rintangan - robot sendiri yang menolak melangkah maju (HP hanya melapor)
        const bool block = obstacleBlocked();
        if (block && gaitStride_ > 0.0f) {
            gaitStride_ = 0.0f;
            if (!obstacleBlockLogged_) logf("rintangan %u mm di depan: langkah maju diblokir\r\n", (unsigned)obstacleFrontMm_);
        }
        obstacleBlockLogged_ = block;
        gaitYaw_    += clampf(gaitTgtYaw_ - gaitYaw_, -1.0f / kRad2Deg, 1.0f / kRad2Deg);
        if (expired && fabsf(gaitStride_) < 0.01f && fabsf(gaitYaw_) < 1e-4f) {
            gaitOfsActive_ = false; gx_ = gxs_ = gyaw_ = 0;
            w.landF = w.landB = 0;
            w.dxi = w.dyi = w.dxis = w.dyis = w.dxib = w.dyib = 0;
            w.fwctEnd = 18; w.walkCt = 0; w.fwctUp = 1; w.fwct = 1;
            w.autoH = HEIGHT; w.sw = w.swx = w.swy = 0; w.jikuasi = 1;
            w.mode = Mode::WALK_END;
            logf("mode=770 (niat berakhir)\r\n");
        }
    }
}

// ============================================================================
//  Fase 1: motion player
// ============================================================================
void Controller::motionStart(int slot) {
    if (walk_.mode != Mode::READY) { logf("MOTION_PLAY ditolak: hanya dari READY\r\n"); return; }
    const Motion* m = motions_.acquire(slot);
    if (!m) { logf("MOTION_PLAY ditolak: slot %d kosong/sibuk\r\n", slot); return; }
    for (int i = 0; i < m->nFrames; ++i)
        for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
            const JointDef& j = JOINTS[b][s];
            const int32_t raw = (int32_t)j.center + (int32_t)j.sign * ((int32_t)m->f[i].joint[b][s] + trim_[b][s]);
            if (raw < j.lo || raw > j.hi) {
                logf("MOTION_PLAY ditolak: frame %d %s di luar batas (%ld)\r\n", i, JOINT_NAMES[b][s], (long)raw);
                motions_.release(slot);
                return;
            }
        }
    mot_ = m; motSlot_ = (int8_t)slot; motFrame_ = 0; motFrameStartUs_ = now();
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) motPrev_[b][s] = *jointSrc(b, s);
    walk_.mode = Mode::MOTION_PLAY;
    logf("mode=810 [Motion] slot %d '%s' %u frame\r\n", slot, m->name, (unsigned)m->nFrames);
}

void Controller::motionStep() {
    if (!mot_) { walk_.mode = Mode::READY; return; }
    const MotionFrame& f = mot_->f[motFrame_];
    const float t = clampf((float)(now() - motFrameStartUs_) / (1000.0f * (float)f.durMs), 0.0f, 1.0f);
    float a = t;
    if (f.interp == MI_COSINE) a = 0.5f * (1.0f - cosf(kPi * t));
    else if (f.interp == MI_STEP) a = 1.0f;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const float v = (float)motPrev_[b][s] + ((float)f.joint[b][s] - (float)motPrev_[b][s]) * a;
        *jointSrc(b, s) = f2i16(v, *jointSrc(b, s));
    }
    if (t >= 1.0f) {
        memcpy(motPrev_, f.joint, sizeof(motPrev_));
        motFrameStartUs_ = now();
        if (++motFrame_ >= mot_->nFrames) motionEnd((mot_->flags & MF_RETURN_HOME) != 0, "selesai");
    }
}

void Controller::motionEnd(bool returnHome, const char* why) {
    if (motSlot_ >= 0) motions_.release(motSlot_);
    logf("Motion %s (slot %d)\r\n", why, motSlot_);
    mot_ = nullptr; motSlot_ = -1; motFrame_ = 0;
    walk_.mode = Mode::READY;
    if (returnHome) startReset("motion selesai -> HOME");
}

// ============================================================================
//  Fase 2: pose langsung (studio, kamera). Hanya dari READY; robot tetap otoritas:
//  batas sendi dicek per pesan, kecepatan dibatasi slew output (servoMaxStep),
//  deteksi jatuh aktif, ttl habis -> ramp kembali ke HOME.
// ============================================================================
void Controller::poseTarget(const Command& c) {
    const Mode m = walk_.mode;
    if (m != Mode::READY && m != Mode::LIVE_POSE) {
        if (now() / 1000 - lastPoseRejectMs_ > 2000) { lastPoseRejectMs_ = now() / 1000; logf("POSE_TARGET ditolak: hanya dari READY\r\n"); }
        return;
    }
    if (c.ttlMs == 0) {                                   // host minta selesai
        if (m == Mode::LIVE_POSE) { walk_.mode = Mode::READY; startReset("live pose selesai -> HOME"); }
        return;
    }
    const uint32_t mask = c.poseMask & ((1u << (2 * SERVO_SLOTS)) - 1u);
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        if (!(mask & (1u << (b * SERVO_SLOTS + s)))) continue;
        const JointDef& j = JOINTS[b][s];
        const int32_t raw = (int32_t)j.center + (int32_t)j.sign * ((int32_t)c.pose[b][s] + trim_[b][s]);
        if (raw < j.lo || raw > j.hi) {
            logf("POSE_TARGET ditolak: %s di luar batas (%ld)\r\n", JOINT_NAMES[b][s], (long)raw);
            return;
        }
    }
    if (m == Mode::READY) {
        for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) poseTgt_[b][s] = *jointSrc(b, s);
        walk_.mode = Mode::LIVE_POSE;
        logf("mode=820 [Live pose] mask=0x%06lx\r\n", (unsigned long)mask);
    }
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        if (mask & (1u << (b * SERVO_SLOTS + s))) poseTgt_[b][s] = c.pose[b][s];
    poseDeadlineUs_ = now() + (int64_t)(c.ttlMs > 1000 ? 1000 : c.ttlMs) * 1000;
}

void Controller::livePoseStep() {
    if (now() >= poseDeadlineUs_) {
        walk_.mode = Mode::READY;
        startReset("live pose: ttl habis -> HOME");
        return;
    }
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) *jointSrc(b, s) = poseTgt_[b][s];
}

bool Controller::obstacleBlocked() const {
    return cfg_.obstacleStopMm > 0.0f && now() < obstacleDeadlineUs_ && (float)obstacleFrontMm_ < cfg_.obstacleStopMm;
}

void Controller::blackboxPush(uint32_t loopUs) {
    BlackboxRecord r;
    r.ms = (uint32_t)(now() / 1000);
    r.mode = (uint16_t)walk_.mode;
    r.rollCdeg  = (int16_t)clampf(roll_ * kRad2Deg * 100.0f, -32000.0f, 32000.0f);
    r.pitchCdeg = (int16_t)clampf(pitch_ * kRad2Deg * 100.0f, -32000.0f, 32000.0f);
    r.loopUs = (uint16_t)clampi((int32_t)loopUs, 0, 65535);
    memcpy(r.cmd, lastOut_, sizeof(r.cmd));
    r.fbValidBits = 0;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        if (fb_[b][s].valid) r.fbValidBits |= 1u << (b * SERVO_SLOTS + s);
    r.flags = (uint8_t)((imuFresh() ? 1 : 0) | (walk_.jikuasi ? 2 : 0) | (intentDeadlineUs_ > now() ? 4 : 0));
    r.fwct = (uint8_t)clampf(walk_.fwct, 0.0f, 255.0f);
    r.autoHx10 = (uint16_t)clampf(walk_.autoH * 10.0f, 0.0f, 65535.0f);
    bb_->push(r);
}

}  // namespace la
