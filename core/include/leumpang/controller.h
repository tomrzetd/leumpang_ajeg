// =============================================================================
//  leumpang-core Controller: UVC + IK + state machine + keselamatan
//  Tidak bergantung RTOS. Satu thread pemanggil (task kontrol / loop simulasi).
//
//  Pemakaian per siklus 10 ms (HAL):
//      ctrl.step(imuInput, bus, inbox);
//      if (perlu publish) ctrl.snapshot(t);   // salin di bawah lock milik HAL
// =============================================================================
#pragma once
#include "leumpang/types.h"
#include "leumpang/ics.h"
#include "leumpang/motion.h"
#include "leumpang/blackbox.h"

namespace la {

// Sumber input non-realtime (konsol, protokol LAP, web). Dikuras di awal siklus.
class Inbox {
public:
    virtual ~Inbox() = default;
    virtual bool nextKey(char& c) = 0;
    virtual bool nextCommand(Command& c) = 0;
};

struct Platform {
    int64_t (*nowUs)(void* ctx);                        // jam monotonic mikrodetik
    void    (*log)(void* ctx, const char* line);        // baris log (boleh nullptr)
    void*   ctx;
};

struct WalkingState {
    Mode    mode;
    int16_t motCt, walkCt;
    float   fwct, fwctEnd, fwctUp;
    float   dxi, dyi, dxis, dyis, dxib, dyib;
    float   autoH, fh, fhMax;
    float   sw, swx, swy, swMax;
    float   waist;
    int16_t landF, landB;
    int8_t  jikuasi;
    uint8_t  fallCt;
    uint16_t uprightCt;
};

struct JointDef { uint8_t id; int16_t center; int8_t sign; int16_t lo, hi; };
extern const JointDef JOINTS[SERVO_BUSES][SERVO_SLOTS];   // profil KHR-3HV
extern const ServoPositions HOME_POSE;
extern const char* const JOINT_NAMES[SERVO_BUSES][SERVO_SLOTS];

class Controller {
public:
    struct Options {
        bool autoCalibrate   = true;
        bool autoRearm       = false;
        bool swcontLegacy    = false;
    };

    void init(const Platform& p, const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS],
              const Options& opt);

    // Boot: panggil bootFreeCycle() BOOT_SYNC_CYCLES kali dengan jeda 10 ms, lalu bootFinish().
    void bootFreeCycle(ServoBus& bus);
    void bootFinish(bool syncFromFeedback);

    // Satu siklus kontrol.
    void step(const ImuInput& imu, ServoBus& bus, Inbox& inbox);
    void noteOverrun() { ++loopOverruns_; }

    void snapshot(Telemetry& t) const;
    Mode mode() const { return walk_.mode; }
    uint32_t cfgGen() const { return cfgGen_; }

    // Fase 1
    MotionStore& motions() { return motions_; }          // dibagi dengan thread komunikasi
    void setBlackbox(BlackboxRing* r) { bb_ = r; }        // nullptr = mati

private:
    // ---- servo ----
    struct ServoFeedback { int16_t pos; int64_t lastUs; uint32_t okCount, missCount; bool valid; };
    int16_t* jointSrc(uint8_t b, uint8_t s);
    int16_t  jointTarget(uint8_t b, uint8_t s);
    void     primeOutputs();
    void     sendSlot(ServoBus& bus, uint8_t slot, bool freeTorque);
    void     sendAllServoPositions(ServoBus& bus);
    void     sendAllServoFree(ServoBus& bus);
    uint8_t  syncStateFromServoFeedback();
    // ---- gerak ----
    void movSv(int16_t* cur, int16_t target);
    void rampTo(const ServoPositions& t);
    void initWalkState();
    // ---- IMU ----
    bool imuFresh() const;
    void imuPull(const ImuInput& in);
    void imuCalReset();
    void imuCalStep();
    void imuCalRestart();
    // ---- keselamatan ----
    void enterFallen(const char* why);
    void startReset(const char* why, int16_t cycles = 80);
    bool manualReset(const char* why);
    void detAng();
    bool walkStateFinite() const;
    // ---- UVC/IK ----
    void uvcSub(); void uvcSub2(); void uvc(); void footUp(); void swCont(); void armCont();
    void footCont(float x, float y, float h, int s);
    void feetCont1(float x0, float y0, float x1, float y1, int s);
    void feetCont2(int s);
    void counterCont();
    // ---- state machine & input ----
    void cmdGo();
    void walkControl();
    void applyRuntimeParam(ParamId id, float v);
    bool setTrim(uint8_t bus, uint8_t slot, int32_t value);
    bool isDebugSafe() const;
    void printHelp();
    void handleKey(char c);
    void handleCommand(const Command& c);
    void housekeeping();
    // ---- Fase 1: gait kontinu & motion ----
    void gaitIntent(const Command& c);
    void walkContStep();
    void gaitOffsets();
    void motionStart(int slot);
    void motionStep();
    void motionEnd(bool returnHome, const char* why);
    void blackboxPush(uint32_t loopUs);
    // ---- Fase 2: pose langsung ----
    void poseTarget(const Command& c);
    void livePoseStep();
    // ---- Fase 3: refleks rintangan ----
    bool obstacleBlocked() const;
    // ---- util ----
    int64_t now() const { return plat_.nowUs(plat_.ctx); }
    void logf(const char* fmt, ...) LA_PRINTF_FMT(2, 3);

    Platform       plat_{};
    Options        opt_{};
    ServoPositions servos_{};
    WalkingState   walk_{};
    RuntimeConfig  cfg_{};
    int16_t        trim_[2][SERVO_SLOTS]{};
    uint32_t       cfgGen_ = 0;

    // IMU terkoreksi
    float    roll_ = 0, pitch_ = 0, rawRoll_ = 0, rawPitch_ = 0, offRoll_ = 0, offPitch_ = 0;
    bool     imuOk_ = false;
    int64_t  imuLastUs_ = 0;
    uint32_t imuSeq_ = 0, imuSrcSeq_ = 0;
    uint16_t imuHz_ = 0;
    struct { bool done; uint16_t n; float sumR, sumP, minR, maxR, minP, maxP; uint32_t lastSeq; } cal_{};

    int16_t       lastOut_[2][SERVO_SLOTS]{};
    ServoFeedback fb_[2][SERVO_SLOTS]{};
    uint32_t      freeTick_ = 0;
    uint32_t      servoPhaseUs_ = 0;
    uint32_t      loopAvgUs_ = 0, loopMaxUs_ = 0, loopOverruns_ = 0;
    uint32_t      lastTeleLogMs_ = 0;
    mutable uint32_t teleSeq_ = 0;
    uint8_t       dispDiv_ = 0;

    // gait kontinu (EKSPERIMENTAL)
    bool     gaitOfsActive_ = false;
    float    gaitStride_ = 0, gaitYaw_ = 0;            // nilai langkah saat ini (mm, rad)
    float    gaitTgtStride_ = 0, gaitTgtYaw_ = 0;      // target dari niat
    float    gx_ = 0, gxs_ = 0, gyaw_ = 0;             // offset tumpu/ayun (mm), yaw (rad)
    int64_t  intentDeadlineUs_ = 0;
    int64_t  poseDeadlineUs_ = 0;
    int64_t  obstacleDeadlineUs_ = 0;
    uint16_t obstacleFrontMm_ = 0xFFFF;
    bool     obstacleBlockLogged_ = false;
    int64_t  lastPoseRejectMs_ = 0;
    int16_t  poseTgt_[2][SERVO_SLOTS]{};
    uint32_t lastGaitRejectMs_ = 0;

    // motion player
    MotionStore   motions_;
    const Motion* mot_ = nullptr;
    int8_t        motSlot_ = -1;
    uint8_t       motFrame_ = 0;
    int64_t       motFrameStartUs_ = 0;
    int16_t       motPrev_[2][SERVO_SLOTS]{};

    BlackboxRing* bb_ = nullptr;

    // parser konsol
    enum KeyState : uint8_t { KS_IDLE, KS_PARAM, KS_TRIM };
    KeyState keySt_ = KS_IDLE;
    int8_t   paramSel_ = 0;
    char     trimGrp_ = 0;
    int8_t   trimSide_ = 0;
    uint8_t  trimIdx_ = 0;
};

}  // namespace la
