// =============================================================================
//  leumpang-core: tipe & konstanta robot (independen board/RTOS)
//  Dipakai firmware (ESP32-S3/P4, STM32H562), simulator, desktop, tes.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

// printf-format check (GCC/Clang); kosong di MSVC (studio Windows)
#if defined(__GNUC__) || defined(__clang__)
#define LA_PRINTF_FMT(a, b) __attribute__((format(printf, a, b)))
#else
#define LA_PRINTF_FMT(a, b)
#endif

namespace la {

// ---- Versi ----------------------------------------------------------------
constexpr uint8_t CORE_VER_MAJOR = 0, CORE_VER_MINOR = 7, CORE_VER_PATCH = 0;

// ---- Servo ICS 3.5 ----------------------------------------------------------
constexpr uint32_t ICS_BAUD     = 1250000;   // 8E1
constexpr int16_t  ICS_POS_MIN  = 3500;
constexpr int16_t  ICS_POS_MAX  = 11500;
constexpr uint16_t ICS_POS_FREE = 0;
constexpr int16_t  ICS_FB_MIN   = 1;
constexpr int16_t  ICS_FB_MAX   = 16383;
constexpr uint8_t  SERVO_SLOTS  = 11;        // per bus; bus 0 = kanan (R), 1 = kiri (L)
constexpr uint8_t  SERVO_BUSES  = 2;
constexpr int16_t  SERVO_MAX_STEP = 300;
constexpr int16_t  TRIM_LIMIT   = 1000;
constexpr int64_t  SERVO_FB_VALID_US = 500000;
constexpr int      BOOT_SYNC_CYCLES  = 12;

// ---- Waktu ----------------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_MS = 10;   // 100 Hz (gait di-tuning per siklus)
constexpr uint32_t TELE_LOG_MS       = 1000;

// ---- IMU (sisi kontrol) ----------------------------------------------------
constexpr int64_t  IMU_STALE_US       = 50000;
constexpr uint16_t IMU_CAL_SAMPLES    = 100;
constexpr float    IMU_CAL_MAX_SPREAD = 0.02f;

// ---- Keselamatan ------------------------------------------------------------
constexpr float    FALL_ANGLE_RAD      = 0.35f;
constexpr uint8_t  FALL_CONFIRM_CYCLES = 5;
constexpr float    REARM_ANGLE_RAD     = 0.17f;
constexpr uint16_t REARM_STABLE_CYCLES = 200;

// ---- Mekanik (KHR-3HV) ------------------------------------------------------
constexpr float OFS_L          = 129.0f;
constexpr float LEG_OFFSET     = 64.5f;
constexpr float LEG_ACOS_DEN   = 130.0f;
constexpr float CHG_SVA        = 1718.9f;     // hitungan servo per radian
constexpr float HEIGHT         = 185.0f;
constexpr float RST_F_LEG      = 3.0f;
constexpr float KNEE_LPF_ALPHA = 0.35f;
constexpr float HIP_YAW_MAX    = 1400.0f;

constexpr float kPi      = 3.14159265f;
constexpr float kRad2Deg = 57.2957795f;

// ---- Mode (nilai stabil: dipakai telemetry & protokol) --------------------
enum class Mode : uint16_t {
    RAW_ANGLES = 700, FALLEN = 705, RESET = 710, READY = 720, BALANCE = 730,
    WALK_INIT = 740, WALK_HOLD = 750, WALK_STEP = 760, WALK_END = 770,
    POSE_HOLD = 780, ANGLE_DISP = 790, MIRROR_DBG = 791,
    WALK_CONT = 800,      // Fase 1: langkah berulang atas GAIT_INTENT (EKSPERIMENTAL)
    MOTION_PLAY = 810,    // Fase 1: memutar template gerak
    LIVE_POSE = 820       // Fase 2: pose langsung dari studio/kamera (POSE_TARGET + ttl)
};
const char* modeName(Mode m);
const char* modeShortName(Mode m);
bool modeIsGait(Mode m);              // WALK_INIT..WALK_END, WALK_CONT
bool modePersistSafe(Mode m);         // aman menulis flash/SD besar (robot diam / servo free)

// ---- Runtime config (persisten) ------------------------------------------
struct RuntimeConfig {
    float    fallAngleRad;
    float    rearmAngleRad;
    uint32_t imuStaleUs;
    int16_t  servoMaxStep;
    float    triggerRollRad;
    float    triggerPitchRad;
    float    footLiftMax;
    float    swayBase;
    float    swayGain;
    int8_t   imuRollSign;
    int8_t   imuPitchSign;
    bool     imuSwap;
    // ---- Fase 1: gait kontinu ----
    bool     gaitEnable;        // default false: GAIT_INTENT ditolak
    float    gaitStepMax;       // mm per langkah (|vx|)
    float    gaitYawMaxRad;     // rad per langkah
};
RuntimeConfig runtimeConfigDefault();
void runtimeConfigSanitize(RuntimeConfig& c);

struct ServoPositions {
    int16_t K0[2], K1[2], K2[2], H[2], A0[2], A1[2];
    int16_t U0[2], U1[2], U2[2], E[2];
    int16_t head, waist;
};

// ---- Perintah -------------------------------------------------------------
enum class CmdType : uint8_t { RESET, GO, STOP, CALIBRATE, SET_PARAM, SET_TRIM,
                               GAIT_INTENT, MOTION_PLAY, MOTION_STOP, POSE_TARGET };
enum class ParamId : uint8_t {
    FALL_DEG, REARM_DEG, IMU_STALE_MS, SERVO_STEP, TRIGGER_ROLL_DEG,
    TRIGGER_PITCH_DEG, FOOT_LIFT, SWAY_BASE, SWAY_GAIN, SIGN_ROLL,
    SIGN_PITCH, IMU_SWAP, GAIT_ENABLE, GAIT_STEP_MAX, GAIT_YAW_MAX_DEG, COUNT_
};
struct Command {
    CmdType  type;
    ParamId  param;
    uint8_t  bus, slot;     // SET_TRIM; slot juga = slot motion untuk MOTION_PLAY
    float    value;         // SET_PARAM / SET_TRIM / GAIT_INTENT vx (mm/langkah)
    float    vy, yawDeg;    // GAIT_INTENT (vy belum dipakai; yaw = derajat per langkah)
    uint16_t ttlMs;         // GAIT_INTENT / POSE_TARGET: kedaluwarsa setelah ttl
    uint32_t poseMask;      // POSE_TARGET: bit (bus*11+slot) = sendi dikendalikan
    int16_t  pose[2][SERVO_SLOTS];   // POSE_TARGET: offset sendi (ruang src)
};

// ---- Masukan IMU per siklus (dari HAL) -------------------------------------
struct ImuInput {
    bool     sensorOk;      // driver IMU sehat
    bool     hasSample;     // ada sampel (boleh sama dengan siklus lalu)
    float    roll, pitch;   // rad, frame sensor (sebelum swap/sign/offset)
    int64_t  tUs;           // waktu sampel (jam yang sama dengan Clock)
    uint32_t seq;           // naik per sampel baru
    uint16_t rateHz;
};

// ---- Telemetry ------------------------------------------------------------
struct Telemetry {
    uint32_t seq, ms;
    Mode     mode;
    float    roll, pitch, rawRoll, rawPitch, offRoll, offPitch;
    int32_t  imuAgeMs;
    uint16_t imuHz;
    bool     imuFreshFlag, imuInit, calDone;
    float    fwct, fwctEnd, dxi, dyi, dxis, dyis, autoH, fh, swx, swy, swMax, waist;
    int8_t   jikuasi;
    ServoPositions kin;
    int16_t  cmd[2][SERVO_SLOTS];
    int16_t  fb[2][SERVO_SLOTS];
    int16_t  trim[2][SERVO_SLOTS];
    uint8_t  fbValid[2][SERVO_SLOTS];
    uint16_t fbAgeMs[2][SERVO_SLOTS];
    uint32_t fbMiss[2][SERVO_SLOTS];
    RuntimeConfig cfg;
    uint32_t cfgGen;
    uint32_t loopAvgUs, loopMaxUs, loopOverruns, servoPhaseUs;
    uint32_t ctrlStackFree;   // diisi HAL (0 = tidak diketahui)
    // Fase 1
    bool     intentActive;
    float    intentVx, intentYawDeg;
    int8_t   motionSlot;      // -1 = tidak memutar
    uint8_t  motionFrame;
    uint32_t blackboxDropped;
};

// ---- Utilitas -------------------------------------------------------------
static inline float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline bool    isFiniteVal(float v) { return (v == v) && (v < 1.0e30f) && (v > -1.0e30f); }

}  // namespace la
