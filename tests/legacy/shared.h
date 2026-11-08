// =============================================================================
//  Tipe & antarmuka antar-modul
//  Aturan kepemilikan state:
//   * State kontrol (servo, gait, IMU terkoreksi, g_cfg, trim) HANYA ditulis oleh
//     task kontrol di core1.
//   * Modul lain berkomunikasi lewat: RobotCommand queue (masuk), key queue (masuk),
//     TelemetrySnapshot (keluar, disalin di bawah spinlock).
// =============================================================================
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

// ---- Mode (nilai sama dengan V3: telemetry kompatibel) ----------------------
enum class Mode : uint16_t {
    RAW_ANGLES = 700, FALLEN = 705, RESET = 710, READY = 720, BALANCE = 730,
    WALK_INIT = 740, WALK_HOLD = 750, WALK_STEP = 760, WALK_END = 770,
    POSE_HOLD = 780, ANGLE_DISP = 790, MIRROR_DBG = 791
};
const char* mode_name(Mode m);
const char* mode_short_name(Mode m);
bool mode_is_gait(Mode m);              // WALK_INIT..WALK_END
bool mode_persist_safe(Mode m);         // aman menulis flash (robot diam / servo free)

// ---- Runtime config (disimpan di NVS) --------------------------------------
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
};
RuntimeConfig runtime_config_default();
void runtime_config_sanitize(RuntimeConfig& c);   // clamp semua field ke rentang aman

struct ServoPositions {
    int16_t K0[2], K1[2], K2[2], H[2], A0[2], A1[2];
    int16_t U0[2], U1[2], U2[2], E[2];
    int16_t head, waist;
};

// ---- Perintah ke task kontrol ----------------------------------------------
enum class RobotCmdType : uint8_t { RESET, GO, STOP, CALIBRATE, SET_PARAM, SET_TRIM };
enum class ParamId : uint8_t {
    FALL_DEG, REARM_DEG, IMU_STALE_MS, SERVO_STEP, TRIGGER_ROLL_DEG,
    TRIGGER_PITCH_DEG, FOOT_LIFT, SWAY_BASE, SWAY_GAIN, SIGN_ROLL,
    SIGN_PITCH, IMU_SWAP
};
struct RobotCommand {
    RobotCmdType type;
    ParamId      param;
    uint8_t      bus, slot;
    float        value;
};
bool control_queue_cmd(const RobotCommand& c);     // non-blocking; false bila penuh
bool control_push_key(char c);                     // karakter dari konsol serial

// ---- Telemetry (core1 -> core0) ------------------------------------------
struct TelemetrySnapshot {
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
    uint16_t fbAgeMs[2][SERVO_SLOTS];             // 65535 = tidak pernah / sangat lama
    uint32_t fbMiss[2][SERVO_SLOTS];
    RuntimeConfig cfg;
    uint32_t cfgGen;                              // naik tiap cfg/trim berubah
    uint32_t loopAvgUs, loopMaxUs, loopOverruns, servoPhaseUs;
    uint32_t ctrlStackFree;
};
TelemetrySnapshot telemetry_copy();

// ---- Kontrol ---------------------------------------------------------------
void control_init(const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS]);
bool control_start();

// ---- IMU (task di core0) ---------------------------------------------------
struct ImuSample {
    float    roll, pitch;          // rad, frame sensor (belum swap/sign/offset)
    float    qw, qx, qy, qz;
    float    gx, gy, gz;           // rad/s, bias dikoreksi
    int64_t  tUs;                  // esp_timer saat sampel
    uint32_t seq;
};
bool     imu_start();
bool     imu_get_latest(ImuSample* out);    // false bila belum ada sampel
bool     imu_is_ok();
uint16_t imu_rate_hz();

// ---- Jaringan & web -------------------------------------------------------
struct NetStatus {
    bool    staConnected;
    int8_t  rssi;
    uint8_t apClients;
    char    apSsid[33];
    char    apIp[16];
    char    staIp[16];
};
void      web_start();                       // Wi-Fi AP(+STA) + HTTP server
NetStatus net_status_get();
void      wifi_set_sta(const char* ssid, const char* pass);   // apply ke RAM, tanpa flash

// ---- Persistensi (main.cpp) ----------------------------------------------
bool     persist_load_cfg(RuntimeConfig* cfg, int16_t trim[2][SERVO_SLOTS]);
bool     persist_load_wifi(char* ssid, size_t ssidLen, char* pass, size_t passLen);
void     persist_request_wifi(const char* ssid, const char* pass);
uint32_t persist_saved_gen();

// ---- UI lokal -------------------------------------------------------------
void local_ui_start();

// ---- Logger non-blocking --------------------------------------------------
void logPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// ---- Utilitas ------------------------------------------------------------
static inline float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline bool    isFiniteVal(float v) { return (v == v) && (v < 1.0e30f) && (v > -1.0e30f); }
