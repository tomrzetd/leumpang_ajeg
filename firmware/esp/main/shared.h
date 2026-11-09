// =============================================================================
//  Firmware ESP (S3/P4): antarmuka antar-modul
//  Logika robot ada di leumpang-core (namespace la); file ini menyediakan nama
//  pendek untuk modul board (web, UI, IMU, persistensi).
// =============================================================================
#pragma once
#include "leumpang/profile.h"
#include "leumpang/files.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "leumpang/types.h"

// ---- nama dari core --------------------------------------------------------
using la::Mode;
using la::RuntimeConfig;
using la::ServoPositions;
using la::ParamId;
using la::clampf;
using la::clampi;
using la::isFiniteVal;
using la::kRad2Deg;
using la::kPi;
using la::SERVO_SLOTS;
using la::CONTROL_PERIOD_MS;
using TelemetrySnapshot = la::Telemetry;
using RobotCommand      = la::Command;
using RobotCmdType      = la::CmdType;

inline const char* mode_name(Mode m)        { return la::modeName(m); }
inline const char* mode_short_name(Mode m)  { return la::modeShortName(m); }
inline bool        mode_is_gait(Mode m)     { return la::modeIsGait(m); }
inline bool        mode_persist_safe(Mode m){ return la::modePersistSafe(m); }
inline RuntimeConfig runtime_config_default() { return la::runtimeConfigDefault(); }

namespace la { class MotionStore; template <typename T, uint32_t N> class SpscRing; struct BlackboxRecord; }

// ---- Kontrol (control.cpp, core1) ------------------------------------------
void control_init(const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS], const la::RobotProfile* profile);
bool control_start();
bool control_queue_cmd(const RobotCommand& c);     // non-blocking; false bila penuh
bool control_push_key(char c);
TelemetrySnapshot telemetry_copy();
la::MotionStore& control_motions();
void control_set_blackbox(la::SpscRing<la::BlackboxRecord, 256>* ring);   // sebelum control_start

// ---- Konsol & SD (console.cpp, storage.cpp) ---------------------------------
void console_early_init();                 // queue log (panggil paling awal)
void console_start();
void storage_mount();                      // mount SD + muat profil robot (SEBELUM control_init)
void storage_init();                       // muat motion, buka blackbox (setelah control_init)
const la::RobotProfile* storage_profile(); // profil dari /sd/config/robot.json (nullptr = bawaan)
la::FileOps* storage_files();              // layanan berkas LAP (nullptr = tanpa SD)
bool storage_sd_ok();
void storage_save_motion(int slot, const uint8_t* d, size_t n);

// ---- IMU (imu_icm20948.cpp, core0) -----------------------------------------
struct ImuSample {
    float    roll, pitch;          // rad, frame sensor
    float    qw, qx, qy, qz;
    float    gx, gy, gz;           // rad/s, bias dikoreksi
    int64_t  tUs;
    uint32_t seq;
};
bool     imu_start();
bool     imu_get_latest(ImuSample* out);
bool     imu_is_ok();
uint16_t imu_rate_hz();

// ---- Jaringan & web --------------------------------------------------------
struct NetStatus {
    bool    staConnected;
    int8_t  rssi;
    uint8_t apClients;
    char    apSsid[33];
    char    apIp[16];
    char    staIp[16];
};
void      web_start();
NetStatus net_status_get();
void      wifi_set_sta(const char* ssid, const char* pass);
void      lap_tcp_start();                 // server LAP TCP :5760 (lap_tcp.cpp)
void      lap_forward_log(const char* line);

// ---- Persistensi (main.cpp) ------------------------------------------------
bool     persist_load_cfg(RuntimeConfig* cfg, int16_t trim[2][SERVO_SLOTS]);
bool     persist_load_wifi(char* ssid, size_t ssidLen, char* pass, size_t passLen);
void     persist_request_wifi(const char* ssid, const char* pass);
uint32_t persist_saved_gen();

// ---- UI lokal --------------------------------------------------------------
void local_ui_start();

// ---- Logger non-blocking ---------------------------------------------------
void logPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
