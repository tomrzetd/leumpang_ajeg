// =============================================================================
//  leumpang-core - profil robot (platform servo controller umum).
//
//  Disimpan di SD: /config/robot.json. Dibaca saat boot; tidak ada -> profil bawaan KHR-3HV
//  (dan ditulis ke SD sebagai contoh). Saluran fisik: 2 bus ICS x 11 slot (22 servo).
//
//  {
//    "name": "KHR-3HV leumpang",
//    "kind": "humanoid-khr",          // atau "generic": tanpa gait, hanya pose/gerak/script
//    "servos": [
//      {"bus":0, "slot":4, "id":4, "name":"R ELBOW", "model":"KRS-2552",
//       "center":4800, "sign":1, "min":3500, "max":11500, "home":0},
//      {"bus":1, "slot":9, "present":false}, ...
//    ]
//  }
//  Slot yang tidak disebut = tidak terpasang (generic) / nilai bawaan KHR (humanoid).
// =============================================================================
#pragma once
#include "types.h"

#include <cstddef>
#include <cstdint>

namespace la {

struct JointDef { uint8_t id; int16_t center; int8_t sign; int16_t lo, hi; };

enum class RobotKind : uint8_t { HUMANOID_KHR = 0, GENERIC = 1 };

struct RobotProfile {
    char      name[24];
    RobotKind kind;
    JointDef  joint[SERVO_BUSES][SERVO_SLOTS];
    char      jname[SERVO_BUSES][SERVO_SLOTS][16];
    char      model[SERVO_BUSES][SERVO_SLOTS][12];
    int16_t   home[SERVO_BUSES][SERVO_SLOTS];     // pose HOME (ruang src)
    bool      present[SERVO_BUSES][SERVO_SLOTS];
    bool      fromSd;                             // dimuat dari berkas (bukan bawaan)

    bool humanoid() const { return kind == RobotKind::HUMANOID_KHR; }
    uint8_t presentCount() const;
};

const RobotProfile& defaultProfile();             // KHR-3HV (identik dengan JOINTS/HOME_POSE V4)

// Parse JSON -> profil. Mulai dari bawaan humanoid (slot tak disebut tetap bawaan) atau generic kosong.
// false + pesan galat (Bahasa Indonesia) bila JSON/isi tidak valid.
bool parseProfile(const char* json, size_t n, RobotProfile& out, char* err, size_t errLen);
bool validateProfile(const RobotProfile& p, char* err, size_t errLen);
// Tulis profil sebagai JSON (rapi). Mengembalikan panjang (0 bila buf kurang).
size_t writeProfile(const RobotProfile& p, char* buf, size_t len);

}  // namespace la
