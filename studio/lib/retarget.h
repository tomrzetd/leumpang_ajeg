// =============================================================================
//  Leumpang Studio - retarget pose manusia (17 keypoint COCO, kamera 2D) ke sendi robot.
//
//  Yang dipetakan: kedua lengan (bahu pitch/roll/yaw, siku) + kepala (yaw).
//  Kaki SENGAJA tidak dipetakan: keseimbangan tetap milik firmware (BALANCE/UVC).
//
//  Kedalaman dari kamera 2D diperkirakan dari pemendekan segmen: panjang referensi
//  segmen = rasio x lebar bahu (dikalibrasi dengan T-pose). Segmen yang memendek
//  dianggap mengarah ke depan (ke kamera), tidak pernah ke belakang.
// =============================================================================
#pragma once
#include "kinematics.h"

#include <cstdint>
#include <vector>

namespace la::studio {

enum Kp : uint8_t { NOSE = 0, L_EYE, R_EYE, L_EAR, R_EAR, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW,
                    L_WRIST, R_WRIST, L_HIP, R_HIP, L_KNEE, R_KNEE, L_ANKLE, R_ANKLE, KP_COUNT };

struct Keypoint { float x = 0, y = 0, score = 0; };      // piksel gambar, y ke bawah

struct RetargetOpts {
    bool  mirror     = true;    // robot meniru seperti cermin (tangan kanan manusia -> tangan kiri robot)
    bool  head       = true;
    float minScore   = 0.3f;
    float upperRatio = 0.80f;   // panjang lengan atas / lebar bahu (T-pose)
    float foreRatio  = 0.75f;   // panjang lengan bawah / lebar bahu
};

struct RetargetResult {
    int16_t  src[SERVO_BUSES][SERVO_SLOTS]{};   // pose lengkap (sendi yang tidak dipetakan = base)
    uint32_t mask = 0;                          // bit bus*11+slot: sendi yang dipetakan frame ini
    bool     armOk[2]{};                        // [robot bus R, robot bus L]
};

// Arah segmen (satuan) di frame tubuh: X depan (ke kamera), Y kiri tubuh, Z atas.
Vec3 segmentDir(const Keypoint& a, const Keypoint& b, float refLenPx);

// Sudut bahu pitch (U0) & roll (U1) dalam radian agar lengan atas robot menunjuk ke 'dir'.
// robotBus 0 = kanan, 1 = kiri. Mengembalikan false bila arah tidak terdefinisi.
bool shoulderAngles(const Vec3& dir, int robotBus, float& pitch, float& roll);

class Retargeter {
public:
    RetargetOpts opts;

    // Hitung pose. 'base' = nilai untuk sendi yang tidak dipetakan (biasanya HOME / pose terakhir).
    RetargetResult solve(const Keypoint kp[KP_COUNT], const JointSrc base) const;

    // T-pose: kedua lengan lurus ke samping. Menyetel upperRatio & foreRatio. false bila keypoint kurang.
    bool calibrateTPose(const Keypoint kp[KP_COUNT]);

private:
    bool solveArm(const Keypoint kp[KP_COUNT], bool humanRight, int robotBus, float shoulderW,
                  JointSrc pose) const;
};

// Filter One-Euro per sendi (ruang src) untuk meredam getar keypoint tanpa lag besar.
class JointFilter {
public:
    float minCutoffHz = 1.2f, beta = 0.004f, dCutoffHz = 1.0f;
    void reset() { init_ = false; }
    void apply(JointSrc v, uint32_t mask, float dtSec);
private:
    bool init_ = false;
    float x_[SERVO_BUSES][SERVO_SLOTS]{}, dx_[SERVO_BUSES][SERVO_SLOTS]{};
};

}  // namespace la::studio
