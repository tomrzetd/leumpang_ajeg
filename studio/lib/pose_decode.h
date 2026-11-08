// =============================================================================
//  Leumpang Studio - pra/pasca-proses model pose MoveNet SinglePose (ONNX).
//  Masukan model: gambar persegi inSize x inSize (192 Lightning / 256 Thunder), RGB.
//  Keluaran: [1,1,17,3] = (y, x, skor) ternormalisasi terhadap gambar persegi.
// =============================================================================
#pragma once
#include "retarget.h"

namespace la::studio {

struct Letterbox {
    int   inSize = 192;
    float scale = 1, padX = 0, padY = 0;   // piksel_masukan = piksel_gambar * scale + pad
};

inline Letterbox letterbox(int imgW, int imgH, int inSize) {
    Letterbox lb; lb.inSize = inSize;
    if (imgW <= 0 || imgH <= 0) return lb;
    lb.scale = (float)inSize / (float)(imgW > imgH ? imgW : imgH);
    lb.padX = 0.5f * ((float)inSize - (float)imgW * lb.scale);
    lb.padY = 0.5f * ((float)inSize - (float)imgH * lb.scale);
    return lb;
}

inline void decodeMoveNet(const float* out17x3, const Letterbox& lb, Keypoint kp[KP_COUNT]) {
    for (int i = 0; i < KP_COUNT; ++i) {
        const float y = out17x3[i * 3 + 0] * (float)lb.inSize, x = out17x3[i * 3 + 1] * (float)lb.inSize;
        kp[i].x = (x - lb.padX) / lb.scale;
        kp[i].y = (y - lb.padY) / lb.scale;
        kp[i].score = out17x3[i * 3 + 2];
    }
}

}  // namespace la::studio
