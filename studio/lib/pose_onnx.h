// =============================================================================
//  Leumpang Studio - inferensi MoveNet SinglePose lewat ONNX Runtime (CPU).
//  Hanya dikompilasi bila LA_STUDIO_POSE aktif. Tipe & ukuran masukan dibaca dari model
//  (int32 / uint8 / float32, NHWC atau NCHW, 192 atau 256), jadi berbagai ekspor MoveNet bisa dipakai.
// =============================================================================
#pragma once
#include "pose_decode.h"

#include <memory>
#include <string>
#include <vector>

namespace la::studio {

class PoseOnnx {
public:
    PoseOnnx();
    ~PoseOnnx();
    bool load(const std::string& utf8Path, std::string* err);
    bool loaded() const;
    int  inputSize() const { return in_; }
    std::string describe() const { return desc_; }
    // rgb: piksel RGB888 baris demi baris. Mengisi kp dalam koordinat gambar asli.
    bool run(const uint8_t* rgb, int w, int h, int strideBytes, Keypoint kp[KP_COUNT], std::string* err);

private:
    struct Impl;
    std::unique_ptr<Impl> d_;
    int in_ = 192;
    bool nchw_ = false;
    std::string desc_;
    std::vector<uint8_t> square_;
};

}  // namespace la::studio
