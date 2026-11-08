#include "pose_onnx.h"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cstring>
#include <exception>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#endif

namespace la::studio {

struct PoseOnnx::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "leumpang-studio"};
    std::unique_ptr<Ort::Session> session;
    std::string inName, outName;
    ONNXTensorElementDataType inType = ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32;
};

PoseOnnx::PoseOnnx() : d_(new Impl) {}
PoseOnnx::~PoseOnnx() = default;
bool PoseOnnx::loaded() const { return d_->session != nullptr; }

bool PoseOnnx::load(const std::string& path, std::string* err) {
    try {
        Ort::SessionOptions so;
        so.SetIntraOpNumThreads(2);
        so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef _WIN32
        const int wn = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
        std::wstring w((size_t)(wn > 0 ? wn - 1 : 0), L'\0');
        if (wn > 1) MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, &w[0], wn);
        d_->session.reset(new Ort::Session(d_->env, w.c_str(), so));
#else
        d_->session.reset(new Ort::Session(d_->env, path.c_str(), so));
#endif
        Ort::AllocatorWithDefaultOptions alloc;
        d_->inName = d_->session->GetInputNameAllocated(0, alloc).get();
        d_->outName = d_->session->GetOutputNameAllocated(0, alloc).get();
        auto info = d_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        d_->inType = info.GetElementType();
        const std::vector<int64_t> shp = info.GetShape();
        if (shp.size() != 4) throw std::runtime_error("masukan model harus 4 dimensi");
        nchw_ = shp[1] == 3;
        const int64_t side = nchw_ ? shp[2] : shp[1];
        in_ = side > 0 ? (int)side : 192;
        auto oinfo = d_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
        size_t outElems = 1;
        for (int64_t v : oinfo.GetShape()) outElems *= (size_t)(v > 0 ? v : 1);
        if (outElems != (size_t)KP_COUNT * 3) throw std::runtime_error("keluaran bukan [1,1,17,3] (bukan MoveNet SinglePose?)");
        const char* tn = d_->inType == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ? "float32"
                       : d_->inType == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8 ? "uint8" : "int32";
        desc_ = std::string("MoveNet ") + std::to_string(in_) + "x" + std::to_string(in_) + " " + tn + (nchw_ ? " NCHW" : " NHWC");
        square_.assign((size_t)in_ * in_ * 3, 0);
        return true;
    } catch (const std::exception& e) {
        d_->session.reset();
        if (err) *err = e.what();
        return false;
    }
}

bool PoseOnnx::run(const uint8_t* rgb, int w, int h, int stride, Keypoint kp[KP_COUNT], std::string* err) {
    if (!d_->session) { if (err) *err = "model belum dimuat"; return false; }
    const Letterbox lb = letterbox(w, h, in_);
    // letterbox + resize bilinear ke persegi in_ x in_ (latar hitam)
    std::fill(square_.begin(), square_.end(), 0);
    const int x0 = (int)lb.padX, y0 = (int)lb.padY;
    const int sw = (int)((float)w * lb.scale + 0.5f), sh = (int)((float)h * lb.scale + 0.5f);
    for (int y = 0; y < sh && y0 + y < in_; ++y) {
        const float fy = ((float)y + 0.5f) / lb.scale - 0.5f;
        const int iy = fy < 0 ? 0 : (int)fy; const int iy1 = iy + 1 < h ? iy + 1 : h - 1; const float ty = fy - (float)iy;
        for (int x = 0; x < sw && x0 + x < in_; ++x) {
            const float fx = ((float)x + 0.5f) / lb.scale - 0.5f;
            const int ix = fx < 0 ? 0 : (int)fx; const int ix1 = ix + 1 < w ? ix + 1 : w - 1; const float tx = fx - (float)ix;
            uint8_t* o = &square_[((size_t)(y0 + y) * in_ + (x0 + x)) * 3];
            const uint8_t* a = rgb + (size_t)iy * stride + (size_t)ix * 3;
            const uint8_t* b = rgb + (size_t)iy * stride + (size_t)ix1 * 3;
            const uint8_t* c = rgb + (size_t)iy1 * stride + (size_t)ix * 3;
            const uint8_t* e = rgb + (size_t)iy1 * stride + (size_t)ix1 * 3;
            for (int k = 0; k < 3; ++k) {
                const float top = (float)a[k] + ((float)b[k] - (float)a[k]) * tx;
                const float bot = (float)c[k] + ((float)e[k] - (float)c[k]) * tx;
                o[k] = (uint8_t)(top + (bot - top) * ty + 0.5f);
            }
        }
    }
    try {
        Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        const int64_t shape[4] = {1, nchw_ ? 3 : in_, nchw_ ? in_ : in_, nchw_ ? in_ : 3};
        const size_t n = square_.size();
        std::vector<int32_t> vi; std::vector<float> vf; std::vector<uint8_t> vu;
        Ort::Value input{nullptr};
        auto at = [&](size_t i) -> uint8_t {                         // indeks NHWC atau NCHW
            if (!nchw_) return square_[i];
            const size_t c = i / ((size_t)in_ * in_), r = i % ((size_t)in_ * in_);
            return square_[r * 3 + c];
        };
        if (d_->inType == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
            vf.resize(n); for (size_t i = 0; i < n; ++i) vf[i] = (float)at(i);
            input = Ort::Value::CreateTensor<float>(mem, vf.data(), n, shape, 4);
        } else if (d_->inType == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
            vu.resize(n); for (size_t i = 0; i < n; ++i) vu[i] = at(i);
            input = Ort::Value::CreateTensor<uint8_t>(mem, vu.data(), n, shape, 4);
        } else {
            vi.resize(n); for (size_t i = 0; i < n; ++i) vi[i] = (int32_t)at(i);
            input = Ort::Value::CreateTensor<int32_t>(mem, vi.data(), n, shape, 4);
        }
        const char* inNames[] = {d_->inName.c_str()};
        const char* outNames[] = {d_->outName.c_str()};
        auto out = d_->session->Run(Ort::RunOptions{nullptr}, inNames, &input, 1, outNames, 1);
        decodeMoveNet(out[0].GetTensorData<float>(), lb, kp);
        return true;
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
}

}  // namespace la::studio
