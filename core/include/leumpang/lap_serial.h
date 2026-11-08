// =============================================================================
//  SerialLink: satu port serial (USB CDC / USB-Serial-JTAG / UART) dipakai bersama
//  oleh konsol teks manusia DAN protokol LAP biner.
//
//  Mode TEKS (awal): byte != 0x00 = tombol konsol; log ditulis sebagai teks.
//  Byte 0x00 memulai penangkapan frame. Bila frame LAP valid diterima -> mode LAP:
//    * semua byte masuk decoder LAP (tombol nonaktif)
//    * log dikirim sebagai paket LOG (tidak ada teks mentah yang merusak stream)
//  Kembali ke mode TEKS bila host diam > 3 s (LapEndpoint::LINK_TIMEOUT_MS).
//  Host (HP/PC) wajib mengawali sesi dengan 0x00 lalu HELLO - sudah otomatis
//  karena setiap frame diakhiri 0x00 dan klien mengirim 0x00 pembuka.
//
//  Semua method dipanggil dari SATU thread (task konsol / main loop).
// =============================================================================
#pragma once
#include "leumpang/lap_endpoint.h"

namespace la {
namespace lap {

class SerialLink {
public:
    struct Io {
        void* ctx = nullptr;
        void (*write)(void* ctx, const uint8_t* d, size_t n) = nullptr;
        void (*key)(void* ctx, char c) = nullptr;
    };
    static constexpr uint32_t CAPTURE_TIMEOUT_MS = 500;

    void init(const Io& io, const LapEndpoint::Host& host, const Hello& self);
    void onRx(const uint8_t* d, size_t n, uint32_t nowMs);
    void logLine(const char* line);           // teks atau paket LOG tergantung mode
    void poll(uint32_t nowMs);
    bool lapMode() const { return lapMode_; }
    LapEndpoint& endpoint() { return ep_; }

private:
    static void epWrite(void* ctx, const uint8_t* d, size_t n);
    Io          io_{};
    LapEndpoint ep_;
    LapEndpoint::Host userHost_{};
    bool        lapMode_ = false;
    bool        capturing_ = false;
    uint32_t    captureStartMs_ = 0;
};

}  // namespace lap
}  // namespace la
