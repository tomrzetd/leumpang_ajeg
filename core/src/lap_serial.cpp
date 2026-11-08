#include "leumpang/lap_serial.h"

namespace la {
namespace lap {

void SerialLink::epWrite(void* ctx, const uint8_t* d, size_t n) {
    SerialLink* s = (SerialLink*)ctx;
    if (s->io_.write) s->io_.write(s->io_.ctx, d, n);
}

void SerialLink::init(const Io& io, const LapEndpoint::Host& host, const Hello& self) {
    io_ = io;
    userHost_ = host;
    LapEndpoint::Host h = host;
    h.write = epWrite;                 // endpoint menulis lewat link ini
    h.writeCtx = this;                 // callback lain tetap memakai ctx pengguna
    ep_.init(h, self);
    lapMode_ = false; capturing_ = false;
}

void SerialLink::onRx(const uint8_t* d, size_t n, uint32_t nowMs) {
    for (size_t i = 0; i < n; ++i) {
        const uint8_t b = d[i];
        if (lapMode_ || capturing_) {
            const uint32_t before = ep_.decoder().frames;
            ep_.onBytes(&b, 1, nowMs);
            if (ep_.decoder().frames != before) { lapMode_ = true; capturing_ = true; }
            continue;
        }
        if (b == 0) {                  // awal frame
            capturing_ = true; captureStartMs_ = nowMs;
            ep_.onBytes(&b, 1, nowMs);
            continue;
        }
        if (io_.key) io_.key(io_.ctx, (char)b);
    }
}

void SerialLink::logLine(const char* line) {
    if (!line) return;
    if (lapMode_) ep_.sendLog(line);
    else if (io_.write) io_.write(io_.ctx, (const uint8_t*)line, strlen(line));
}

void SerialLink::poll(uint32_t nowMs) {
    if (capturing_ && !lapMode_ && nowMs - captureStartMs_ > CAPTURE_TIMEOUT_MS) {
        capturing_ = false;            // 0x00 nyasar tanpa frame valid: kembali ke teks
        static const uint8_t z = 0;
        ep_.onBytes(&z, 1, nowMs);     // kosongkan buffer decoder
    }
    ep_.poll(nowMs);
    if (lapMode_ && !ep_.rxRecent(nowMs)) { lapMode_ = false; capturing_ = false; }
}

}  // namespace lap
}  // namespace la
