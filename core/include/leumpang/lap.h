// =============================================================================
//  LAP - Leumpang Ajeg Protocol v1
//  Transport-agnostic: USB CDC (HP OTG / PC), TCP (Wi-Fi), UART.
//  Spesifikasi lengkap: docs/PROTOCOL.md
//
//  Frame di kabel:  COBS( type | seq | body... | crc16_le ) 0x00
//  crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) atas type..body.
//  Semua angka little-endian; sudut dalam DERAJAT (f32), panjang mm.
// =============================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "leumpang/types.h"

namespace la {
namespace lap {

constexpr uint8_t  VERSION   = 1;
constexpr size_t   MAX_BODY  = 240;
constexpr size_t   MAX_RAW   = MAX_BODY + 4;              // type + seq + body + crc
constexpr size_t   MAX_FRAME = MAX_RAW + MAX_RAW / 254 + 2;  // COBS + delimiter
constexpr uint16_t TCP_PORT  = 5760;

enum MsgType : uint8_t {
    HELLO        = 0x01,   // dua arah
    PING         = 0x02,
    PONG         = 0x03,
    ACK          = 0x04,
    TELEMETRY    = 0x10,   // robot -> host
    SERVO_STATE  = 0x11,
    GAIT_STATE   = 0x12,
    CONFIG       = 0x13,
    LOG          = 0x14,
    CMD          = 0x20,   // host -> robot
    SET_PARAM    = 0x21,
    SET_TRIM     = 0x22,
    STREAM_CFG   = 0x23,
    GAIT_INTENT  = 0x30,   // HP -> robot (Fase 1)
    PERCEPTION   = 0x31,   // HP -> robot (Fase 3): rintangan per sektor + orang
    MOTION_BEGIN = 0x40,   // (Fase 1)
    MOTION_CHUNK = 0x41,
    MOTION_END   = 0x42,
    MOTION_PLAY  = 0x43,
    MOTION_STATUS= 0x44,   // robot -> host: isi 4 slot
    POSE_TARGET  = 0x45,   // host -> robot (Fase 2): pose langsung, mode 820
    SERVO_TOOL   = 0x46,   // host -> robot (Fase 4A): alat servo ICS (mode 830)
    SERVO_TOOL_RESULT = 0x47,  // robot -> host
    FILE_LIST    = 0x80,   // host -> robot (Fase 4A): berkas SD
    FILE_READ    = 0x81, FILE_WRITE = 0x82, FILE_DELETE = 0x83, FILE_MKDIR = 0x84, FILE_RENAME = 0x85, FILE_STAT = 0x86,
    FILE_ENTRIES = 0x88,   // robot -> host
    FILE_DATA    = 0x89, FILE_RESULT = 0x8A,
    EXT          = 0x60,   // dua arah (Fase 3): Leumpang Bus - topik modul/sensor tambahan
    SIM_IMU      = 0x70,   // hanya simulator
    SIM_FAULT    = 0x71,
};

enum AckStatus : uint8_t { OK = 0, BAD_LEN = 1, UNSUPPORTED = 2, REJECTED_STATE = 3, BUSY = 4, BAD_VALUE = 5 };
enum Role  : uint8_t { ROLE_ROBOT = 0, ROLE_PHONE = 1, ROLE_DESKTOP = 2, ROLE_SIM = 3, ROLE_TOOL = 4 };
enum Board : uint8_t { BOARD_UNKNOWN = 0, BOARD_ESP32S3 = 1, BOARD_ESP32P4 = 2, BOARD_STM32H562 = 3, BOARD_SIM = 4 };
enum CmdId : uint8_t { CMD_RESET = 0, CMD_GO = 1, CMD_STOP = 2, CMD_CAL = 3, CMD_GET_CONFIG = 4, CMD_HELLO = 5,
                      CMD_MOTION_LIST = 6, CMD_REBOOT = 7 };
enum Caps  : uint32_t {
    CAP_WIFI = 1u << 0, CAP_SD = 1u << 1, CAP_GAIT_INTENT = 1u << 2, CAP_MOTION = 1u << 3,
    CAP_SIM = 1u << 4, CAP_LCD = 1u << 5, CAP_USB_CDC = 1u << 6, CAP_LIVE_POSE = 1u << 7,
    CAP_PERCEPTION = 1u << 8, CAP_EXT = 1u << 9, CAP_FILES = 1u << 10, CAP_PROFILE = 1u << 11,
    CAP_SERVO_TOOL = 1u << 12,
};
// Flag TELEMETRY
enum TeleFlags : uint16_t {
    TF_IMU_FRESH = 1u << 0, TF_IMU_INIT = 1u << 1, TF_CAL_DONE = 1u << 2,
    TF_PERSIST_PENDING = 1u << 3, TF_JIKU_LEFT = 1u << 4, TF_INTENT_ACTIVE = 1u << 5,
    TF_OBSTACLE_BLOCK = 1u << 6,
};

// ---- primitif ---------------------------------------------------------------
uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF);
size_t   cobsEncode(const uint8_t* in, size_t n, uint8_t* out);        // out >= n + n/254 + 1
size_t   cobsDecode(const uint8_t* in, size_t n, uint8_t* out);        // 0 = rusak

struct Packet {
    uint8_t type = 0, seq = 0;
    uint8_t len = 0;
    uint8_t body[MAX_BODY];
};

// Encode paket menjadi frame lengkap (termasuk 0x00). Return panjang, 0 bila tidak muat.
size_t encodeFrame(const Packet& p, uint8_t* out, size_t cap);

// Decoder stream byte-demi-byte. Byte sampah/teks di antara frame dibuang otomatis.
class Decoder {
public:
    bool feed(uint8_t b, Packet& out);     // true = paket valid siap
    uint32_t frames = 0, crcErrors = 0, overflows = 0, cobsErrors = 0;
private:
    uint8_t buf_[MAX_FRAME];
    size_t  n_ = 0;
    bool    overflow_ = false;
};

// ---- serialisasi body -------------------------------------------------------
class Writer {
public:
    Writer(uint8_t* p, size_t cap) : p_(p), cap_(cap) {}
    void u8(uint8_t v)   { put(&v, 1); }
    void i8(int8_t v)    { put(&v, 1); }
    void u16(uint16_t v) { uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)}; put(b, 2); }
    void i16(int16_t v)  { u16((uint16_t)v); }
    void u32(uint32_t v) { uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)}; put(b, 4); }
    void f32(float v)    { uint32_t u; memcpy(&u, &v, 4); u32(u); }
    void fixstr(const char* s, size_t n) { for (size_t i = 0; i < n; ++i) u8(s && i < strlen(s) ? (uint8_t)s[i] : 0); }
    void bytes(const void* d, size_t n) { put(d, n); }
    size_t size() const { return n_; }
    bool ok() const { return ok_; }
private:
    void put(const void* d, size_t n) { if (n_ + n > cap_) { ok_ = false; return; } memcpy(p_ + n_, d, n); n_ += n; }
    uint8_t* p_; size_t cap_; size_t n_ = 0; bool ok_ = true;
};

class Reader {
public:
    Reader(const uint8_t* p, size_t n) : p_(p), n_(n) {}
    uint8_t  u8()  { uint8_t v = 0; get(&v, 1); return v; }
    int8_t   i8()  { return (int8_t)u8(); }
    uint16_t u16() { uint8_t b[2] = {0, 0}; get(b, 2); return (uint16_t)(b[0] | (b[1] << 8)); }
    int16_t  i16() { return (int16_t)u16(); }
    uint32_t u32() { uint8_t b[4] = {0, 0, 0, 0}; get(b, 4); return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24); }
    float    f32() { const uint32_t u = u32(); float v; memcpy(&v, &u, 4); return v; }
    void fixstr(char* out, size_t n) { for (size_t i = 0; i < n; ++i) out[i] = (char)u8(); out[n] = 0; }
    size_t remaining() const { return ok_ ? n_ - i_ : 0; }
    bool ok() const { return ok_; }
private:
    void get(void* d, size_t n) { if (i_ + n > n_) { ok_ = false; memset(d, 0, n); return; } memcpy(d, p_ + i_, n); i_ += n; }
    const uint8_t* p_; size_t n_; size_t i_ = 0; bool ok_ = true;
};

// ---- pesan ----------------------------------------------------------------
struct Hello {
    uint8_t  protoVer = VERSION, role = ROLE_ROBOT, board = BOARD_UNKNOWN;
    uint8_t  fwMajor = 0, fwMinor = 0, fwPatch = 0;
    uint32_t caps = 0;
    char     name[17] = {0};
    void enc(Writer& w) const { w.u8(protoVer); w.u8(role); w.u8(board); w.u8(fwMajor); w.u8(fwMinor); w.u8(fwPatch); w.u32(caps); w.fixstr(name, 16); }
    bool dec(Reader& r) { protoVer = r.u8(); role = r.u8(); board = r.u8(); fwMajor = r.u8(); fwMinor = r.u8(); fwPatch = r.u8(); caps = r.u32(); r.fixstr(name, 16); return r.ok(); }
};

struct Ack {
    uint8_t ackSeq = 0, ackType = 0, status = OK;
    void enc(Writer& w) const { w.u8(ackSeq); w.u8(ackType); w.u8(status); }
    bool dec(Reader& r) { ackSeq = r.u8(); ackType = r.u8(); status = r.u8(); return r.ok(); }
};

struct TelemetryMsg {
    uint32_t ms = 0;
    uint16_t mode = 0, flags = 0;
    float    rollDeg = 0, pitchDeg = 0;
    int16_t  imuAgeMs = -1;
    uint16_t imuHz = 0;
    float    fwct = 0, autoH = 0;
    uint16_t loopAvgUs = 0, loopMaxUs = 0, servoPhaseUs = 0;
    uint32_t overruns = 0;
    uint8_t  fbOk = 0;
    uint32_t cfgGen = 0;
    void enc(Writer& w) const {
        w.u32(ms); w.u16(mode); w.u16(flags); w.f32(rollDeg); w.f32(pitchDeg); w.i16(imuAgeMs); w.u16(imuHz);
        w.f32(fwct); w.f32(autoH); w.u16(loopAvgUs); w.u16(loopMaxUs); w.u16(servoPhaseUs); w.u32(overruns);
        w.u8(fbOk); w.u32(cfgGen);
    }
    bool dec(Reader& r) {
        ms = r.u32(); mode = r.u16(); flags = r.u16(); rollDeg = r.f32(); pitchDeg = r.f32(); imuAgeMs = r.i16();
        imuHz = r.u16(); fwct = r.f32(); autoH = r.f32(); loopAvgUs = r.u16(); loopMaxUs = r.u16();
        servoPhaseUs = r.u16(); overruns = r.u32(); fbOk = r.u8(); cfgGen = r.u32(); return r.ok();
    }
};

struct ServoStateMsg {                 // 22 servo x 7 byte = 154 B
    int16_t cmd[2][SERVO_SLOTS]{}, fb[2][SERVO_SLOTS]{}, trim[2][SERVO_SLOTS]{};
    uint8_t valid[2][SERVO_SLOTS]{};
    void enc(Writer& w) const {
        for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
            w.i16(cmd[b][s]); w.i16(fb[b][s]); w.i16(trim[b][s]); w.u8(valid[b][s]); }
    }
    bool dec(Reader& r) {
        for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
            cmd[b][s] = r.i16(); fb[b][s] = r.i16(); trim[b][s] = r.i16(); valid[b][s] = r.u8(); }
        return r.ok();
    }
};

struct GaitStateMsg {
    float fwct = 0, fwctEnd = 0, dxi = 0, dyi = 0, dxis = 0, dyis = 0, autoH = 0, fh = 0, swx = 0, swy = 0, swMax = 0, waist = 0;
    int8_t jikuasi = 0;
    int16_t kin[22]{};                 // ServoPositions urutan deklarasi: K0[2],K1,K2,H,A0,A1,U0,U1,U2,E,head,waist
    void enc(Writer& w) const {
        const float f[] = {fwct, fwctEnd, dxi, dyi, dxis, dyis, autoH, fh, swx, swy, swMax, waist};
        for (float v : f) w.f32(v);
        w.i8(jikuasi);
        for (int16_t v : kin) w.i16(v);
    }
    bool dec(Reader& r) {
        float* f[] = {&fwct, &fwctEnd, &dxi, &dyi, &dxis, &dyis, &autoH, &fh, &swx, &swy, &swMax, &waist};
        for (float* v : f) *v = r.f32();
        jikuasi = r.i8();
        for (int16_t& v : kin) v = r.i16();
        return r.ok();
    }
};

struct ConfigMsg {                     // nilai dalam satuan ParamId (derajat, ms, mm)
    float v[(int)ParamId::COUNT_]{};
    uint32_t cfgGen = 0, savedGen = 0;
    void enc(Writer& w) const { w.u8((uint8_t)ParamId::COUNT_); for (float x : v) w.f32(x); w.u32(cfgGen); w.u32(savedGen); }
    bool dec(Reader& r) {
        const uint8_t n = r.u8();
        for (uint8_t i = 0; i < n; ++i) { const float x = r.f32(); if (i < (uint8_t)ParamId::COUNT_) v[i] = x; }
        cfgGen = r.u32(); savedGen = r.u32(); return r.ok();
    }
};

struct CmdMsg       { uint8_t cmd = 0;
    void enc(Writer& w) const { w.u8(cmd); } bool dec(Reader& r) { cmd = r.u8(); return r.ok(); } };
struct SetParamMsg  { uint8_t param = 0; float value = 0;
    void enc(Writer& w) const { w.u8(param); w.f32(value); } bool dec(Reader& r) { param = r.u8(); value = r.f32(); return r.ok(); } };
struct SetTrimMsg   { uint8_t bus = 0, slot = 0; int16_t value = 0;
    void enc(Writer& w) const { w.u8(bus); w.u8(slot); w.i16(value); } bool dec(Reader& r) { bus = r.u8(); slot = r.u8(); value = r.i16(); return r.ok(); } };
struct StreamCfgMsg { uint8_t teleHz = 20, servoHz = 5, gaitHz = 0, logOn = 1;
    void enc(Writer& w) const { w.u8(teleHz); w.u8(servoHz); w.u8(gaitHz); w.u8(logOn); }
    bool dec(Reader& r) { teleHz = r.u8(); servoHz = r.u8(); gaitHz = r.u8(); logOn = r.u8(); return r.ok(); } };
struct GaitIntentMsg { float vx = 0, vy = 0, yawDeg = 0; uint16_t ttlMs = 0;   // mm/langkah, deg/langkah, ttl <= 1000
    void enc(Writer& w) const { w.f32(vx); w.f32(vy); w.f32(yawDeg); w.u16(ttlMs); }
    bool dec(Reader& r) { vx = r.f32(); vy = r.f32(); yawDeg = r.f32(); ttlMs = r.u16(); return r.ok(); } };
struct MotionBeginMsg { uint8_t slot = 0; uint16_t total = 0;
    void enc(Writer& w) const { w.u8(slot); w.u16(total); } bool dec(Reader& r) { slot = r.u8(); total = r.u16(); return r.ok(); } };
struct MotionEndMsg   { uint8_t slot = 0; uint8_t save = 1;           // save=1 -> HAL simpan ke SD
    void enc(Writer& w) const { w.u8(slot); w.u8(save); } bool dec(Reader& r) { slot = r.u8(); save = r.remaining() ? r.u8() : 1; return r.ok(); } };
struct MotionPlayMsg  { uint8_t slot = 0, play = 1;
    void enc(Writer& w) const { w.u8(slot); w.u8(play); } bool dec(Reader& r) { slot = r.u8(); play = r.u8(); return r.ok(); } };
struct MotionStatusMsg { uint8_t state[4]{}, nFrames[4]{}; char name[4][17]{};
    void enc(Writer& w) const { for (int i = 0; i < 4; ++i) { w.u8(state[i]); w.u8(nFrames[i]); w.fixstr(name[i], 16); } }
    bool dec(Reader& r) { for (int i = 0; i < 4; ++i) { state[i] = r.u8(); nFrames[i] = r.u8(); r.fixstr(name[i], 16); } return r.ok(); } };
// Rintangan dari HP: n sektor merata dari -halfFov (kanan) .. +halfFov (kiri), jarak mm (0xFFFF = bebas/tak diketahui).
struct PerceptionMsg {
    static constexpr int MAX_SECTORS = 16;
    uint16_t ttlMs = 0; uint8_t flags = 0, n = 0; int16_t halfFovDeg10 = 300;
    uint16_t distMm[MAX_SECTORS]{};
    int16_t personBearingDeg10 = 0; uint16_t personDistMm = 0xFFFF;
    void enc(Writer& w) const {
        w.u16(ttlMs); w.u8(flags); w.u8(n); w.i16(halfFovDeg10);
        for (int i = 0; i < n && i < MAX_SECTORS; ++i) w.u16(distMm[i]);
        w.i16(personBearingDeg10); w.u16(personDistMm);
    }
    bool dec(Reader& r) {
        ttlMs = r.u16(); flags = r.u8(); n = r.u8(); halfFovDeg10 = r.i16();
        if (n == 0 || n > MAX_SECTORS) return false;
        for (int i = 0; i < n; ++i) distMm[i] = r.u16();
        personBearingDeg10 = r.i16(); personDistMm = r.u16();
        return r.ok();
    }
    // jarak terdekat di sektor yang pusatnya dalam +-coneDeg dari depan
    uint16_t frontMin(float coneDeg = 30.0f) const {
        uint16_t m = 0xFFFF;
        const float half = halfFovDeg10 / 10.0f, w = 2.0f * half / (float)(n ? n : 1);
        for (int i = 0; i < n; ++i) {
            const float c = -half + ((float)i + 0.5f) * w;
            if (c >= -coneDeg && c <= coneDeg && distMm[i] < m) m = distMm[i];
        }
        return m;
    }
};
// Leumpang Bus di atas LAP: topik 16 bit + data mentah. 0x0000-0x0FFF dicadangkan core, 0x1000+ modul pengguna.
struct ExtMsg {
    static constexpr size_t MAX_DATA = MAX_BODY - 2;
    uint16_t topic = 0; uint8_t len = 0; uint8_t data[MAX_DATA]{};
    void enc(Writer& w) const { w.u16(topic); w.bytes(data, len); }
    bool dec(Reader& r) {
        topic = r.u16();
        const size_t k = r.remaining();
        len = (uint8_t)(k > MAX_DATA ? MAX_DATA : k);
        for (size_t i = 0; i < len; ++i) data[i] = r.u8();
        return r.ok();
    }
};

struct PoseTargetMsg {                    // 4 + 2 + 44 = 50 B
    uint32_t mask = 0; uint16_t ttlMs = 0; int16_t joint[2][SERVO_SLOTS]{};
    void enc(Writer& w) const { w.u32(mask); w.u16(ttlMs); for (auto& b : joint) for (int16_t v : b) w.i16(v); }
    bool dec(Reader& r) { mask = r.u32(); ttlMs = r.u16(); for (auto& b : joint) for (int16_t& v : b) v = r.i16(); return r.ok(); } };
struct SimImuMsg    { float rollDeg = 0, pitchDeg = 0; uint8_t sensorOk = 1;
    void enc(Writer& w) const { w.f32(rollDeg); w.f32(pitchDeg); w.u8(sensorOk); }
    bool dec(Reader& r) { rollDeg = r.f32(); pitchDeg = r.f32(); sensorOk = r.u8(); return r.ok(); } };
struct SimFaultMsg  { uint8_t bus = 0, slot = 0, dead = 0;
    void enc(Writer& w) const { w.u8(bus); w.u8(slot); w.u8(dead); }
    bool dec(Reader& r) { bus = r.u8(); slot = r.u8(); dead = r.u8(); return r.ok(); } };

// Bangun Packet dari struct pesan.
template <typename M>
inline bool makePacket(Packet& p, uint8_t type, uint8_t seq, const M& m) {
    p.type = type; p.seq = seq;
    Writer w(p.body, MAX_BODY);
    m.enc(w);
    p.len = (uint8_t)w.size();
    return w.ok();
}
template <typename M>
inline bool parsePacket(const Packet& p, M& m) {
    Reader r(p.body, p.len);
    return m.dec(r);
}

// Konversi bersama Telemetry -> pesan.
void fillTelemetry(const Telemetry& t, uint32_t savedGen, TelemetryMsg& m);
void fillServoState(const Telemetry& t, ServoStateMsg& m);
void fillGaitState(const Telemetry& t, GaitStateMsg& m);
void fillConfig(const RuntimeConfig& c, uint32_t cfgGen, uint32_t savedGen, ConfigMsg& m);

}  // namespace lap
}  // namespace la
