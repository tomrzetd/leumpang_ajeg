// =============================================================================
//  Leumpang Studio - sisi host protokol LAP (tanpa Qt; diuji melawan SimRobot).
//  Transport (TCP / serial / sim) cukup memberi 'write' dan memanggil feed().
// =============================================================================
#pragma once
#include "leumpang/lap.h"

#include <functional>
#include <string>
#include <vector>

namespace la::studio {

class LapClient {
public:
    std::function<void(const uint8_t*, size_t)> write;      // wajib
    std::function<void(uint8_t type)> onPacket;            // setelah state diperbarui
    std::function<void(const std::string&)> onLog;
    std::function<void(const lap::Packet&)> onFileReply;      // FILE_ENTRIES / FILE_DATA / FILE_RESULT
    struct ToolResult { uint8_t req, op, status, bus, id; int32_t value; uint8_t len; uint8_t data[64]; };
    std::function<void(const ToolResult&)> onToolResult;      // SERVO_TOOL_RESULT
    std::function<void(const PadFeedback&)> onPadFeedback;    // PAD_FEEDBACK (studio bertindak sebagai radio)

    struct Rates { uint8_t teleHz = 20, servoHz = 20, gaitHz = 30, logOn = 1; };

    void begin(const Rates& r);                 // 0x00 (masuk mode LAP di serial) + HELLO + STREAM_CFG
    void feed(const uint8_t* d, size_t n);
    void reset();                               // lupakan state robot

    // perintah
    void ping();
    void cmd(uint8_t id);
    void setParam(uint8_t id, float v);
    void setTrim(uint8_t bus, uint8_t slot, int16_t v);
    void gaitIntent(float vx, float yawDeg, uint16_t ttlMs);
    void poseTarget(const int16_t j[SERVO_BUSES][SERVO_SLOTS], uint32_t mask, uint16_t ttlMs);
    bool motionUpload(uint8_t slot, const uint8_t* data, size_t n, bool save);
    void motionPlay(uint8_t slot, bool play);
    void simImu(float rollDeg, float pitchDeg);
    void simFault(uint8_t bus, uint8_t slot, bool dead);
    // berkas SD (Fase 4A). req = nomor permintaan, dipantulkan di balasan.
    void fileList(uint8_t req, uint16_t start, const char* path);
    void fileRead(uint8_t req, uint32_t offset, uint8_t len, const char* path);
    void fileWrite(uint8_t req, uint32_t offset, bool truncate, const char* path, const uint8_t* d, size_t n);
    void fileSimple(uint8_t type, uint8_t req, const char* path);          // FILE_DELETE / FILE_MKDIR / FILE_STAT
    void fileRename(uint8_t req, const char* from, const char* to);
    static constexpr size_t FILE_CHUNK = 160;
    // alat servo ICS (Fase 4A, mode 830). op = la::ToolOp.
    // Fase 4B: kirim status stik (studio/PC sebagai radio; robot asli memakai board radio)
    void sendPad(const PadState& p);
    void servoTool(uint8_t req, uint8_t op, uint8_t bus = 0, uint8_t id = 0, uint8_t sc = 0, uint8_t arg = 0);

    // state terakhir dari robot
    bool                haveHello = false, haveTele = false, haveServo = false, haveGait = false, haveConfig = false;
    lap::Hello          hello;
    lap::TelemetryMsg   tele;
    lap::ServoStateMsg  servo;
    lap::GaitStateMsg   gait;
    lap::ConfigMsg      config;
    lap::MotionStatusMsg motions;
    lap::Ack            lastAck;
    uint32_t            rxPackets = 0;

    // kin GAIT_STATE (urutan ServoPositions) -> [bus][slot]
    void gaitSrc(int16_t out[SERVO_BUSES][SERVO_SLOTS]) const;

private:
    template <typename M> void send(uint8_t type, const M& m) {
        lap::Packet p;
        if (lap::makePacket(p, type, seq_++, m)) sendPacket(p);
    }
    void sendPacket(const lap::Packet& p);
    void handle(const lap::Packet& p);
    lap::Decoder dec_;
    uint8_t seq_ = 0;
};

}  // namespace la::studio
