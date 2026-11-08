#include "lap_client.h"

#include <cstring>

namespace la::studio {
using namespace la::lap;

void LapClient::sendPacket(const Packet& p) {
    uint8_t buf[MAX_FRAME + 8];
    const size_t n = encodeFrame(p, buf, sizeof(buf));
    if (n && write) write(buf, n);
}

void LapClient::reset() {
    haveHello = haveTele = haveServo = haveGait = haveConfig = false;
    rxPackets = 0;
    dec_ = Decoder();
}

void LapClient::begin(const Rates& r) {
    const uint8_t z = 0;
    if (write) write(&z, 1);
    Hello me; me.role = ROLE_DESKTOP; me.board = BOARD_UNKNOWN;
    me.fwMajor = CORE_VER_MAJOR; me.fwMinor = CORE_VER_MINOR; me.fwPatch = CORE_VER_PATCH;
    strncpy(me.name, "leumpang-studio", 16);
    send(HELLO, me);
    StreamCfgMsg s; s.teleHz = r.teleHz; s.servoHz = r.servoHz; s.gaitHz = r.gaitHz; s.logOn = r.logOn;
    send(STREAM_CFG, s);
    cmd(CMD_MOTION_LIST);
}

void LapClient::feed(const uint8_t* d, size_t n) {
    Packet p;
    for (size_t i = 0; i < n; ++i)
        if (dec_.feed(d[i], p)) handle(p);
}

void LapClient::handle(const Packet& p) {
    ++rxPackets;
    switch (p.type) {
        case HELLO:         haveHello  = parsePacket(p, hello); break;
        case TELEMETRY:     haveTele   = parsePacket(p, tele); break;
        case SERVO_STATE:   haveServo  = parsePacket(p, servo); break;
        case GAIT_STATE:    haveGait   = parsePacket(p, gait); break;
        case CONFIG:        haveConfig = parsePacket(p, config); break;
        case MOTION_STATUS: parsePacket(p, motions); break;
        case ACK:           parsePacket(p, lastAck); break;
        case LOG:
            if (onLog) onLog(std::string((const char*)p.body, p.len));
            break;
        default: break;
    }
    if (onPacket) onPacket(p.type);
}

void LapClient::ping() { Packet p; p.type = PING; p.seq = seq_++; p.len = 0; sendPacket(p); }
void LapClient::cmd(uint8_t id) { CmdMsg m; m.cmd = id; send(CMD, m); }
void LapClient::setParam(uint8_t id, float v) { SetParamMsg m; m.param = id; m.value = v; send(SET_PARAM, m); }
void LapClient::setTrim(uint8_t b, uint8_t s, int16_t v) { SetTrimMsg m; m.bus = b; m.slot = s; m.value = v; send(SET_TRIM, m); }
void LapClient::gaitIntent(float vx, float yawDeg, uint16_t ttl) {
    GaitIntentMsg m; m.vx = vx; m.yawDeg = yawDeg; m.ttlMs = ttl; send(GAIT_INTENT, m);
}
void LapClient::poseTarget(const int16_t j[SERVO_BUSES][SERVO_SLOTS], uint32_t mask, uint16_t ttl) {
    PoseTargetMsg m; m.mask = mask; m.ttlMs = ttl; memcpy(m.joint, j, sizeof(m.joint)); send(POSE_TARGET, m);
}
bool LapClient::motionUpload(uint8_t slot, const uint8_t* data, size_t n, bool save) {
    if (!data || n == 0 || n > 0xFFFF) return false;
    MotionBeginMsg b; b.slot = slot; b.total = (uint16_t)n; send(MOTION_BEGIN, b);
    constexpr size_t CHUNK = MAX_BODY - 3;
    for (size_t off = 0; off < n; off += CHUNK) {
        Packet p; p.type = MOTION_CHUNK; p.seq = seq_++;
        Writer w(p.body, MAX_BODY);
        const size_t k = n - off < CHUNK ? n - off : CHUNK;
        w.u8(slot); w.u16((uint16_t)off); w.bytes(data + off, k);
        p.len = (uint8_t)w.size();
        sendPacket(p);
    }
    MotionEndMsg e; e.slot = slot; e.save = save ? 1 : 0; send(MOTION_END, e);
    return true;
}
void LapClient::motionPlay(uint8_t slot, bool play) { MotionPlayMsg m; m.slot = slot; m.play = play ? 1 : 0; send(MOTION_PLAY, m); }
void LapClient::simImu(float r, float pt) { SimImuMsg m; m.rollDeg = r; m.pitchDeg = pt; m.sensorOk = 1; send(SIM_IMU, m); }
void LapClient::simFault(uint8_t b, uint8_t s, bool dead) { SimFaultMsg m; m.bus = b; m.slot = s; m.dead = dead; send(SIM_FAULT, m); }

void LapClient::gaitSrc(int16_t o[SERVO_BUSES][SERVO_SLOTS]) const {
    // kin: K0[2],K1[2],K2[2],H[2],A0[2],A1[2],U0[2],U1[2],U2[2],E[2],head,waist
    const int16_t* k = gait.kin;
    for (int b = 0; b < 2; ++b) {
        o[b][7] = k[0 + b]; o[b][6] = k[2 + b]; o[b][5] = k[4 + b]; o[b][8] = k[6 + b];
        o[b][9] = k[8 + b]; o[b][10] = k[10 + b]; o[b][1] = k[12 + b]; o[b][2] = k[14 + b];
        o[b][3] = k[16 + b]; o[b][4] = k[18 + b];
    }
    o[1][0] = k[20]; o[0][0] = k[21];
}

}  // namespace la::studio
