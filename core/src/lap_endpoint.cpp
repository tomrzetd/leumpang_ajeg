#include "leumpang/lap_endpoint.h"

namespace la {
namespace lap {

void LapEndpoint::init(const Host& h, const Hello& self) {
    h_ = h; self_ = self;
    active_ = false; seq_ = 0;
    stream_ = StreamCfgMsg{};
}

void LapEndpoint::sendPacket(const Packet& p) {
    uint8_t f[MAX_FRAME];
    const size_t n = encodeFrame(p, f, sizeof(f));
    if (n && h_.write) h_.write(h_.ctx, f, n);
}

void LapEndpoint::ack(const Packet& p, uint8_t status) {
    Ack a; a.ackSeq = p.seq; a.ackType = p.type; a.status = status;
    send(ACK, a);
}

void LapEndpoint::sendConfig() {
    if (!h_.snapshot) return;
    h_.snapshot(h_.ctx, snap_);
    ConfigMsg c;
    fillConfig(snap_.cfg, snap_.cfgGen, h_.savedGen ? h_.savedGen(h_.ctx) : snap_.cfgGen, c);
    send(CONFIG, c);
    lastCfgGen_ = snap_.cfgGen;
}

void LapEndpoint::onBytes(const uint8_t* d, size_t n, uint32_t nowMs) {
    Packet p;
    for (size_t i = 0; i < n; ++i)
        if (dec_.feed(d[i], p)) handle(p, nowMs);
}

void LapEndpoint::handle(const Packet& p, uint32_t nowMs) {
    lastRxMs_ = nowMs;
    auto submit = [&](const Command& c) {
        ack(p, (h_.submit && h_.submit(h_.ctx, c)) ? OK : BUSY);
    };
    switch (p.type) {
        case HELLO: {
            Hello peer;
            if (!parsePacket(p, peer)) { ack(p, BAD_LEN); return; }
            active_ = true;
            send(HELLO, self_);
            if (peer.protoVer != VERSION) ack(p, UNSUPPORTED);
            sendConfig();
            return;
        }
        case PING: {
            active_ = true;
            Packet r = p; r.type = PONG; r.seq = seq_++;
            sendPacket(r);
            return;
        }
        case STREAM_CFG: {
            StreamCfgMsg s;
            if (!parsePacket(p, s)) { ack(p, BAD_LEN); return; }
            s.teleHz  = (uint8_t)clampi(s.teleHz, 0, 50);
            s.servoHz = (uint8_t)clampi(s.servoHz, 0, 20);
            s.gaitHz  = (uint8_t)clampi(s.gaitHz, 0, 50);
            stream_ = s; active_ = true;
            ack(p, OK);
            return;
        }
        case CMD: {
            CmdMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            Command c{};
            switch (m.cmd) {
                case CMD_RESET: c.type = CmdType::RESET; break;
                case CMD_GO:    c.type = CmdType::GO; break;
                case CMD_STOP:  c.type = CmdType::STOP; break;
                case CMD_CAL:   c.type = CmdType::CALIBRATE; break;
                case CMD_GET_CONFIG: ack(p, OK); sendConfig(); return;
                case CMD_HELLO: send(HELLO, self_); ack(p, OK); return;
                default: ack(p, BAD_VALUE); return;
            }
            submit(c);
            return;
        }
        case SET_PARAM: {
            SetParamMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            if (m.param >= (uint8_t)ParamId::COUNT_ || !isFiniteVal(m.value)) { ack(p, BAD_VALUE); return; }
            Command c{}; c.type = CmdType::SET_PARAM; c.param = (ParamId)m.param; c.value = m.value;
            submit(c);
            return;
        }
        case SET_TRIM: {
            SetTrimMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            if (m.bus > 1 || m.slot >= SERVO_SLOTS) { ack(p, BAD_VALUE); return; }
            Command c{}; c.type = CmdType::SET_TRIM; c.bus = m.bus; c.slot = m.slot; c.value = m.value;
            submit(c);
            return;
        }
        case SIM_IMU: case SIM_FAULT:
            ack(p, (h_.simHook && h_.simHook(h_.ctx, p)) ? OK : UNSUPPORTED);
            return;
        case PONG: case ACK:
            return;
        default:   // GAIT_INTENT, PERCEPTION, MOTION_*: Fase 1
            ack(p, UNSUPPORTED);
            return;
    }
}

void LapEndpoint::poll(uint32_t nowMs) {
    if (active_ && nowMs - lastRxMs_ > LINK_TIMEOUT_MS) active_ = false;
    if (!active_ || !h_.snapshot) return;
    const bool tele  = stream_.teleHz  && nowMs - lastTele_  >= 1000u / stream_.teleHz;
    const bool servo = stream_.servoHz && nowMs - lastServo_ >= 1000u / stream_.servoHz;
    const bool gait  = stream_.gaitHz  && nowMs - lastGait_  >= 1000u / stream_.gaitHz;
    if (!tele && !servo && !gait) return;
    h_.snapshot(h_.ctx, snap_);
    const uint32_t saved = h_.savedGen ? h_.savedGen(h_.ctx) : snap_.cfgGen;
    if (tele)  { TelemetryMsg m;  fillTelemetry(snap_, saved, m); send(TELEMETRY, m);  lastTele_ = nowMs; }
    if (servo) { ServoStateMsg m; fillServoState(snap_, m);       send(SERVO_STATE, m); lastServo_ = nowMs; }
    if (gait)  { GaitStateMsg m;  fillGaitState(snap_, m);        send(GAIT_STATE, m);  lastGait_ = nowMs; }
    if (snap_.cfgGen != lastCfgGen_) sendConfig();      // config berubah -> broadcast
}

void LapEndpoint::sendLog(const char* line) {
    if (!active_ || !stream_.logOn || !line) return;
    Packet p; p.type = LOG; p.seq = seq_++;
    size_t n = strlen(line);
    if (n > MAX_BODY) n = MAX_BODY;
    memcpy(p.body, line, n); p.len = (uint8_t)n;
    sendPacket(p);
}

}  // namespace lap
}  // namespace la
