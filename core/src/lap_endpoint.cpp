#include "leumpang/lap_endpoint.h"

#include <stdio.h>
#include <string.h>

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
    if (n && h_.write) h_.write(h_.writeCtx ? h_.writeCtx : h_.ctx, f, n);
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

void LapEndpoint::sendMotionStatus() {
    if (!h_.motions) return;
    MotionStatusMsg m;
    for (int i = 0; i < MOTION_SLOTS && i < 4; ++i) {
        m.state[i] = (uint8_t)h_.motions->state(i);
        if (m.state[i] == MotionStore::READY || m.state[i] == MotionStore::PLAYING) {
            const Motion* mo = h_.motions->writable(i);          // baca header saja (stabil saat READY/PLAYING)
            m.nFrames[i] = mo->nFrames;
            memcpy(m.name[i], mo->name, 16);
        }
    }
    send(MOTION_STATUS, m);
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
                case CMD_MOTION_LIST: ack(p, h_.motions ? OK : UNSUPPORTED); sendMotionStatus(); return;
                case CMD_REBOOT: {
                    // hanya saat robot diam: servo bebas di FALLEN / RAW / READY
                    Telemetry t{}; if (h_.snapshot) h_.snapshot(h_.ctx, t);
                    const bool still = t.mode == Mode::READY || t.mode == Mode::FALLEN || t.mode == Mode::RAW_ANGLES ||
                                       t.mode == Mode::TOOL;
                    if (!h_.reboot) { ack(p, UNSUPPORTED); return; }
                    if (!still) { ack(p, REJECTED_STATE); return; }
                    ack(p, OK);
                    sendLog("reboot diminta host (profil/firmware baru)\r\n");
                    rebootAtMs_ = (nowMs + 300u) | 1u;                // beri waktu ACK/LOG terkirim
                    return;
                }
                default: ack(p, BAD_VALUE); return;
            }
            submit(c);
            return;
        }
        case SERVO_TOOL: {
            // body: req, op, bus, id, sc, arg -> SERVO_TOOL_RESULT (bukan ACK)
            Reader r(p.body, p.len);
            Command c{}; c.type = CmdType::SERVO_TOOL;
            c.toolReq = r.u8(); c.toolOp = r.u8(); c.bus = r.u8(); c.toolId = r.u8(); c.toolSc = r.u8(); c.toolArg = r.u8();
            if (!r.ok()) { ack(p, BAD_LEN); return; }
            if (!h_.snapshot || !h_.submit) { toolReply(c.toolReq, c.toolOp, TOOL_UNSUPPORTED, c.bus, c.toolId, 0, nullptr, 0); return; }
            if (toolWait_) { toolReply(c.toolReq, c.toolOp, TOOL_BUSY, c.bus, c.toolId, 0, nullptr, 0); return; }
            h_.snapshot(h_.ctx, snap_);
            if (!h_.submit(h_.ctx, c)) { toolReply(c.toolReq, c.toolOp, TOOL_BUSY, c.bus, c.toolId, 0, nullptr, 0); return; }
            toolWait_ = true; toolReq_ = c.toolReq; toolSeq0_ = snap_.toolSeq; toolSinceMs_ = nowMs;
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
        case GAIT_INTENT: {
            GaitIntentMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            if (!isFiniteVal(m.vx) || !isFiniteVal(m.yawDeg)) { ack(p, BAD_VALUE); return; }
            if (h_.snapshot) {
                h_.snapshot(h_.ctx, snap_);
                if (!snap_.cfg.gaitEnable || (snap_.mode != Mode::BALANCE && snap_.mode != Mode::WALK_CONT)) {
                    ack(p, REJECTED_STATE); return;
                }
            }
            Command c{}; c.type = CmdType::GAIT_INTENT; c.value = m.vx; c.vy = m.vy; c.yawDeg = m.yawDeg;
            c.ttlMs = (uint16_t)clampi(m.ttlMs, 0, 1000);
            submit(c);
            return;
        }
        case MOTION_BEGIN: {
            MotionBeginMsg m;
            if (!h_.motions) { ack(p, UNSUPPORTED); return; }
            if (!parsePacket(p, m) ) { ack(p, BAD_LEN); return; }
            if (!MotionStore::validSlot(m.slot) || m.total < MOTION_HDR_BYTES + 2 || m.total > MOTION_MAX_BYTES) { ack(p, BAD_VALUE); return; }
            if (upSlot_ >= 0 && upSlot_ != m.slot) h_.motions->commit(upSlot_, false);   // batalkan unggahan lama
            if (!(upSlot_ == m.slot) && !h_.motions->beginWrite(m.slot)) { ack(p, REJECTED_STATE); upSlot_ = -1; return; }
            upSlot_ = m.slot; upTotal_ = m.total; upGot_ = 0;
            ack(p, OK);
            return;
        }
        case MOTION_CHUNK: {
            if (!h_.motions) { ack(p, UNSUPPORTED); return; }
            Reader r(p.body, p.len);
            const uint8_t slot = r.u8(); const uint16_t off = r.u16();
            const size_t n = r.remaining();
            if (!r.ok() || (int)slot != upSlot_ || off != upGot_ || upGot_ + n > upTotal_) { ack(p, BAD_VALUE); return; }
            memcpy(upBuf_ + upGot_, p.body + 3, n);
            upGot_ += n;
            ack(p, OK);
            return;
        }
        case MOTION_END: {
            MotionEndMsg m;
            if (!h_.motions) { ack(p, UNSUPPORTED); return; }
            if (!parsePacket(p, m) || (int)m.slot != upSlot_ || upGot_ != upTotal_) { ack(p, BAD_VALUE); return; }
            const MotionError e = motionDecode(upBuf_, upGot_, *h_.motions->writable(upSlot_));
            h_.motions->commit(upSlot_, e == MotionError::OK);
            if (e == MotionError::OK && m.save && h_.motionSaved) h_.motionSaved(h_.ctx, upSlot_, upBuf_, upGot_);
            if (e != MotionError::OK) {
                char line[64]; snprintf(line, sizeof(line), "Motion ditolak: %s\r\n", motionErrorName(e));
                sendLog(line);
            }
            upSlot_ = -1;
            ack(p, e == MotionError::OK ? OK : BAD_VALUE);
            sendMotionStatus();
            return;
        }
        case MOTION_PLAY: {
            MotionPlayMsg m;
            if (!h_.motions) { ack(p, UNSUPPORTED); return; }
            if (!parsePacket(p, m) || !MotionStore::validSlot(m.slot)) { ack(p, BAD_VALUE); return; }
            Command c{}; c.type = m.play ? CmdType::MOTION_PLAY : CmdType::MOTION_STOP; c.slot = m.slot;
            submit(c);
            return;
        }
        case POSE_TARGET: {
            PoseTargetMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            Command c{}; c.type = CmdType::POSE_TARGET; c.ttlMs = m.ttlMs; c.poseMask = m.mask;
            memcpy(c.pose, m.joint, sizeof(c.pose));
            submit(c);
            return;
        }
        case SIM_IMU: case SIM_FAULT:
            ack(p, (h_.simHook && h_.simHook(h_.ctx, p)) ? OK : UNSUPPORTED);
            return;
        case PONG: case ACK:
            return;
        case PERCEPTION: {
            PerceptionMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            Command c{}; c.type = CmdType::PERCEPTION; c.ttlMs = m.ttlMs; c.value = (float)m.frontMin();
            submit(c);
            return;
        }
        case FILE_LIST: case FILE_READ: case FILE_WRITE: case FILE_DELETE:
        case FILE_MKDIR: case FILE_RENAME: case FILE_STAT:
            if (!h_.files) { ack(p, UNSUPPORTED); return; }
            handleFile(p);
            return;
        case EXT: {
            ExtMsg m;
            if (!parsePacket(p, m)) { ack(p, BAD_LEN); return; }
            if (!h_.extHook) { ack(p, UNSUPPORTED); return; }
            ack(p, h_.extHook(h_.ctx, m.topic, m.data, m.len) ? OK : BAD_VALUE);
            return;
        }
        default:
            ack(p, UNSUPPORTED);
            return;
    }
}

void LapEndpoint::poll(uint32_t nowMs) {
    if (rebootAtMs_ && (int32_t)(nowMs - rebootAtMs_) >= 0) { rebootAtMs_ = 0; if (h_.reboot) h_.reboot(h_.ctx); }
    if (active_ && nowMs - lastRxMs_ > LINK_TIMEOUT_MS) active_ = false;
    if (!active_ || !h_.snapshot) return;
    if (toolWait_) {
        h_.snapshot(h_.ctx, snap_);
        if (snap_.toolSeq != toolSeq0_ && snap_.toolReq == toolReq_) {
            toolWait_ = false;
            toolReply(snap_.toolReq, snap_.toolOp, snap_.toolStatus, snap_.toolBus, snap_.toolId, snap_.toolValue,
                      snap_.toolData, snap_.toolLen);
        } else if (nowMs - toolSinceMs_ > 2000) {              // SCAN ~0,35 s; lebih dari 2 s = hilang
            toolWait_ = false;
            toolReply(toolReq_, 0, TOOL_NO_REPLY, 0, 0, 0, nullptr, 0);
        }
    }
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

void LapEndpoint::toolReply(uint8_t req, uint8_t op, uint8_t status, uint8_t bus, uint8_t id, int32_t value,
                            const uint8_t* d, uint8_t len) {
    // body: req, op, status, bus, id, i32 value, u8 len, data[len]
    Packet o; o.type = SERVO_TOOL_RESULT; o.seq = seq_++;
    Writer w(o.body, MAX_BODY);
    w.u8(req); w.u8(op); w.u8(status); w.u8(bus); w.u8(id); w.u32((uint32_t)value);
    if (len > 64) len = 64;
    w.u8(len);
    for (uint8_t i = 0; i < len; ++i) w.u8(d[i]);
    o.len = (uint8_t)w.size();
    sendPacket(o);
}

bool LapEndpoint::sendExt(uint16_t topic, const uint8_t* d, size_t n) {
    if (!active_ || n > ExtMsg::MAX_DATA) return false;
    ExtMsg m; m.topic = topic; m.len = (uint8_t)n;
    if (n) memcpy(m.data, d, n);
    send(EXT, m);
    return true;
}

void LapEndpoint::handleFile(const Packet& p) {
    Reader r(p.body, p.len);
    const uint8_t req = r.u8();
    char path[100], path2[100];
    Packet o; o.seq = seq_++;
    Writer w(o.body, MAX_BODY);
    auto rest = [&](char* out) {                                   // sisa body = jalur
        const size_t n = r.remaining();
        char raw[100]; size_t k = 0;
        while (r.remaining() && k < sizeof(raw) - 1) raw[k++] = (char)r.u8();
        raw[k] = 0;
        return n < sizeof(raw) && sanitizePath(raw, k, out, 100);
    };
    auto result = [&](uint8_t st, uint32_t v, uint8_t extra = 0) {
        o.type = FILE_RESULT; w.u8(req); w.u8(st); w.u32(v); w.u8(extra);
        o.len = (uint8_t)w.size(); sendPacket(o);
    };
    switch (p.type) {
        case FILE_LIST: {
            const uint16_t start = r.u16();
            if (!rest(path)) { result(FS_BAD_PATH, 0); return; }
            FileEntry e[4]; bool more = false;
            const int n = h_.files->list(path, start, e, 4, more);
            o.type = FILE_ENTRIES;
            w.u8(req); w.u8(n < 0 ? (uint8_t)-n : (uint8_t)FS_OK); w.u16(start); w.u8(n < 0 ? 0 : (uint8_t)n); w.u8(more ? 1 : 0);
            for (int i = 0; i < n; ++i) {
                const size_t L = strlen(e[i].name);
                w.u8(e[i].dir ? 1 : 0); w.u32(e[i].size); w.u8((uint8_t)L); w.bytes(e[i].name, L);
            }
            o.len = (uint8_t)w.size(); sendPacket(o);
            return;
        }
        case FILE_READ: {
            const uint32_t off = r.u32(); uint8_t len = r.u8();
            if (len > 200) len = 200;
            if (!rest(path)) { result(FS_BAD_PATH, 0); return; }
            uint8_t buf[200];
            const int n = h_.files->read(path, off, buf, len);
            o.type = FILE_DATA;
            w.u8(req); w.u8(n < 0 ? (uint8_t)-n : (uint8_t)FS_OK); w.u32(off);
            if (n > 0) w.bytes(buf, (size_t)n);
            o.len = (uint8_t)w.size(); sendPacket(o);
            return;
        }
        case FILE_WRITE: {
            const uint32_t off = r.u32(); const uint8_t flags = r.u8(), pl = r.u8();
            char raw[100]; size_t k = 0;
            while (k < pl && k < sizeof(raw) - 1) raw[k++] = (char)r.u8();
            raw[k] = 0;
            if (k != pl || !r.ok() || !sanitizePath(raw, k, path, sizeof path) || !path[0]) { result(FS_BAD_PATH, 0); return; }
            uint8_t data[MAX_BODY]; size_t n = 0;
            while (r.remaining()) data[n++] = r.u8();
            const FileStatus st = h_.files->write(path, off, data, n, (flags & 1) != 0);
            result(st, off + (uint32_t)n);
            return;
        }
        case FILE_DELETE: if (!rest(path)) { result(FS_BAD_PATH, 0); return; } result(h_.files->remove(path), 0); return;
        case FILE_MKDIR:  if (!rest(path)) { result(FS_BAD_PATH, 0); return; } result(h_.files->mkdir(path), 0); return;
        case FILE_RENAME: {
            const uint8_t la = r.u8();
            char raw[100]; size_t k = 0;
            while (k < la && k < sizeof(raw) - 1) raw[k++] = (char)r.u8();
            raw[k] = 0;
            if (!sanitizePath(raw, k, path, sizeof path) || !path[0] || !rest(path2) || !path2[0]) { result(FS_BAD_PATH, 0); return; }
            result(h_.files->rename(path, path2), 0);
            return;
        }
        case FILE_STAT: {
            if (!rest(path)) { result(FS_BAD_PATH, 0); return; }
            uint32_t size = 0; bool dir = false;
            const FileStatus st = h_.files->stat(path, size, dir);
            result(st, size, dir ? 1 : 0);
            return;
        }
        default: return;
    }
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
