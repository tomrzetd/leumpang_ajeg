#include "sim_robot.h"

#include <cmath>
#include <cstring>

namespace la::studio {

void SimRobot::Servo::exchange(IcsExchange& x) {
    for (int b = 0; b < 2; ++b) {
        const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
        x.got[b] = !dead[b][id];
        if (!x.got[b]) continue;
        x.pos[b] = (int16_t)std::lround(pos[b][id]);
        if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; cmd[b][id] = p; }
    }
}
void SimRobot::Servo::physics() {
    const float speed = 120.0f;                                // hitungan / 10 ms
    for (int b = 0; b < 2; ++b) for (int i = 0; i < SERVO_SLOTS; ++i) {
        if (freeT[b][i] || dead[b][i]) continue;
        const float d = (float)cmd[b][i] - pos[b][i];
        pos[b][i] += d > speed ? speed : (d < -speed ? -speed : d);
    }
}
bool SimRobot::Box::nextKey(char& c) { if (keys.empty()) return false; c = keys.front(); keys.pop_front(); return true; }
bool SimRobot::Box::nextCommand(Command& c) { if (cmds.empty()) return false; c = cmds.front(); cmds.pop_front(); return true; }

int64_t SimRobot::clock(void* ctx) { return static_cast<SimRobot*>(ctx)->now_; }
void SimRobot::logLine(void* ctx, const char* line) {
    SimRobot* s = static_cast<SimRobot*>(ctx);
    if (s->log_) s->log_(line);
    s->ep_.sendLog(line);
}

void SimRobot::start(Sink toHost, LogSink log, bool randomStart) {
    sink_ = std::move(toHost);
    log_ = std::move(log);
    uint32_t r = 99;
    for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
        const JointDef& j = JOINTS[b][i < SERVO_SLOTS ? i : 0];
        r = r * 1103515245u + 12345u;
        servo_.pos[b][i] = (float)j.center + (randomStart ? (float)((int)((r >> 8) % 1200) - 600) : 0.0f);
        servo_.cmd[b][i] = 0; servo_.freeT[b][i] = true; servo_.dead[b][i] = false;
    }
    Platform pf{clock, logLine, this};
    const int16_t trim[2][SERVO_SLOTS] = {};
    RuntimeConfig cfg = runtimeConfigDefault();
    ctrl_.init(pf, cfg, trim, Controller::Options{});

    lap::Hello self;
    self.role = lap::ROLE_SIM; self.board = lap::BOARD_SIM;
    self.fwMajor = CORE_VER_MAJOR; self.fwMinor = CORE_VER_MINOR; self.fwPatch = CORE_VER_PATCH;
    self.caps = lap::CAP_SIM | lap::CAP_MOTION | lap::CAP_GAIT_INTENT | lap::CAP_LIVE_POSE | lap::CAP_PERCEPTION;
    strncpy(self.name, "studio-sim", 16);
    lap::LapEndpoint::Host h;
    h.ctx = this;
    h.write = [](void* c, const uint8_t* d, size_t n) { auto* s = static_cast<SimRobot*>(c); if (s->sink_) s->sink_(d, n); };
    h.submit = [](void* c, const Command& cmd) {
        auto* s = static_cast<SimRobot*>(c);
        if (s->box_.cmds.size() >= 32) return false;
        s->box_.cmds.push_back(cmd); return true;
    };
    h.snapshot = [](void* c, Telemetry& t) { static_cast<SimRobot*>(c)->ctrl_.snapshot(t); };
    h.simHook = [](void* c, const lap::Packet& p) {
        auto* s = static_cast<SimRobot*>(c);
        if (p.type == lap::SIM_IMU) {
            lap::SimImuMsg m;
            if (!lap::parsePacket(p, m) || !isFiniteVal(m.rollDeg) || !isFiniteVal(m.pitchDeg)) return false;
            s->rollDeg_ = m.rollDeg; s->pitchDeg_ = m.pitchDeg; return true;
        }
        if (p.type == lap::SIM_FAULT) {
            lap::SimFaultMsg m;
            if (!lap::parsePacket(p, m) || m.bus > 1 || m.slot >= SERVO_SLOTS) return false;
            s->servo_.dead[m.bus][JOINTS[m.bus][m.slot].id] = m.dead != 0; return true;
        }
        return false;
    };
    h.motions = &ctrl_.motions();
    ep_.init(h, self);

    for (int i = 0; i < BOOT_SYNC_CYCLES; ++i) { ctrl_.bootFreeCycle(servo_); now_ += 10000; }
    ctrl_.bootFinish(true);
    started_ = true;
}

void SimRobot::fromHost(const uint8_t* d, size_t n) {
    if (started_) ep_.onBytes(d, n, (uint32_t)(now_ / 1000));
}

void SimRobot::advance(int cycles) {
    if (!started_) return;
    for (int i = 0; i < cycles; ++i) {
        noise_ = noise_ * 1664525u + 1013904223u;
        const float n1 = ((float)((noise_ >> 9) & 1023) / 1023.0f - 0.5f) * 0.08f;
        ImuInput in{};
        in.sensorOk = true; in.hasSample = true; in.rateHz = 500;
        in.roll = (rollDeg_ + n1) / kRad2Deg; in.pitch = (pitchDeg_ - n1) / kRad2Deg;
        in.tUs = now_; in.seq = ++imuSeq_;
        ctrl_.step(in, servo_, box_);
        servo_.physics();
        now_ += (int64_t)CONTROL_PERIOD_MS * 1000;
        ep_.poll((uint32_t)(now_ / 1000));
    }
}

}  // namespace la::studio
