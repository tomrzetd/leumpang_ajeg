// =============================================================================
//  Leumpang Studio - robot simulasi di dalam proses (digital twin tanpa hardware).
//  Isinya sama dengan sim/leumpang_sim.cpp: leumpang-core Controller + LapEndpoint +
//  model servo + IMU sintetis; host berbicara LAP lewat fromHost()/callback toHost.
//  Tanpa thread: panggil advance() dari timer 10 ms.
// =============================================================================
#pragma once
#include "leumpang/controller.h"
#include "leumpang/ics_sim.h"
#include "leumpang/lap_endpoint.h"
#include "leumpang/files.h"

#include <deque>
#include <functional>

namespace la::studio {

class SimRobot {
public:
    using Sink = std::function<void(const uint8_t*, size_t)>;
    using LogSink = std::function<void(const char*)>;

    void start(Sink toHost, LogSink log = nullptr, bool randomStart = true, const char* sdRoot = nullptr);
    bool rebootRequested() const { return reboot_; }
    ~SimRobot() { delete files_; }
    void fromHost(const uint8_t* d, size_t n);
    void advance(int cycles = 1);              // satu siklus = CONTROL_PERIOD_MS
    int64_t nowUs() const { return now_; }
    void setImu(float rollDeg, float pitchDeg) { rollDeg_ = rollDeg; pitchDeg_ = pitchDeg; }
    const Controller& controller() const { return ctrl_; }
    // Fase 4A: uji alat servo - matikan/hidupkan servo tiruan per ID
    void setServoPresent(int bus, int id, bool on) { servo_.dead[bus][id] = !on; }
    bool servoPresent(int bus, int id) const { return !servo_.dead[bus][id]; }
    uint8_t servoParam(int bus, int id, int sc) const { return servo_.param[bus][id][sc]; }

private:
    struct Servo : IcsSimBus {};
    struct Box : Inbox {
        std::deque<char> keys; std::deque<Command> cmds;
        bool nextKey(char& c) override;
        bool nextCommand(Command& c) override;
    };
    static int64_t clock(void* ctx);
    static void    logLine(void* ctx, const char* line);

    Controller         ctrl_;
    lap::LapEndpoint   ep_;
    Servo              servo_{};
    Box                box_;
    Sink               sink_;
    LogSink            log_;
    int64_t            now_ = 1000000;
    float              rollDeg_ = 0, pitchDeg_ = 0;
    uint32_t           imuSeq_ = 0, noise_ = 12345;
    bool               started_ = false, reboot_ = false;
    FileOps*           files_ = nullptr;
};

}  // namespace la::studio
