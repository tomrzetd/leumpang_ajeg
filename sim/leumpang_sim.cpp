// =============================================================================
//  leumpang_sim - simulator host (Linux/macOS) untuk leumpang-core
//  * la::Controller asli, 100 Hz real-time (atau --fast)
//  * model servo ICS (orde-1 + batas kecepatan, reply posisi saat command diterima)
//  * IMU sintetis: roll/pitch diatur lewat LAP SIM_IMU (+ derau opsional)
//  * server LAP TCP (default :5760) - klien yang sama dengan robot sungguhan
//  Fase 2: model servo/IMU diganti fisika MuJoCo (digital twin).
// =============================================================================
#include "leumpang/controller.h"
#include "leumpang/lap_endpoint.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0          // macOS
#endif

namespace {

int64_t g_now = 0;                 // jam simulasi (us)
bool    g_fast = false;
bool    g_quiet = false;
bool    g_gaitEnable = false;
auto    g_t0 = std::chrono::steady_clock::now();
volatile sig_atomic_t g_stop = 0;

int64_t wallUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - g_t0).count();
}

// ---- model servo -------------------------------------------------------------
struct ServoModel : la::ServoBus {
    float pos[2][32]; int cmd[2][32]; bool freeT[2][32]; bool dead[2][32];
    float speed = 120.0f;          // hitungan per 10 ms (~ 0,07 s/60 deg)
    std::mt19937 rng{42};
    void init(bool randomStart) {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
            const la::JointDef& j = la::JOINTS[b][i < la::SERVO_SLOTS ? i : 0];
            pos[b][i] = randomStart ? (float)j.center + (float)((int)(rng() % 1200) - 600) : (float)j.center;
            cmd[b][i] = 0; freeT[b][i] = true; dead[b][i] = false;
        }
    }
    void exchange(la::IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) {
            const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
            x.got[b] = false;
            if (dead[b][id]) continue;
            x.got[b] = true;
            x.pos[b] = (int16_t)lroundf(pos[b][id]);
            if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; cmd[b][id] = p; }
        }
        g_now += 80;                                         // ~ waktu bus nyata per slot
    }
    void physics() {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < la::SERVO_SLOTS; ++i) {
            if (freeT[b][i] || dead[b][i]) continue;
            const float d = (float)cmd[b][i] - pos[b][i];
            pos[b][i] += d > speed ? speed : (d < -speed ? -speed : d);
        }
    }
} g_servo;

// ---- IMU sintetis ----------------------------------------------------------
struct {
    float rollDeg = 0, pitchDeg = 0, noiseDeg = 0.05f;
    bool ok = true;
    uint32_t seq = 0;
    std::mt19937 rng{7};
} g_imu;

la::ImuInput imuSample() {
    std::normal_distribution<float> n(0.0f, g_imu.noiseDeg);
    la::ImuInput in{};
    in.sensorOk = g_imu.ok; in.hasSample = g_imu.ok; in.rateHz = 500;
    in.roll = (g_imu.rollDeg + n(g_imu.rng)) / la::kRad2Deg;
    in.pitch = (g_imu.pitchDeg + n(g_imu.rng)) / la::kRad2Deg;
    in.tUs = g_now; in.seq = ++g_imu.seq;
    return in;
}

// ---- inbox (stdin + LAP) ---------------------------------------------------
struct SimInbox : la::Inbox {
    std::deque<char> keys; std::deque<la::Command> cmds;
    bool nextKey(char& c) override { if (keys.empty()) return false; c = keys.front(); keys.pop_front(); return true; }
    bool nextCommand(la::Command& c) override { if (cmds.empty()) return false; c = cmds.front(); cmds.pop_front(); return true; }
} g_inbox;

la::Controller       g_ctrl;
la::BlackboxRing     g_bb;
FILE*                g_bbFile = nullptr;
la::lap::LapEndpoint g_ep;
int                  g_client = -1;

void logLine(void*, const char* line) {
    if (!g_quiet) { fputs(line, stdout); fflush(stdout); }
    if (g_client >= 0) g_ep.sendLog(line);
}
void epWrite(void*, const uint8_t* d, size_t n) {
    if (g_client < 0) return;
    if (send(g_client, d, n, MSG_DONTWAIT | MSG_NOSIGNAL) < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(g_client); g_client = -1;
    }
}
bool epSubmit(void*, const la::Command& c) {
    if (g_inbox.cmds.size() >= 32) return false;
    g_inbox.cmds.push_back(c); return true;
}
void epSnapshot(void*, la::Telemetry& t) { g_ctrl.snapshot(t); }
bool epSim(void*, const la::lap::Packet& p) {
    if (p.type == la::lap::SIM_IMU) {
        la::lap::SimImuMsg m;
        if (!la::lap::parsePacket(p, m) || !la::isFiniteVal(m.rollDeg) || !la::isFiniteVal(m.pitchDeg)) return false;
        g_imu.rollDeg = m.rollDeg; g_imu.pitchDeg = m.pitchDeg; g_imu.ok = m.sensorOk != 0;
        return true;
    }
    if (p.type == la::lap::SIM_FAULT) {
        la::lap::SimFaultMsg m;
        if (!la::lap::parsePacket(p, m) || m.bus > 1 || m.slot >= la::SERVO_SLOTS) return false;
        g_servo.dead[m.bus][la::JOINTS[m.bus][m.slot].id] = m.dead != 0;
        return true;
    }
    return false;
}

int listenOn(uint16_t port) {
    const int s = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(port); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (getenv("LEUMPANG_SIM_BIND_ANY")) a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (sockaddr*)&a, sizeof(a)) != 0 || listen(s, 1) != 0) { perror("bind/listen"); exit(1); }
    fcntl(s, F_SETFL, O_NONBLOCK);
    return s;
}

void usage() {
    puts("leumpang_sim [--port N] [--fast] [--seconds S] [--dead B:S] [--home-start] [--quiet] [--no-stdin]\n"
         "             [--gait] [--blackbox file.lbb]\n"
         "  --gait       aktifkan gaitEnable (GAIT_INTENT diterima)\n"
         "  --blackbox F rekam blackbox 100 Hz ke F (format .lbb)\n"
         "  --fast       jalankan secepat mungkin (tanpa jeda real-time)\n"
         "  --seconds S  berhenti setelah S detik waktu simulasi\n"
         "  --dead B:S   servo bus B slot S mati (tidak menjawab)\n"
         "  --home-start servo mulai di posisi center (default: acak, menguji boot sync)\n"
         "Ketik perintah konsol robot (r g x c t y z ? ...) di stdin.");
}

}  // namespace

int main(int argc, char** argv) {
    uint16_t port = la::lap::TCP_PORT;
    double seconds = 0;
    bool useStdin = true, randomStart = true;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = (uint16_t)atoi(argv[++i]);
        else if (a == "--fast") g_fast = true;
        else if (a == "--seconds" && i + 1 < argc) seconds = atof(argv[++i]);
        else if (a == "--quiet") g_quiet = true;
        else if (a == "--no-stdin") useStdin = false;
        else if (a == "--home-start") randomStart = false;
        else if (a == "--blackbox" && i + 1 < argc) g_bbFile = fopen(argv[++i], "wb");
        else if (a == "--gait") g_gaitEnable = true;
        else if (a == "--dead" && i + 1 < argc) {
            int b = 0, s = 0;
            if (sscanf(argv[++i], "%d:%d", &b, &s) == 2 && b >= 0 && b < 2 && s >= 0 && s < la::SERVO_SLOTS)
                g_servo.dead[b][la::JOINTS[b][s].id] = true;
        } else { usage(); return a == "--help" || a == "-h" ? 0 : 2; }
    }
    signal(SIGINT, [](int) { g_stop = 1; });
    signal(SIGPIPE, SIG_IGN);

    bool deadSaved[2][32];
    memcpy(deadSaved, g_servo.dead, sizeof(deadSaved));
    g_servo.init(randomStart);
    memcpy(g_servo.dead, deadSaved, sizeof(deadSaved));

    la::Platform pf{ [](void*) -> int64_t { return g_now; }, logLine, nullptr };
    int16_t trim[2][la::SERVO_SLOTS] = {};
    la::RuntimeConfig cfg0 = la::runtimeConfigDefault();
    cfg0.gaitEnable = g_gaitEnable;
    g_ctrl.init(pf, cfg0, trim, la::Controller::Options{});
    if (g_bbFile) {
        uint8_t hdr[16]; la::blackboxHeader(hdr, (uint32_t)(g_now / 1000));
        fwrite(hdr, 1, 16, g_bbFile);
        g_ctrl.setBlackbox(&g_bb);
    }

    la::lap::Hello self;
    self.role = la::lap::ROLE_SIM; self.board = la::lap::BOARD_SIM;
    self.fwMajor = la::CORE_VER_MAJOR; self.fwMinor = la::CORE_VER_MINOR; self.fwPatch = la::CORE_VER_PATCH;
    self.caps = la::lap::CAP_SIM | la::lap::CAP_MOTION | la::lap::CAP_GAIT_INTENT | (g_bbFile ? (uint32_t)la::lap::CAP_SD : 0u);
    strncpy(self.name, "leumpang-sim", 16);
    la::lap::LapEndpoint::Host h;
    h.write = epWrite; h.submit = epSubmit; h.snapshot = epSnapshot; h.simHook = epSim;
    h.motions = &g_ctrl.motions();
    h.motionSaved = [](void*, int slot, const uint8_t*, size_t n) {
        printf("[sim] motion slot %d diunggah (%zu B)\n", slot, n); fflush(stdout); };

    const int lsock = listenOn(port);
    if (useStdin) fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    printf("leumpang_sim: core %u.%u.%u, LAP TCP 127.0.0.1:%u%s\n", la::CORE_VER_MAJOR, la::CORE_VER_MINOR,
           la::CORE_VER_PATCH, port, g_fast ? " (fast)" : "");
    fflush(stdout);

    g_now = 1000000;
    for (int i = 0; i < la::BOOT_SYNC_CYCLES; ++i) { g_ctrl.bootFreeCycle(g_servo); g_now += 10000; }
    g_ctrl.bootFinish(true);

    const int64_t simStart = g_now, wallStart = wallUs();
    uint8_t buf[512];
    int64_t cycleStart = g_now;
    while (!g_stop) {
        // ---- siklus kontrol 10 ms ----
        g_now = cycleStart;
        g_ctrl.step(imuSample(), g_servo, g_inbox);
        g_servo.physics();
        if (g_bbFile) {
            la::BlackboxRecord r; uint8_t rec[la::BLACKBOX_REC_SIZE];
            while (g_bb.pop(r)) { r.encode(rec); fwrite(rec, 1, sizeof(rec), g_bbFile); }
        }
        cycleStart += (int64_t)la::CONTROL_PERIOD_MS * 1000;
        if (seconds > 0 && (cycleStart - simStart) >= (int64_t)(seconds * 1e6)) break;

        // ---- I/O sampai siklus berikutnya ----
        const int64_t deadlineWall = wallStart + (cycleStart - simStart);
        do {
            const int c = accept(lsock, nullptr, nullptr);
            if (c >= 0) {
                if (g_client >= 0) close(g_client);
                g_client = c;
                int one = 1;
                fcntl(c, F_SETFL, O_NONBLOCK);
                setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                g_ep.init(h, self);
            }
            pollfd fds[2]; int nf = 0;
            if (g_client >= 0) fds[nf++] = {g_client, POLLIN, 0};
            if (useStdin) fds[nf++] = {0, POLLIN, 0};
            const int64_t waitUs = g_fast ? 0 : deadlineWall - wallUs();
            poll(fds, (nfds_t)nf, waitUs > 0 ? (int)((waitUs + 999) / 1000) : 0);
            if (g_client >= 0) {
                const ssize_t r = recv(g_client, buf, sizeof(buf), MSG_DONTWAIT);
                if (r > 0) g_ep.onBytes(buf, (size_t)r, (uint32_t)(g_now / 1000));
                else if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) { close(g_client); g_client = -1; }
            }
            if (useStdin) {
                const ssize_t r = read(0, buf, sizeof(buf));
                for (ssize_t i = 0; i < r; ++i) g_inbox.keys.push_back((char)buf[i]);
            }
            if (g_client >= 0) g_ep.poll((uint32_t)(cycleStart / 1000));
        } while (!g_fast && wallUs() < deadlineWall && !g_stop);
    }
    if (g_client >= 0) close(g_client);
    close(lsock);
    if (g_bbFile) fclose(g_bbFile);
    la::Telemetry t; g_ctrl.snapshot(t);
    printf("leumpang_sim selesai: mode=%s overruns=%u\n", la::modeName(t.mode), (unsigned)t.loopOverruns);
    return 0;
}
