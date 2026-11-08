// Tes skenario leumpang-core Controller + parser ICS
#include "leumpang/controller.h"
#include "check.h"

#include <cmath>
#include <cstdlib>
#include <deque>

using namespace la;

static int64_t g_now = 1000000;

struct Model : ServoBus {
    float pos[2][32]; int cmd[2][32]; bool freeT[2][32]; bool dead[2][32];
    void init() {
        srand(7);
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
            pos[b][i] = 7500.0f + (float)(rand() % 2000 - 1000); cmd[b][i] = 0; freeT[b][i] = true; dead[b][i] = false; }
    }
    void exchange(IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) {
            const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
            // lewat jalur kabel: echo + reply, lalu parser asli
            uint8_t rx[6] = { x.tx[b][0], x.tx[b][1], x.tx[b][2], 0, 0, 0 };
            const int cur = (int)lroundf(pos[b][id]);
            rx[3] = (uint8_t)(x.tx[b][0] & 0x7F); rx[4] = (uint8_t)((cur >> 7) & 0x7F); rx[5] = (uint8_t)(cur & 0x7F);
            x.got[b] = !dead[b][id] && ics::parseReply(rx, 6, x.tx[b], &x.pos[b]);
            if (!dead[b][id]) { if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; cmd[b][id] = p; } }
        }
    }
    void physics() {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 11; ++i) {
            if (freeT[b][i] || dead[b][i]) continue;
            const float d = cmd[b][i] - pos[b][i];
            pos[b][i] += d > 120 ? 120 : (d < -120 ? -120 : d);
        }
    }
} M;

struct Box : Inbox {
    std::deque<char> k; std::deque<Command> c;
    bool nextKey(char& v) override { if (k.empty()) return false; v = k.front(); k.pop_front(); return true; }
    bool nextCommand(Command& v) override { if (c.empty()) return false; v = c.front(); c.pop_front(); return true; }
} box;

static Controller ctrl;
static float g_roll = 0, g_pitch = 0; static uint32_t g_seq = 0;

static void step(int n) {
    for (int i = 0; i < n; ++i) {
        ImuInput in{true, true, g_roll, g_pitch, g_now, ++g_seq, 500};
        ctrl.step(in, M, box);
        M.physics();
        g_now += 10000;
    }
}
static Telemetry T() { Telemetry t; ctrl.snapshot(t); return t; }

static void testIcsParser() {
    const uint8_t sent[3] = {0x85, 0x3A, 0x4C};
    int16_t pos = 0;
    const uint8_t noEcho[3] = {0x05, 0x3A, 0x4C};
    CHECK(ics::parseReply(noEcho, 3, sent, &pos) && pos == ((0x3A << 7) | 0x4C), "reply tanpa echo");
    const uint8_t withEcho[6] = {0x85, 0x3A, 0x4C, 0x05, 0x10, 0x20};
    CHECK(ics::parseReply(withEcho, 6, sent, &pos) && pos == ((0x10 << 7) | 0x20), "reply setelah echo identik");
    const uint8_t partial[5] = {0x85, 0x3A, 0x4C, 0x05, 0x10};
    CHECK(!ics::parseReply(partial, 5, sent, &pos), "echo + reply belum lengkap -> belum valid (early-exit aman)");
    const uint8_t wrongId[3] = {0x06, 0x3A, 0x4C};
    CHECK(!ics::parseReply(wrongId, 3, sent, &pos), "ID salah ditolak");
    // kasus jebakan scanner V3: posH & 0x1F == id
    const uint8_t sent2[3] = {0x9A, 0x1A, 0x40};
    const uint8_t trap[4] = {0x9A, 0x1A, 0x40, 0x1A};
    CHECK(!ics::parseReply(trap, 4, sent2, &pos), "tidak salah-cocok pada byte echo (jebakan V3)");
}

int main() {
    testIcsParser();
    M.init(); M.dead[1][3] = true;
    int16_t trim[2][SERVO_SLOTS] = {};
    Platform pf{ [](void*) -> int64_t { return g_now; }, nullptr, nullptr };
    ctrl.init(pf, runtimeConfigDefault(), trim, Controller::Options{});
    for (int i = 0; i < BOOT_SYNC_CYCLES; ++i) { ctrl.bootFreeCycle(M); g_now += 10000; }
    ctrl.bootFinish(true);
    step(1);
    Telemetry t = T();
    int jump = 0;
    for (int b = 0; b < 2; ++b) for (int i = 0; i < 11; ++i) {
        if (b == 1 && i == 3) continue;
        const int d = abs(t.cmd[b][i] - (int)lroundf(M.pos[b][i])); if (d > jump) jump = d; }
    CHECK(jump <= SERVO_MAX_STEP + 130, "boot: berangkat dari posisi aktual, bukan lompat ke HOME");
    CHECK(t.fbValid[1][3] == 0 && t.fbValid[0][3] == 1, "servo mati terdeteksi");
    step(110); CHECK(T().mode == Mode::READY, "RESET -> READY");
    step(110); CHECK(T().calDone, "kalibrasi IMU selesai");
    Command c{}; c.type = CmdType::SET_TRIM; c.bus = 0; c.slot = 8; c.value = 120; box.c.push_back(c);
    const uint32_t g0 = T().cfgGen; step(2);
    CHECK(T().trim[0][8] == 120 && T().cfgGen == g0 + 1, "trim diterapkan + cfgGen naik");
    box.k.push_back('g'); step(2); CHECK(T().mode == Mode::BALANCE, "'g' -> BALANCE");
    c = Command{}; c.type = CmdType::SET_PARAM; c.param = ParamId::SIGN_ROLL; c.value = -1; box.c.push_back(c);
    step(2); CHECK(T().cfg.imuRollSign == 1, "ganti frame IMU saat BALANCE ditolak");
    bool init = false, stp = false, end = false, fin = true; int16_t mn = 32767, mx = -32768;
    g_roll = 0.06f;
    for (int i = 0; i < 400; ++i) {
        step(1); if (i == 8) g_roll = 0; t = T();
        init |= t.mode == Mode::WALK_INIT; stp |= t.mode == Mode::WALK_STEP; end |= t.mode == Mode::WALK_END;
        fin &= isFiniteVal(t.dxi) && isFiniteVal(t.autoH);
        for (int b = 0; b < 2; ++b) for (int s = 0; s < 11; ++s) { if (t.cmd[b][s] < mn) mn = t.cmd[b][s]; if (t.cmd[b][s] > mx) mx = t.cmd[b][s]; }
    }
    CHECK(init && stp && end && T().mode == Mode::BALANCE, "siklus langkah 740->760->770->730");
    CHECK(fin && mn >= ICS_POS_MIN && mx <= ICS_POS_MAX, "state finite & command dalam batas");
    g_pitch = 0.5f; step(6); CHECK(T().mode == Mode::FALLEN, "jatuh terdeteksi -> FALLEN");
    box.k.push_back('r'); step(2); CHECK(T().mode == Mode::FALLEN, "'r' ditolak saat miring");
    g_pitch = 0; step(20); box.k.push_back('r'); step(1); CHECK(T().mode == Mode::RESET, "'r' saat tegak -> RESET");
    step(100); CHECK(T().mode == Mode::READY, "kembali READY");
    // IMU basi saat berjalan -> reset
    box.k.push_back('g'); step(2);
    for (int i = 0; i < 10; ++i) { ImuInput in{true, true, 0, 0, g_now - 200000, g_seq, 500}; ctrl.step(in, M, box); g_now += 10000; }
    CHECK(T().mode == Mode::RESET, "IMU basi saat BALANCE -> RESET");
    return checkSummary("test_controller");
}
