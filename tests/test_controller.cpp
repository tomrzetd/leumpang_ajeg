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
    step(110);

    // ================= Fase 1: blackbox =================
    BlackboxRing ring;
    ctrl.setBlackbox(&ring);
    step(50);
    BlackboxRecord rec; int nrec = 0; uint32_t lastMs = 0; bool mono = true; uint8_t enc[64];
    while (ring.pop(rec)) { mono &= rec.ms >= lastMs; lastMs = rec.ms; rec.encode(enc); ++nrec; }
    CHECK(nrec == 50 && mono, "blackbox: 1 record/siklus, waktu monoton");
    step(400);                                         // tidak dikuras -> ring penuh
    CHECK(ring.dropped() == 400 - 256 && T().blackboxDropped == 144, "blackbox penuh: record dibuang & dihitung, kontrol tetap jalan");
    ctrl.setBlackbox(nullptr);

    // ================= Fase 1: motion player =================
    Motion mo{}; strcpy(mo.name, "uji"); mo.nFrames = 2; mo.flags = 0;
    for (int f = 0; f < 2; ++f) {
        mo.f[f].durMs = 200; mo.f[f].interp = MI_COSINE;
        for (int b = 0; b < 2; ++b) for (int s = 0; s < 11; ++s) mo.f[f].joint[b][s] = 0;
        mo.f[f].joint[0][1] = -2700; mo.f[f].joint[1][1] = -2760;   // bahu pitch seperti HOME
        mo.f[f].joint[0][7] = 661; mo.f[f].joint[1][7] = 661; mo.f[f].joint[0][8] = 1322; mo.f[f].joint[1][8] = 1322;
        mo.f[f].joint[0][9] = 661; mo.f[f].joint[1][9] = 661;
    }
    mo.f[0].joint[0][4] = -800;                         // siku kanan
    uint8_t blob[MOTION_MAX_BYTES];
    const size_t blen = motionEncode(mo, blob);
    Motion back{};
    CHECK(motionDecode(blob, blen, back) == MotionError::OK && back.f[0].joint[0][4] == -800 && !strcmp(back.name, "uji"),
          "LAM1 encode/decode roundtrip");
    blob[30] ^= 1; CHECK(motionDecode(blob, blen, back) == MotionError::BAD_CRC, "LAM1 CRC rusak ditolak"); blob[30] ^= 1;
    CHECK(ctrl.motions().load(0, blob, blen), "motion dimuat ke slot 0");
    c = Command{}; c.type = CmdType::MOTION_PLAY; c.slot = 0; box.c.push_back(c);
    step(1); CHECK(T().mode == Mode::MOTION_PLAY && ctrl.motions().state(0) == MotionStore::PLAYING, "MOTION_PLAY dari READY");
    step(10); const int16_t mid = T().kin.E[0];
    step(10); const int16_t end0 = T().kin.E[0];
    CHECK(mid < 0 && mid > -800 && end0 == -800, "interpolasi cosine siku 0 -> -800 dalam 200 ms");
    step(25);
    CHECK(T().mode == Mode::READY && ctrl.motions().state(0) == MotionStore::READY && T().kin.E[0] == 0,
          "motion selesai -> READY, slot dilepas");
    // di luar batas sendi
    mo.f[1].joint[0][4] = -3000;                        // siku R: 4800 - 3000 = 1800 < 3500
    const size_t blen2 = motionEncode(mo, blob);
    ctrl.motions().load(1, blob, blen2);
    c.slot = 1; box.c.push_back(c); step(2);
    CHECK(T().mode == Mode::READY && ctrl.motions().state(1) == MotionStore::READY, "motion di luar batas sendi ditolak");
    c.slot = 3; box.c.push_back(c); step(1);
    CHECK(T().mode == Mode::READY, "slot kosong ditolak");
    c.slot = 0; box.c.push_back(c); step(3);
    Command stop{}; stop.type = CmdType::MOTION_STOP; box.c.push_back(stop); step(1);
    CHECK(T().mode == Mode::READY && ctrl.motions().state(0) == MotionStore::READY, "MOTION_STOP menghentikan & melepas slot");

    // ================= Fase 1: gait kontinu =================
    box.k.push_back('g'); step(2);
    Command gi{}; gi.type = CmdType::GAIT_INTENT; gi.value = 8; gi.yawDeg = 2; gi.ttlMs = 500;
    box.c.push_back(gi); step(2);
    CHECK(T().mode == Mode::BALANCE, "GAIT_INTENT ditolak bila gaitEnable=0");
    Command pe{}; pe.type = CmdType::SET_PARAM; pe.param = ParamId::GAIT_ENABLE; pe.value = 1; box.c.push_back(pe);
    step(1);
    box.c.push_back(gi); step(1);
    CHECK(T().mode == Mode::WALK_CONT && T().intentActive, "GAIT_INTENT (enable) -> WALK_CONT");
    bool fin2 = true; int16_t lo2 = 32767, hi2 = -32768; float maxStride = 0; int swaps = 0; int8_t lastJ = T().jikuasi;
    int16_t k2min = 32767, k2max = -32768;
    for (int i = 0; i < 300; ++i) {                     // 3 s: niat diperbarui tiap 200 ms
        if (i % 20 == 0) box.c.push_back(gi);
        g_roll = 0.01f * sinf((float)i * 0.3f);
        step(1); t = T();
        fin2 &= isFiniteVal(t.dxi) && isFiniteVal(t.autoH) && t.mode == Mode::WALK_CONT;
        if (t.intentVx > maxStride) maxStride = t.intentVx;
        if (t.jikuasi != lastJ) { ++swaps; lastJ = t.jikuasi; }
        for (int b = 0; b < 2; ++b) for (int s = 0; s < 11; ++s) { lo2 = t.cmd[b][s] < lo2 ? t.cmd[b][s] : lo2; hi2 = t.cmd[b][s] > hi2 ? t.cmd[b][s] : hi2; }
        k2min = t.kin.K2[0] < k2min ? t.kin.K2[0] : k2min; k2max = t.kin.K2[0] > k2max ? t.kin.K2[0] : k2max;
    }
    printf("      WALK_CONT 3 s: %d ganti kaki, stride akhir %.1f mm, K2 kanan %d..%d, cmd %d..%d\n",
           swaps, (double)maxStride, k2min, k2max, lo2, hi2);
    CHECK(fin2 && swaps >= 14, "WALK_CONT stabil numerik, melangkah berulang (>=14 ganti kaki / 3 s)");
    CHECK(fabsf(maxStride - 8.0f) < 0.01f, "stride naik bertahap (maks 3 mm/langkah) sampai target 8 mm");
    CHECK(lo2 >= ICS_POS_MIN && hi2 <= ICS_POS_MAX && k2max - k2min > 40, "command dalam batas & yaw aktif");
    g_roll = 0;
    bool ended = false; int cyc = 0;
    for (; cyc < 400 && !ended; ++cyc) { step(1); ended = T().mode == Mode::BALANCE; }
    CHECK(ended, "niat kedaluwarsa -> stride turun ke 0 -> WALK_END -> BALANCE");
    printf("      berhenti dalam %d siklus setelah niat terakhir\n", cyc);
    box.c.push_back(gi); step(1);
    for (int i = 0; i < 30; ++i) { if (i % 10 == 0) box.c.push_back(gi); step(1); }
    g_pitch = 0.5f; step(6);
    CHECK(T().mode == Mode::FALLEN && !T().intentActive, "jatuh saat WALK_CONT -> FALLEN, niat dibatalkan");
    g_pitch = 0;

    // ================= Fase 2: live pose (820) =================
    step(20); box.k.push_back('r'); step(1); step(110);
    CHECK(T().mode == Mode::READY, "kembali READY sebelum uji live pose");
    Command lp{}; lp.type = CmdType::POSE_TARGET; lp.ttlMs = 300;
    lp.poseMask = 1u << 4;                              // hanya siku kanan
    lp.pose[0][4] = -900; lp.pose[1][4] = 1234;         // L diabaikan (tidak di mask)
    box.c.push_back(lp); step(1);
    CHECK(T().mode == Mode::LIVE_POSE, "POSE_TARGET dari READY -> LIVE_POSE");
    for (int i = 0; i < 40; ++i) { if (i % 10 == 0) box.c.push_back(lp); step(1); }   // host kirim ulang < ttl
    CHECK(T().kin.E[0] == -900 && T().kin.E[1] == 0 && T().kin.U0[0] == -2700, "hanya sendi di mask yang bergerak");
    const int16_t c0 = T().cmd[0][4]; step(1);
    CHECK(T().cmd[0][4] == c0, "output sampai target (slew servoMaxStep)");
    Command bad = lp; bad.pose[0][4] = -3000; box.c.push_back(bad); step(2);
    CHECK(T().kin.E[0] == -900, "target di luar batas sendi ditolak, pose lama dipertahankan");
    step(35);
    CHECK(T().mode == Mode::RESET || T().mode == Mode::READY, "ttl habis -> ramp HOME");
    step(110);
    CHECK(T().mode == Mode::READY && T().kin.E[0] == 0, "kembali HOME/READY setelah live pose");
    box.k.push_back('g'); step(2);
    box.c.push_back(lp); step(2);
    CHECK(T().mode == Mode::BALANCE, "POSE_TARGET ditolak saat BALANCE");
    box.k.push_back('r'); step(110);
    box.c.push_back(lp); step(2);
    Command fin0 = lp; fin0.ttlMs = 0; box.c.push_back(fin0); step(1);
    CHECK(T().mode == Mode::RESET, "POSE_TARGET ttl=0 -> selesai, ramp HOME");

    // ================= Fase 3: refleks rintangan (PERCEPTION) =================
    step(110);
    box.k.push_back('g'); step(3);
    Command g3{}; g3.type = CmdType::GAIT_INTENT; g3.value = 9; g3.ttlMs = 500;
    Command ob{}; ob.type = CmdType::PERCEPTION; ob.value = 150; ob.ttlMs = 400;   // 150 mm < 250 mm
    float maxFwd = 0; bool flagSeen = false;
    for (int i = 0; i < 200; ++i) {
        if (i % 20 == 0) { box.c.push_back(g3); box.c.push_back(ob); }
        step(1); t = T();
        maxFwd = t.intentVx > maxFwd ? t.intentVx : maxFwd; flagSeen |= t.obstacleBlock;
    }
    CHECK(T().mode == Mode::WALK_CONT && maxFwd == 0.0f && flagSeen && T().obstacleFrontMm == 150,
          "rintangan 150 mm: robot melangkah di tempat, maju diblokir firmware");
    ob.value = 900;
    for (int i = 0; i < 200; ++i) { if (i % 20 == 0) { box.c.push_back(g3); box.c.push_back(ob); } step(1); }
    CHECK(T().intentVx > 5.0f && !T().obstacleBlock, "rintangan menjauh (900 mm): langkah maju kembali");
    ob.value = 100; box.c.push_back(ob); step(1);
    for (int i = 0; i < 80; ++i) { if (i % 20 == 0) box.c.push_back(g3); step(1); }       // laporan HP basi
    CHECK(!T().obstacleBlock && T().obstacleFrontMm == 0xFFFF, "laporan rintangan kedaluwarsa (ttl) tidak memblokir");
    Command mundur = g3; mundur.value = -6; ob.ttlMs = 1000; box.c.push_back(ob);
    for (int i = 0; i < 120; ++i) { if (i % 20 == 0) { box.c.push_back(mundur); box.c.push_back(ob); } step(1); }
    CHECK(T().intentVx < -2.0f, "mundur tetap diizinkan saat ada rintangan di depan");
    return checkSummary("test_controller");
}
