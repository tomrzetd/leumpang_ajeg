// Tes pustaka studio (tanpa Qt): FK digital twin vs IK leumpang-core, retarget kamera,
// dokumen gerak vs pemutar firmware, penyederhanaan rekaman, dekode MoveNet.
#include "kinematics.h"
#include "motion_doc.h"
#include "pose_decode.h"
#include "retarget.h"
#include "lap_client.h"
#include "sim_robot.h"

#include "leumpang/controller.h"
#include "check.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <deque>

using namespace la;
using namespace la::studio;

static const float kDeg = 3.14159265f / 180.0f;

// ---- harness controller minimal (sama dengan test_controller) ----
static int64_t g_now = 1000000;
struct Bus : ServoBus {
    int cmd[2][32]{}; bool freeT[2][32];
    float pos[2][32];
    Bus() { for (auto& b : pos) for (float& p : b) p = 7500; for (auto& b : freeT) for (bool& f : b) f = true; }
    void exchange(IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) {
            const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
            uint8_t rx[6] = {x.tx[b][0], x.tx[b][1], x.tx[b][2], (uint8_t)(x.tx[b][0] & 0x7F), 0, 0};
            const int cur = (int)lroundf(pos[b][id]);
            rx[4] = (uint8_t)((cur >> 7) & 0x7F); rx[5] = (uint8_t)(cur & 0x7F);
            x.got[b] = ics::parseReply(rx, 6, x.tx[b], &x.pos[b]);
            if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; pos[b][id] = (float)p; }
        }
    }
} g_bus;
struct Box : Inbox {
    std::deque<char> k; std::deque<Command> c;
    bool nextKey(char& v) override { if (k.empty()) return false; v = k.front(); k.pop_front(); return true; }
    bool nextCommand(Command& v) override { if (c.empty()) return false; v = c.front(); c.pop_front(); return true; }
} g_box;
static Controller g_ctrl;
static const int16_t kZeroTrim[2][SERVO_SLOTS] = {};
static uint32_t g_seq = 0;
static void step(int n) {
    for (int i = 0; i < n; ++i) {
        ImuInput in{true, true, 0.0f, 0.0f, g_now, ++g_seq, 500};
        g_ctrl.step(in, g_bus, g_box);
        g_now += 10000;
    }
}
static Telemetry T() { Telemetry t; g_ctrl.snapshot(t); return t; }
static void kinToSrc(const ServoPositions& v, JointSrc o) {
    for (int b = 0; b < 2; ++b) {
        o[b][0] = b == 0 ? v.waist : v.head;
        o[b][1] = v.U0[b]; o[b][2] = v.U1[b]; o[b][3] = v.U2[b]; o[b][4] = v.E[b];
        o[b][5] = v.K2[b]; o[b][6] = v.K1[b]; o[b][7] = v.K0[b]; o[b][8] = v.H[b]; o[b][9] = v.A0[b]; o[b][10] = v.A1[b];
    }
}

// ---- util ----
static Pose P[gen::LINK_COUNT];
static const int RF = linkIndex("r_foot"), LF = linkIndex("l_foot");
static const int RHY = linkIndex("r_hip_yaw"), LHY = linkIndex("l_hip_yaw");
static const int RHR = linkIndex("r_hip_roll"), LHR = linkIndex("l_hip_roll");
static const int RFA = linkIndex("r_forearm"), LFA = linkIndex("l_forearm");
static const int RUA = linkIndex("r_upperarm2"), LUA = linkIndex("l_upperarm2");

static float footTiltDeg(int foot, int hipYaw) {
    // sol harus sejajar pelvis (selain yaw pinggul): sumbu Z kaki = sumbu Z pelvis
    const Vec3 z = P[foot].q.rotate(Vec3{0, 0, 1});
    (void)hipYaw;
    return std::acos(std::fmin(1.0f, z.z)) / kDeg;
}
static Vec3 handTip(int forearm) { return P[forearm].p + P[forearm].q.rotate(Vec3{70, 0, 0}); }
static float frand(float lo, float hi) { return lo + (hi - lo) * (float)rand() / (float)RAND_MAX; }

int main() {
    srand(3);
    // ================= FK =================
    JointSrc home; homeSrc(home);
    forward(home, P);
    CHECK(footTiltDeg(RF, RHY) < 0.05f && footTiltDeg(LF, LHY) < 0.05f, "FK HOME: kedua sol datar");
    const float hipToAnkle = P[RHR].p.z - P[RF].p.z;
    printf("      HOME: sumbu hip-roll -> ankle-roll %.2f mm (HEIGHT core %.1f)\n", (double)hipToAnkle, (double)HEIGHT);
    CHECK(std::fabs(hipToAnkle - HEIGHT) < 0.5f, "FK HOME: jarak hip-roll ke ankle-roll = HEIGHT core (185 mm)");
    CHECK(handTip(RFA).z < P[linkIndex("r_shoulder")].p.z - 100 && handTip(LFA).z < P[linkIndex("l_shoulder")].p.z - 100,
          "FK HOME: bahu pitch -2700 = lengan tergantung ke bawah");
    {
        JointSrc s; memcpy(s, home, sizeof s);
        s[0][2] = 900; s[1][2] = 900; forward(s, P);
        const float ry = handTip(RFA).y, ly = handTip(LFA).y;
        forward(home, P);
        CHECK(ry < handTip(RFA).y - 20 && ly > handTip(LFA).y + 20, "bahu roll + = lengan membuka ke samping (kanan & kiri)");
        s[0][2] = 0; s[1][2] = 0; s[0][4] = -1100; forward(s, P);
        CHECK(handTip(RFA).x > 40, "siku - = lengan bawah menekuk ke depan");
    }
    Pose W[gen::LINK_COUNT];
    WorldOpts wo; wo.pitchRad = 5 * kDeg;
    world(home, wo, W);
    Vec3 c[4]; float mz = 1e9f;
    for (int f : {RF, LF}) { soleCorners(W[f], f == RF, c); for (auto& v : c) mz = std::fmin(mz, v.z); }
    CHECK(std::fabs(mz) < 1e-3f && W[0].p.z > 200, "pose dunia: titik sol terendah di lantai (z = 0)");

    // ================= FK vs IK core =================
    g_ctrl.init(Platform{[](void*) { return g_now; }, [](void*, const char*) {}, nullptr},
                runtimeConfigDefault(), kZeroTrim, Controller::Options{});
    for (int i = 0; i < BOOT_SYNC_CYCLES; ++i) { g_ctrl.bootFreeCycle(g_bus); g_now += 10000; }
    g_ctrl.bootFinish(true);
    step(400);
    CHECK(T().mode == Mode::READY, "controller READY");
    g_box.k.push_back('g'); step(200);
    Telemetry t = T();
    JointSrc src; kinToSrc(t.kin, src); forward(src, P);
    const Vec3 rel = P[RF].p - P[RHR].p;
    const Vec3 relY = P[RHY].q.conj().rotate(rel);
    printf("      BALANCE: ankle-roll relatif hip-roll (frame hip-yaw) = (%.2f, %.2f, %.2f), autoH %.2f\n",
           (double)relY.x, (double)relY.y, (double)relY.z, (double)t.autoH);
    CHECK(t.mode == Mode::BALANCE && std::fabs(-relY.z - t.autoH) < 0.6f, "BALANCE: tinggi kaki FK = autoH IK core");
    // gait kontinu: kaki harus tetap sejajar tanah sepanjang langkah
    Command pe{}; pe.type = CmdType::SET_PARAM; pe.param = ParamId::GAIT_ENABLE; pe.value = 1; g_box.c.push_back(pe);
    Command gi{}; gi.type = CmdType::GAIT_INTENT; gi.value = 10; gi.yawDeg = 3; gi.ttlMs = 500;
    float worstTilt = 0, maxLift = 0;
    for (int i = 0; i < 300; ++i) {
        if (i % 20 == 0) g_box.c.push_back(gi);
        step(1); t = T(); kinToSrc(t.kin, src); forward(src, P);
        worstTilt = std::fmax(worstTilt, std::fmax(footTiltDeg(RF, RHY), footTiltDeg(LF, LHY)));
        maxLift = std::fmax(maxLift, std::fabs(P[RF].p.z - P[LF].p.z));
    }
    printf("      WALK_CONT 3 s: kemiringan sol maks %.3f deg, beda tinggi kaki maks %.1f mm\n", (double)worstTilt, (double)maxLift);
    CHECK(worstTilt < 0.2f, "WALK_CONT: sol tetap sejajar pelvis (sumbu pitch/roll model konsisten dengan IK)");
    CHECK(maxLift > 5.0f, "WALK_CONT: kaki ayun terangkat");

    // ================= retarget =================
    for (int i = 0; i < 200; ++i) {                                  // arah acak -> sudut -> FK -> arah
        Vec3 d{frand(0.0f, 1.0f), frand(-1, 1), frand(-1, 1)}; d = d.unit();
        for (int bus = 0; bus < 2; ++bus) {
            float pitch, roll;
            const bool ok = shoulderAngles(d, bus, pitch, roll);
            JointSrc s; memcpy(s, home, sizeof s);
            s[bus][1] = (int16_t)lroundf(pitch * gen::CHG_SVA); s[bus][2] = (int16_t)lroundf(roll * gen::CHG_SVA);
            s[bus][3] = 0;
            forward(s, P);
            const Vec3 got = P[bus == 0 ? RUA : LUA].q.rotate(Vec3{1, 0, 0});
            if (!ok || (got - d).norm() > 0.01f) { CHECK(false, "shoulderAngles roundtrip"); i = 1000; break; }
        }
    }
    CHECK(true, "shoulderAngles: 400 arah acak -> FK lengan atas kembali ke arah yang sama");

    // keypoint sintetis: orang menghadap kamera, bahu 100 px
    auto person = [](Keypoint kp[KP_COUNT], Vec3 rUp, Vec3 rFo, Vec3 lUp, Vec3 lFo) {
        for (int k = 0; k < KP_COUNT; ++k) kp[k] = {320, 200, 0.9f};
        // kiri tubuh = +x gambar
        kp[L_SHOULDER] = {370, 200, 0.9f}; kp[R_SHOULDER] = {270, 200, 0.9f}; kp[NOSE] = {320, 150, 0.9f};
        const float U = 80, F = 75;                                   // upperRatio .8, foreRatio .75
        auto put = [&](int a, int b, Vec3 d, float L) { kp[b] = {kp[a].x + d.y * L, kp[a].y - d.z * L, 0.9f}; };
        put(R_SHOULDER, R_ELBOW, rUp, U); put(R_ELBOW, R_WRIST, rFo, F);
        put(L_SHOULDER, L_ELBOW, lUp, U); put(L_ELBOW, L_WRIST, lFo, F);
    };
    Retargeter rt; rt.opts.mirror = false;
    Keypoint kp[KP_COUNT];
    person(kp, {0, -1, 0}, {0, -1, 0}, {0, 1, 0}, {0, 1, 0});          // T-pose
    RetargetResult rr = rt.solve(kp, home);
    printf("      T-pose -> R U0 %d U1 %d E %d | L U0 %d U1 %d E %d\n", rr.src[0][1], rr.src[0][2], rr.src[0][4],
           rr.src[1][1], rr.src[1][2], rr.src[1][4]);
    CHECK(rr.armOk[0] && rr.armOk[1] && std::abs(rr.src[0][2] - 2700) < 40 && std::abs(rr.src[1][2] - 2700) < 40 &&
          std::abs(rr.src[0][4]) < 40, "retarget T-pose: bahu roll ~ +90 deg kedua sisi, siku lurus");
    person(kp, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1});        // lengan turun
    rr = rt.solve(kp, home);
    CHECK(std::abs(rr.src[0][1] + 2700) < 40 && std::abs(rr.src[0][2]) < 40 && std::abs(rr.src[1][1] + 2700) < 40,
          "retarget lengan turun: bahu pitch ~ -90 deg (= HOME)");
    CHECK(rr.mask == ((0x1Eu) | (0x1Eu << 11) | (1u << 11)), "mask: 4 sendi lengan x2 + kepala, kaki tidak");
    rt.opts.mirror = true;
    person(kp, {0, -1, 0}, {0, -1, 0}, {0, 0, -1}, {0, 0, -1});        // hanya tangan kanan manusia terangkat
    rr = rt.solve(kp, home);
    CHECK(std::abs(rr.src[1][2] - 2700) < 40 && std::abs(rr.src[0][2]) < 40, "mode cermin: tangan kanan manusia -> tangan kiri robot");
    rt.opts.mirror = false;
    // pose lengan acak (lengan & lengan bawah tidak mengarah ke belakang) -> keypoint 2D -> retarget -> FK
    int good = 0, total = 0; float worst = 0;
    for (int i = 0; i < 1500; ++i) {
        JointSrc s; memcpy(s, home, sizeof s);
        s[0][1] = (int16_t)(frand(-85, 85) * kDeg * gen::CHG_SVA); s[0][2] = (int16_t)(frand(0, 85) * kDeg * gen::CHG_SVA);
        s[0][3] = (int16_t)(frand(-80, 80) * kDeg * gen::CHG_SVA); s[0][4] = (int16_t)(-frand(20, 120) * kDeg * gen::CHG_SVA);
        bool reachable = true;                                        // hanya pose dalam batas sendi robot
        for (int k = 1; k <= 4; ++k) reachable &= srcWithinLimits(0, (uint8_t)k, s[0][k], 0);
        if (!reachable) continue;
        forward(s, P);
        const Vec3 du = P[RUA].q.rotate(Vec3{1, 0, 0}), df = P[RFA].q.rotate(Vec3{1, 0, 0});
        if (du.x < 0.05f || df.x < 0.05f) continue;                   // kamera 2D tidak bisa membedakan depan/belakang
        ++total;
        person(kp, du, df, {0, 0, -1}, {0, 0, -1});
        rr = rt.solve(kp, home);
        forward(rr.src, P);
        const float e = std::fmax((P[RUA].q.rotate(Vec3{1, 0, 0}) - du).norm(), (P[RFA].q.rotate(Vec3{1, 0, 0}) - df).norm());
        worst = std::fmax(worst, e);
        good += e < 0.12f;                                            // ~7 deg (grid yaw 5 deg)
    }
    printf("      retarget lengan acak: %d/%d cocok (< ~7 deg), galat terburuk %.3f\n", good, total, (double)worst);
    CHECK(total > 50 && good >= total * 95 / 100, "retarget 2D -> sendi -> FK mereproduksi arah lengan atas & bawah");
    {
        JointFilter jf; JointSrc v; memcpy(v, home, sizeof v);
        float maxStep = 0; int16_t prev = 0;
        for (int i = 0; i < 90; ++i) {
            v[0][4] = (int16_t)(-1000 + ((i % 2) ? 60 : -60));         // getar +-60
            jf.apply(v, 1u << 4, 1.0f / 30.0f);
            if (i > 30) maxStep = std::fmax(maxStep, (float)std::abs(v[0][4] - prev));
            prev = v[0][4];
        }
        CHECK(maxStep < 40 && std::abs(v[0][4] + 1000) < 60, "JointFilter meredam getar keypoint");
    }

    // ================= dokumen gerak vs pemutar firmware =================
    MotionDoc doc; doc.name = "uji-studio"; doc.returnHome = false;
    for (int f = 0; f < 3; ++f) {
        Keyframe k; memcpy(k.j, home, sizeof k.j);
        k.ms = (uint16_t)(300 + 100 * f); k.interp = (uint8_t)(f % 3);
        k.j[0][4] = (int16_t)(-400 * (f + 1)); k.j[1][2] = (int16_t)(300 * f);
        doc.frames.push_back(k);
    }
    std::string err;
    const std::vector<uint8_t> lam = doc.toLam(&err);
    MotionDoc back;
    CHECK(!lam.empty() && MotionDoc::fromLam(lam.data(), lam.size(), back, &err) && back.frames.size() == 3 &&
          back.frames[2].j[0][4] == -1200 && back.name == "uji-studio", "MotionDoc -> LAM1 -> MotionDoc");
    g_box.k.push_back('r'); step(150);
    CHECK(T().mode == Mode::READY, "controller kembali READY untuk uji motion");
    CHECK(g_ctrl.motions().load(2, lam.data(), lam.size()), "LAM dari studio diterima MotionStore firmware");
    Command mp{}; mp.type = CmdType::MOTION_PLAY; mp.slot = 2; g_box.c.push_back(mp);
    step(1);
    int maxDev = 0; JointSrc pv;
    for (uint32_t ms = 0; ms <= doc.totalMs() + 30; ms += 10) {
        t = T(); kinToSrc(t.kin, src);
        if (t.mode != Mode::MOTION_PLAY) break;
        // firmware: frame berikut mulai satu siklus setelah frame selesai -> toleransi +-1 siklus
        int best = 1 << 30;
        for (int dt = -10; dt <= 20; dt += 10) {
            if ((int)ms + dt < 0) continue;
            doc.sample((uint32_t)((int)ms + dt), home, pv);
            int d = 0;
            for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) d = std::max(d, std::abs(pv[b][s] - src[b][s]));
            best = std::min(best, d);
        }
        maxDev = std::max(maxDev, best);
        step(1);
    }
    printf("      pratinjau studio vs firmware: deviasi maks %d hitungan\n", maxDev);
    CHECK(maxDev <= 45, "pratinjau MotionDoc::sample = Controller::motionStep (+-1 siklus)");
    Keyframe bad = doc.frames[0]; bad.j[0][4] = -3000; doc.frames.push_back(bad);
    CHECK(!doc.validate(nullptr, &err) && err.find("R ELBOW") != std::string::npos, "validasi menolak sendi di luar batas");

    // ================= simplify rekaman =================
    std::vector<PoseSample> rec;
    for (uint32_t ms = 0; ms <= 6000; ms += 33) {
        PoseSample p{}; p.tMs = ms; memcpy(p.j, home, sizeof p.j);
        p.j[0][2] = (int16_t)(1500 + 1200 * std::sin((float)ms * 0.002f));
        p.j[1][4] = (int16_t)(-600 - 500 * std::sin((float)ms * 0.0035f));
        rec.push_back(p);
    }
    std::vector<Keyframe> kf = simplify(rec, 25);
    MotionDoc sd; sd.frames = kf;
    int dev = 0;
    for (const PoseSample& p : rec) {
        JointSrc o; sd.sample(p.tMs + 500, rec[0].j, o);             // +500: keyframe awal
        for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) dev = std::max(dev, std::abs(o[b][s] - p.j[b][s]));
    }
    printf("      simplify: %zu sampel -> %zu keyframe, deviasi maks %d\n", rec.size(), kf.size(), dev);
    CHECK(kf.size() <= 64 && kf.size() >= 5 && dev <= 27, "simplify: <= 64 keyframe, deviasi <= toleransi");
    CHECK(sd.validate(nullptr, &err), "hasil simplify valid untuk firmware");
    std::vector<Keyframe> kf2 = simplify(rec, 1, 12);
    CHECK(kf2.size() <= 12, "simplify menaikkan toleransi bila keyframe > maks");

    // ================= MoveNet =================
    Letterbox lb = letterbox(640, 480, 192);
    float out[KP_COUNT * 3];
    for (int i = 0; i < KP_COUNT; ++i) {
        const float px = 100.0f + 20.0f * (float)i, py = 50.0f + 10.0f * (float)i;
        out[i * 3 + 0] = (py * lb.scale + lb.padY) / 192.0f; out[i * 3 + 1] = (px * lb.scale + lb.padX) / 192.0f; out[i * 3 + 2] = 0.8f;
    }
    Keypoint dk[KP_COUNT]; decodeMoveNet(out, lb, dk);
    CHECK(std::fabs(dk[16].x - 420) < 0.01f && std::fabs(dk[16].y - 210) < 0.01f && lb.padY > 0 && std::fabs(lb.padX) < 1e-3f,
          "letterbox + dekode MoveNet kembali ke koordinat gambar");

    // ================= LapClient <-> SimRobot (jalur studio tanpa hardware) =================
    {
        static SimRobot sim; static LapClient cli; std::vector<uint8_t> toSim;
        cli.write = [&](const uint8_t* d, size_t n) { toSim.insert(toSim.end(), d, d + n); };
        sim.start([](const uint8_t* d, size_t n) { cli.feed(d, n); });
        auto run = [&](int cycles) {
            for (int i = 0; i < cycles; ++i) {
                if (!toSim.empty()) { std::vector<uint8_t> b; b.swap(toSim); sim.fromHost(b.data(), b.size()); }
                sim.advance(1);
                if (i % 100 == 0) cli.ping();
            }
        };
        cli.begin(LapClient::Rates{});
        run(250);
        CHECK(cli.haveHello && cli.hello.board == lap::BOARD_SIM && (cli.hello.caps & lap::CAP_LIVE_POSE) &&
              cli.haveTele && cli.haveServo && cli.haveGait && cli.haveConfig, "studio<->sim: HELLO, telemetry, servo, gait, config");
        CHECK(cli.tele.mode == (uint16_t)Mode::READY, "sim boot -> READY");
        JointSrc gs; cli.gaitSrc(gs);
        Telemetry st; sim.controller().snapshot(st); JointSrc cs; kinToSrc(st.kin, cs);
        CHECK(!memcmp(gs, cs, sizeof gs), "GAIT_STATE kin -> [bus][slot] sama dengan controller");
        int16_t fbSrc = rawToSrc(0, 1, cli.servo.fb[0][1], cli.servo.trim[0][1]);
        CHECK(std::abs(fbSrc - (-2700)) < 150, "feedback servo -> src (bahu R ~ HOME)");
        const std::vector<uint8_t> lam2 = back.toLam(&err);
        cli.motionUpload(1, lam2.data(), lam2.size(), false);
        run(5);
        CHECK(cli.motions.state[1] == 2 && !strcmp(cli.motions.name[1], "uji-studio"), "unggah LAM lewat LapClient -> slot 1 READY");
        cli.motionPlay(1, true); run(3);
        CHECK(cli.tele.mode == (uint16_t)Mode::MOTION_PLAY, "MOTION_PLAY lewat LapClient");
        run(200);
        int16_t pose[2][SERVO_SLOTS]; memcpy(pose, home, sizeof pose); pose[1][4] = -700;
        for (int i = 0; i < 6; ++i) { cli.poseTarget(pose, 1u << (11 + 4), 300); run(10); }
        cli.gaitSrc(gs);
        CHECK(cli.tele.mode == (uint16_t)Mode::LIVE_POSE && gs[1][4] == -700, "POSE_TARGET lewat LapClient -> LIVE_POSE");
        int logs = 0; cli.onLog = [&](const std::string&) { ++logs; };
        cli.cmd(lap::CMD_RESET); run(150);
        CHECK(cli.tele.mode == (uint16_t)Mode::READY && logs > 0, "CMD_RESET -> READY, LOG diterima");
    }
    return checkSummary("test_studio");
}
