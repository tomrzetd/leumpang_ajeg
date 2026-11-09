// Profil robot (platform servo umum): JSON <-> profil, validasi, dan pengaruhnya ke controller.
#include "leumpang/controller.h"
#include "leumpang/profile.h"
#include "check.h"

#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

using namespace la;

static int64_t g_now = 1000000;
struct Bus : ServoBus {
    float pos[2][32]; int tgt[2][32];
    Bus() { for (auto& b : pos) for (float& p : b) p = 7500; }
    void exchange(IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) {
            const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
            x.got[b] = true; x.pos[b] = (int16_t)pos[b][id];
            if (p) { pos[b][id] = (float)p; tgt[b][id] = p; }
        }
    }
};
struct Box : Inbox {
    std::deque<char> k;
    bool nextKey(char& c) override { if (k.empty()) return false; c = k.front(); k.pop_front(); return true; }
    bool nextCommand(Command&) override { return false; }
};
static std::string g_log;

static void run(Controller& c, Bus& bus, Box& box, int n) {
    static uint32_t seq = 0;
    for (int i = 0; i < n; ++i) { ImuInput in{true, true, 0, 0, g_now, ++seq, 500}; c.step(in, bus, box); g_now += 10000; }
}
static void boot(Controller& c, Bus& bus, const RobotProfile* p) {
    static const int16_t trim[2][SERVO_SLOTS] = {};
    Controller::Options o; o.profile = p;
    c.init(Platform{[](void*) { return g_now; }, [](void*, const char* l) { g_log += l; }, nullptr}, runtimeConfigDefault(), trim, o);
    for (int i = 0; i < BOOT_SYNC_CYCLES; ++i) { c.bootFreeCycle(bus); g_now += 10000; }
    c.bootFinish(true);
}

int main() {
    char err[160] = "";
    static char buf[8192];
    const RobotProfile& def = defaultProfile();
    CHECK(def.humanoid() && def.presentCount() == 22 && validateProfile(def, err, sizeof err), "profil bawaan KHR valid, 22 servo");
    const size_t n = writeProfile(def, buf, sizeof buf);
    RobotProfile back;
    CHECK(n > 0 && parseProfile(buf, n, back, err, sizeof err), "bawaan -> JSON -> profil");
    CHECK(!memcmp(back.joint, def.joint, sizeof def.joint) && !memcmp(back.home, def.home, sizeof def.home) &&
          !strcmp(back.jname[0][4], "R ELBOW"), "roundtrip identik (joint, home, nama)");

    // kalibrasi: hanya beberapa servo diubah, sisanya bawaan; nama sendi bisa dipakai sebagai alamat
    const char* tune = R"({"name":"khr saya","kind":"humanoid-khr","servos":[
        {"name":"R ELBOW","center":4900,"min":4000,"max":9000},
        {"bus":1,"slot":4,"id":4,"sign":-1,"model":"KRS-3304","home":-500,"center":9500}]})";
    RobotProfile t;
    const bool tuneOk = parseProfile(tune, strlen(tune), t, err, sizeof err);
    if (!tuneOk) printf("      galat: %s\n", err);
    CHECK(tuneOk && t.joint[0][4].center == 4900 && t.joint[0][4].lo == 4000 && t.joint[1][4].sign == -1 &&
          !strcmp(t.model[1][4], "KRS-3304") && t.joint[0][7].center == def.joint[0][7].center && t.fromSd,
          "profil parsial: servo tertentu diubah, lainnya bawaan");

    // robot lain: lengan 4 servo
    const char* arm = R"({"name":"lengan-4dof","kind":"generic","servos":[
        {"bus":0,"slot":0,"id":1,"name":"BASE","center":7500},
        {"bus":0,"slot":1,"id":2,"name":"BAHU","center":7000,"sign":-1,"min":5000,"max":10000,"home":-300},
        {"bus":0,"slot":2,"id":3,"name":"SIKU"},
        {"bus":1,"slot":0,"id":4,"name":"CAPIT","min":6000,"max":9000}]})";
    RobotProfile g;
    const bool armOk = parseProfile(arm, strlen(arm), g, err, sizeof err);
    if (!armOk) printf("      galat: %s\n", err);
    CHECK(armOk && !g.humanoid() && g.presentCount() == 4 && g.home[0][1] == -300 && !strcmp(g.jname[1][0], "CAPIT"),
          "profil generic: 4 servo, slot lain tidak terpasang");

    struct Bad { const char* json; const char* why; };
    const Bad bad[] = {
        {R"({"kind":"hexapod"})", "kind tidak dikenal"},
        {R"({"kind":"generic","servos":[{"bus":0,"slot":0,"id":1},{"bus":0,"slot":1,"id":1}]})", "id ganda"},
        {R"({"kind":"generic","servos":[{"bus":0,"slot":0,"min":9000,"max":8000}]})", "min >= max"},
        {R"({"kind":"generic","servos":[{"bus":0,"slot":0,"center":12000}]})", "center di luar"},
        {R"({"kind":"generic","servos":[{"bus":2,"slot":0}]})", "bus tidak valid"},
        {R"({"kind":"humanoid-khr","servos":[{"name":"R KNEE","present":false}]})", "humanoid kurang sendi"},
        {R"({"kind":"generic","servos":[{"bus":0,"slot":0,"home":9000}]})", "HOME di luar batas"},
        {R"({"kind":"generic","servos":[{"bus":0,"slot":0)", "JSON terpotong"},
        {"bukan json", "bukan JSON"},
    };
    int rejected = 0;
    for (const Bad& b : bad) {
        RobotProfile x;
        err[0] = 0;
        if (!parseProfile(b.json, strlen(b.json), x, err, sizeof err) && err[0]) ++rejected;
        else printf("      TIDAK ditolak: %s\n", b.why);
    }
    CHECK(rejected == (int)(sizeof bad / sizeof bad[0]), "9 profil rusak ditolak dengan pesan galat");

    // controller memakai profil
    {
        static Controller c; Bus bus; Box box;
        boot(c, bus, &t);
        run(c, bus, box, 200);
        Telemetry tm; c.snapshot(tm);
        CHECK(tm.mode == Mode::READY, "controller dengan profil kalibrasi -> READY");
        CHECK(bus.tgt[0][4] == 4900 + t.home[0][4], "center kalibrasi dipakai untuk perintah servo (R ELBOW)");
        CHECK(c.profile().joint[1][4].center == 9500 && bus.tgt[1][4] == 9500 - t.home[1][4],
              "sign, center & HOME kalibrasi dipakai (L ELBOW, KRS-3304)");
    }
    {
        static Controller c; Bus bus; Box box;
        g_log.clear();
        boot(c, bus, &g);
        run(c, bus, box, 200);
        Telemetry tm; c.snapshot(tm);
        CHECK(tm.mode == Mode::READY && bus.tgt[0][2] == 7000 + 300, "profil generic: HOME dari profil (BAHU = 7000 - (-300))");
        box.k.push_back('g'); run(c, bus, box, 3); c.snapshot(tm);
        CHECK(tm.mode == Mode::READY && g_log.find("generic") != std::string::npos, "profil generic: GO (gait) ditolak");
        CHECK(g_log.find("/4 posisi servo") != std::string::npos, "boot sync menghitung servo terpasang saja");
    }
    return checkSummary("test_profile");
}
