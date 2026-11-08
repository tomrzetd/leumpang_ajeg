// Tes unit + fuzz protokol LAP (C++)
#include "leumpang/lap.h"
#include "leumpang/lap_endpoint.h"
#include "check.h"

#include <random>
#include <vector>

using namespace la;
using namespace la::lap;

static std::vector<uint8_t> enc(const Packet& p) {
    uint8_t f[MAX_FRAME];
    const size_t n = encodeFrame(p, f, sizeof(f));
    return std::vector<uint8_t>(f, f + n);
}
static std::vector<Packet> decAll(Decoder& d, const std::vector<uint8_t>& bytes) {
    std::vector<Packet> out; Packet p;
    for (uint8_t b : bytes) if (d.feed(b, p)) out.push_back(p);
    return out;
}

static void testCrc() {
    const char* s = "123456789";
    CHECK_EQ(crc16((const uint8_t*)s, 9), 0x29B1, "CRC-16/CCITT-FALSE vektor standar");
}

static void testCobsVectors() {
    struct V { std::vector<uint8_t> in, out; } v[] = {
        {{0x00}, {0x01, 0x01}},
        {{0x00, 0x00}, {0x01, 0x01, 0x01}},
        {{0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33}},
        {{0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44}},
        {{0x11, 0x00, 0x00, 0x00}, {0x02, 0x11, 0x01, 0x01, 0x01}},
    };
    bool ok = true;
    for (auto& t : v) {
        uint8_t o[16], d[16];
        const size_t n = cobsEncode(t.in.data(), t.in.size(), o);
        ok &= n == t.out.size() && !memcmp(o, t.out.data(), n);
        const size_t m = cobsDecode(o, n, d);
        ok &= m == t.in.size() && !memcmp(d, t.in.data(), m);
    }
    // 254 byte non-nol -> blok 0xFF
    std::vector<uint8_t> big(254, 0x5A); uint8_t o[300], d[300];
    const size_t n = cobsEncode(big.data(), big.size(), o);
    ok &= o[0] == 0xFF && cobsDecode(o, n, d) == 254 && !memcmp(d, big.data(), 254);
    CHECK(ok, "COBS vektor Wikipedia + blok 254 byte");
}

static void testRoundtripRandom() {
    std::mt19937 rng(99);
    Decoder d; int good = 0;
    for (int i = 0; i < 5000; ++i) {
        Packet p; p.type = (uint8_t)(rng() % 0x80); p.seq = (uint8_t)rng(); p.len = (uint8_t)(rng() % (MAX_BODY + 1));
        for (int k = 0; k < p.len; ++k) p.body[k] = (rng() % 4 == 0) ? 0 : (uint8_t)rng();
        auto f = enc(p);
        auto got = decAll(d, f);
        if (got.size() == 1 && got[0].type == p.type && got[0].seq == p.seq && got[0].len == p.len &&
            !memcmp(got[0].body, p.body, p.len)) ++good;
    }
    CHECK_EQ(good, 5000, "5000 paket acak (body 0..240, banyak byte nol) roundtrip");
}

static void testResync() {
    Decoder d;
    Packet a; CmdMsg c; c.cmd = CMD_GO; makePacket(a, CMD, 7, c);
    std::vector<uint8_t> s;
    const char* text = "TELE m=720 teks log biasa\r\n";
    s.insert(s.end(), text, text + strlen(text));
    s.push_back(0);                                    // sampah teks diakhiri delimiter
    auto f = enc(a); s.insert(s.end(), f.begin(), f.end());
    auto f2 = enc(a); f2[3] ^= 0x40; s.insert(s.end(), f2.begin(), f2.end());     // CRC rusak
    s.insert(s.end(), f.begin(), f.begin() + 3);       // terpotong...
    s.push_back(0);                                    // ...lalu delimiter
    s.insert(s.end(), f.begin(), f.end());
    auto got = decAll(d, s);
    CHECK_EQ((int)got.size(), 2, "resync: teks, CRC rusak, frame terpotong dibuang; 2 frame valid diterima");
    CHECK(d.crcErrors + d.cobsErrors >= 2, "error terhitung");
}

static void testFuzz() {
    std::mt19937 rng(5);
    Decoder d; Packet p; long n = 0;
    for (int i = 0; i < 2000000; ++i) if (d.feed((uint8_t)(rng() % 7 == 0 ? 0 : rng()), p)) ++n;
    std::vector<uint8_t> longJunk(5000, 0x33); longJunk.push_back(0);
    decAll(d, longJunk);
    CHECK(d.overflows >= 1, "frame kepanjangan -> overflow, tidak crash");
    printf("      fuzz 2M byte: %ld frame lolos CRC kebetulan, cobs=%u crc=%u\n", n, d.cobsErrors, d.crcErrors);
    CHECK(n < 50, "fuzz 2 juta byte acak tanpa crash (ASan/UBSan)");
}

static void testMessages() {
    Packet p; bool ok = true;
    TelemetryMsg t; t.ms = 123456; t.mode = 730; t.flags = TF_CAL_DONE; t.rollDeg = 1.5f; t.pitchDeg = -2.25f;
    t.imuAgeMs = 3; t.imuHz = 500; t.fwct = 12; t.autoH = 181.5f; t.loopAvgUs = 900; t.loopMaxUs = 1200;
    t.servoPhaseUs = 850; t.overruns = 2; t.fbOk = 21; t.cfgGen = 9;
    ok &= makePacket(p, TELEMETRY, 1, t);
    TelemetryMsg t2; ok &= parsePacket(p, t2) && p.len == 43 && t2.ms == t.ms && t2.mode == 730 && t2.rollDeg == 1.5f &&
        t2.pitchDeg == -2.25f && t2.imuAgeMs == 3 && t2.autoH == 181.5f && t2.overruns == 2 && t2.fbOk == 21 && t2.cfgGen == 9;
    ServoStateMsg s; s.cmd[1][10] = -1234; s.valid[0][3] = 1;
    ok &= makePacket(p, SERVO_STATE, 2, s) && p.len == 154;
    ServoStateMsg s2; ok &= parsePacket(p, s2) && s2.cmd[1][10] == -1234 && s2.valid[0][3] == 1;
    GaitStateMsg g; g.kin[21] = 77; ok &= makePacket(p, GAIT_STATE, 3, g) && p.len == 93;
    Hello h; memcpy(h.name, "abcdefghijklmnop", 16); ok &= makePacket(p, HELLO, 4, h) && p.len == 26;
    Hello h2; ok &= parsePacket(p, h2) && !strcmp(h2.name, "abcdefghijklmnop");
    TelemetryMsg shortMsg; Packet q = p; q.len = 5; ok &= !parsePacket(q, shortMsg);
    CHECK(ok, "encode/decode TELEMETRY(43B) SERVO_STATE(154B) GAIT_STATE(93B) HELLO(26B); body pendek ditolak");
}

// ---- endpoint ----
struct EpHarness {
    LapEndpoint ep; Decoder d; std::vector<Packet> out; std::vector<Command> cmds; Telemetry tele{};
    bool full = false;
    EpHarness() {
        LapEndpoint::Host h; h.ctx = this;
        h.write = [](void* c, const uint8_t* b, size_t n) { auto* s = (EpHarness*)c; Packet p; for (size_t i = 0; i < n; ++i) if (s->d.feed(b[i], p)) s->out.push_back(p); };
        h.submit = [](void* c, const Command& m) { auto* s = (EpHarness*)c; if (s->full) return false; s->cmds.push_back(m); return true; };
        h.snapshot = [](void* c, Telemetry& t) { t = ((EpHarness*)c)->tele; };
        Hello self; self.board = BOARD_SIM; strcpy(self.name, "test");
        tele.mode = Mode::READY; tele.cfg = runtimeConfigDefault();
        ep.init(h, self);
    }
    template <typename M> void rx(uint8_t type, uint8_t seq, const M& m, uint32_t ms) {
        Packet p; makePacket(p, type, seq, m); auto f = enc(p); ep.onBytes(f.data(), f.size(), ms);
    }
    int count(uint8_t type) const { int n = 0; for (auto& p : out) if (p.type == type) ++n; return n; }
    const Packet* last(uint8_t type) const { const Packet* r = nullptr; for (auto& p : out) if (p.type == type) r = &p; return r; }
};

static void testEndpoint() {
    EpHarness e;
    e.ep.poll(0); e.ep.poll(100);
    CHECK_EQ((int)e.out.size(), 0, "tidak ada stream sebelum HELLO dari host");
    Hello me; me.role = ROLE_DESKTOP; e.rx(HELLO, 1, me, 1000);
    CHECK(e.count(HELLO) == 1 && e.count(CONFIG) == 1, "HELLO dijawab HELLO + CONFIG");
    for (uint32_t t = 1000; t <= 2000; t += 10) e.ep.poll(t);
    CHECK(e.count(TELEMETRY) >= 19 && e.count(TELEMETRY) <= 21, "stream TELEMETRY 20 Hz default");
    CmdMsg c; c.cmd = CMD_GO; e.rx(CMD, 9, c, 2000);
    Ack a; parsePacket(*e.last(ACK), a);
    CHECK(e.cmds.size() == 1 && e.cmds[0].type == CmdType::GO && a.ackSeq == 9 && a.status == OK, "CMD go -> Command GO + ACK OK");
    SetParamMsg sp; sp.param = 200; sp.value = 1; e.rx(SET_PARAM, 10, sp, 2000);
    parsePacket(*e.last(ACK), a); CHECK_EQ(a.status, BAD_VALUE, "param id tak dikenal -> BAD_VALUE");
    GaitIntentMsg gi; e.rx(GAIT_INTENT, 11, gi, 2000);
    parsePacket(*e.last(ACK), a); CHECK_EQ(a.status, UNSUPPORTED, "GAIT_INTENT -> UNSUPPORTED (Fase 1)");
    SimImuMsg si; e.rx(SIM_IMU, 12, si, 2000);
    parsePacket(*e.last(ACK), a); CHECK_EQ(a.status, UNSUPPORTED, "SIM_IMU tanpa simHook -> UNSUPPORTED");
    e.full = true; e.rx(CMD, 13, c, 2000);
    parsePacket(*e.last(ACK), a); CHECK_EQ(a.status, BUSY, "antrean penuh -> BUSY");
    e.tele.cfgGen = 5; const int cfgBefore = e.count(CONFIG); e.ep.poll(2100);
    CHECK_EQ(e.count(CONFIG), cfgBefore + 1, "cfgGen berubah -> CONFIG di-broadcast");
    const int teleBefore = e.count(TELEMETRY);
    for (uint32_t t = 2100; t <= 6000; t += 10) e.ep.poll(t);
    CHECK(!e.ep.linkActive() && e.count(TELEMETRY) - teleBefore < 60, "tanpa paket host > 3 s -> stream berhenti");
}

int main() {
    testCrc(); testCobsVectors(); testRoundtripRandom(); testResync(); testFuzz(); testMessages(); testEndpoint();
    return checkSummary("test_lap");
}
