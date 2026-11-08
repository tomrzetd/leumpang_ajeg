// Uji paritas: firmware V4 lama (tests/legacy) vs leumpang-core baru, input identik.
#include "../tests/legacy/shared.h"
#include "idf_stub.h"
#include "leumpang/controller.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <vector>
#include <random>

void host_control_step(); void host_boot();
void legacy_out(int16_t o[2][SERVO_SLOTS], uint16_t* m, ServoPositions* k, float* roll, float* pitch);

// ---------------- model servo (dipakai kedua sisi, instance terpisah) ----------------
struct ServoModel {
    float pos[2][32]; int cmd[2][32]; bool freeT[2][32]; bool dead[2][32];
    void init(unsigned seed) {
        std::mt19937 rng(seed);
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
            pos[b][i] = 7500.0f + (float)((int)(rng() % 2000) - 1000); cmd[b][i] = 0; freeT[b][i] = true; dead[b][i] = false; }
    }
    // kembalikan posisi saat command diterima (semantik ICS 3.5)
    bool frame(int b, const uint8_t* f, int16_t* out) {
        const int id = f[0] & 0x1F, p = (f[1] << 7) | f[2];
        if (dead[b][id]) return false;
        *out = (int16_t)lroundf(pos[b][id]);
        if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; cmd[b][id] = p; }
        return true;
    }
    void physics() {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 11; ++i) {
            if (freeT[b][i] || dead[b][i]) continue;
            const float d = cmd[b][i] - pos[b][i], m = 120.0f;
            pos[b][i] += d > m ? m : (d < -m ? -m : d);
        }
    }
    void drift(float v) { for (int b = 0; b < 2; ++b) for (int i = 0; i < 11; ++i) if (freeT[b][i]) pos[b][i] += v; }
};
static ServoModel MA, MB;

// ---------------- sisi LEGACY: stub IDF ----------------
static int64_t gL = 1000000;
static float gRoll = 0, gPitch = 0; static uint32_t gSeq = 0; static int64_t gImuT = 0;
struct RxQ { std::deque<uint8_t> q; } RX[3];
extern "C" {
esp_event_base_t const WIFI_EVENT = "W"; esp_event_base_t const IP_EVENT = "I";
int64_t esp_timer_get_time(void) { return ++gL; }
void vPortEnterCritical(portMUX_TYPE*) {} void vPortExitCritical(portMUX_TYPE*) {}
struct Q { size_t sz, cap; std::deque<std::vector<uint8_t>> d; };
QueueHandle_t xQueueCreate(UBaseType_t n, UBaseType_t s) { return new Q{s, n, {}}; }
BaseType_t xQueueSend(QueueHandle_t h, const void* p, TickType_t) { Q* q = (Q*)h; if (q->d.size() >= q->cap) return 0;
    q->d.emplace_back((const uint8_t*)p, (const uint8_t*)p + q->sz); return 1; }
BaseType_t xQueueReceive(QueueHandle_t h, void* p, TickType_t) { Q* q = (Q*)h; if (q->d.empty()) return 0;
    memcpy(p, q->d.front().data(), q->sz); q->d.pop_front(); return 1; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t, TaskHandle_t*, BaseType_t) { return 1; }
void vTaskDelay(TickType_t t) { gL += (int64_t)t * 1000; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t) { return 0; }
TickType_t xTaskGetTickCount(void) { return 0; }
BaseType_t xTaskDelayUntil(TickType_t*, TickType_t) { return 1; }
esp_err_t gpio_config(const gpio_config_t*) { return 0; }
esp_err_t gpio_set_level(gpio_num_t, uint32_t) { return 0; }
esp_err_t gpio_set_pull_mode(gpio_num_t, gpio_pull_mode_t) { return 0; }
esp_err_t uart_driver_install(uart_port_t, int, int, int, QueueHandle_t*, int) { return 0; }
esp_err_t uart_param_config(uart_port_t, const uart_config_t*) { return 0; }
esp_err_t uart_set_pin(uart_port_t, int, int, int, int) { return 0; }
esp_err_t uart_set_mode(uart_port_t, uart_mode_t) { return 0; }
esp_err_t uart_set_line_inverse(uart_port_t, uint32_t) { return 0; }
esp_err_t uart_set_rx_full_threshold(uart_port_t, int) { return 0; }
esp_err_t uart_set_rx_timeout(uart_port_t, const uint8_t) { return 0; }
esp_err_t uart_flush_input(uart_port_t p) { RX[p].q.clear(); return 0; }
int uart_write_bytes(uart_port_t p, const void* d, size_t n) {
    const uint8_t* f = (const uint8_t*)d; int16_t pos;
    if (MA.frame(p == 1 ? 0 : 1, f, &pos)) {
        RX[p].q.push_back((uint8_t)(f[0] & 0x7F)); RX[p].q.push_back((uint8_t)((pos >> 7) & 0x7F)); RX[p].q.push_back((uint8_t)(pos & 0x7F)); }
    return (int)n;
}
esp_err_t uart_wait_tx_idle_polling(uart_port_t) { return 0; }
esp_err_t uart_get_buffered_data_len(uart_port_t p, size_t* n) { *n = RX[p].q.size(); return 0; }
int uart_read_bytes(uart_port_t p, void* buf, uint32_t len, TickType_t) {
    uint8_t* o = (uint8_t*)buf; uint32_t k = 0;
    while (k < len && !RX[p].q.empty()) { o[k++] = RX[p].q.front(); RX[p].q.pop_front(); }
    return (int)k; }
}
bool imu_get_latest(ImuSample* o) { memset(o, 0, sizeof(*o)); o->roll = gRoll; o->pitch = gPitch; o->tUs = gImuT; o->seq = gSeq; return true; }
bool imu_is_ok() { return true; }
uint16_t imu_rate_hz() { return 500; }
void logPrintf(const char*, ...) {}

// ---------------- sisi CORE ----------------
static int64_t gC = 1000000;
struct Bus : la::ServoBus {
    void exchange(la::IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) x.got[b] = MB.frame(b, x.tx[b], &x.pos[b]);
    }
} busB;
struct Box : la::Inbox {
    std::deque<char> k; std::deque<la::Command> c;
    bool nextKey(char& v) override { if (k.empty()) return false; v = k.front(); k.pop_front(); return true; }
    bool nextCommand(la::Command& v) override { if (c.empty()) return false; v = c.front(); c.pop_front(); return true; }
} box;
static la::Controller ctrl;

// ---------------- driver skenario ----------------
static int64_t T0 = 1000000; static int cyc = 0; static long diffs = 0, compared = 0; static std::vector<std::pair<unsigned,long>> hist;
static void key(char c) { control_push_key(c); box.k.push_back(c); }
static void cmd(RobotCmdType t, ParamId p, uint8_t bus, uint8_t slot, float v) {
    RobotCommand a{}; a.type = t; a.param = p; a.bus = bus; a.slot = slot; a.value = v; control_queue_cmd(a);
    la::Command b{}; b.type = (la::CmdType)(int)t; b.param = (la::ParamId)(int)p; b.bus = bus; b.slot = slot; b.value = v; box.c.push_back(b);
}
static void cycle() {
    const int64_t t = T0 + (int64_t)cyc * 10000;
    gL = t; gC = t; gImuT = t; ++gSeq;
    host_control_step();
    la::ImuInput in{true, true, gRoll, gPitch, t, gSeq, 500};
    ctrl.step(in, busB, box);
    MA.physics(); MB.physics();
    int16_t oa[2][SERVO_SLOTS]; uint16_t ma; ServoPositions ka; float ra, pa;
    legacy_out(oa, &ma, &ka, &ra, &pa);
    la::Telemetry tb; ctrl.snapshot(tb);
    ++compared;
    { bool f=false; for (auto& h : hist) if (h.first == ma) { ++h.second; f = true; } if (!f) hist.push_back({ma, 1}); }
    bool same = (ma == (uint16_t)tb.mode) && !memcmp(oa, tb.cmd, sizeof(oa)) && !memcmp(&ka, &tb.kin, sizeof(ka)) &&
                ra == tb.roll && pa == tb.pitch;
    if (!same) { if (diffs < 5) printf("DIVERGEN siklus %d: mode %u vs %u\n", cyc, ma, (unsigned)tb.mode); ++diffs; }
    ++cyc;
}
static void run(int n) { for (int i = 0; i < n; ++i) cycle(); }

int main() {
    MA.init(7); MB.init(7);
    MA.dead[1][3] = MB.dead[1][3] = true;
    RuntimeConfig cfgA = runtime_config_default(); int16_t trim[2][SERVO_SLOTS] = {};
    control_init(cfgA, trim);
    la::Platform pf{ [](void*) -> int64_t { return ++gC; }, nullptr, nullptr };
    la::Controller::Options opt;
    ctrl.init(pf, la::runtimeConfigDefault(), trim, opt);
    // boot
    host_boot();
    for (int i = 0; i < la::BOOT_SYNC_CYCLES; ++i) { ctrl.bootFreeCycle(busB); gC += 10000; }
    ctrl.bootFinish(true);
    // skenario deterministik
    run(230);
    cmd(RobotCmdType::SET_TRIM, ParamId::FALL_DEG, 0, 8, 120); run(3);
    key('g'); run(3);
    cmd(RobotCmdType::SET_PARAM, ParamId::SIGN_ROLL, 0, 0, -1); run(2);
    gRoll = 0.06f; run(8); gRoll = 0; run(400);
    gPitch = 0.5f; run(8); key('r'); run(2);
    gPitch = 0; MA.drift(250); MB.drift(250); run(30); key('r'); run(130);
    // fuzz: IMU acak + tombol + perintah acak
    std::mt19937 rng(1234);
    const char keys[] = "rrrgxctttyyzzp012+- kualhebr?";
    for (int i = 0; i < 20000; ++i) {
        const float u = (float)(rng() % 1000) / 1000.0f;
        if (i % 50 == 0) { gRoll = (u - 0.5f) * 0.12f; gPitch = ((float)(rng() % 1000) / 1000.0f - 0.5f) * 0.10f; }
        if (rng() % 1500 == 0) gPitch = 0.6f;                        // jatuh sesekali
        if (rng() % 37 == 0) key(keys[rng() % (sizeof(keys) - 1)]);
        if (rng() % 97 == 0) cmd((RobotCmdType)(rng() % 6), (ParamId)(rng() % 12), (uint8_t)(rng() % 2), (uint8_t)(rng() % 11),
                                 (float)((int)(rng() % 200) - 100) / 4.0f);
        if (rng() % 300 == 0) { MA.drift(40); MB.drift(40); }
        cycle();
    }
    printf("kunjungan mode:"); for (auto& h : hist) printf(" %u:%ld", h.first, h.second); printf("\n");
    printf("paritas: %ld siklus dibandingkan, %ld divergen\n", compared, diffs);
    printf(diffs ? "PARITAS GAGAL\n" : "PARITAS LULUS: core identik dengan firmware V4\n");
    return diffs ? 1 : 0;
}
