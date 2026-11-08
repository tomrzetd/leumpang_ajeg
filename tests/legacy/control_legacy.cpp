// =============================================================================
//  KHR Biped V4 - TASK KONTROL (core1): bus ICS, kinematika UVC, state machine
//  Port dari biped_esp32s3_n16r8_dashboard_v3.cpp. Rumus gait/IK dipertahankan;
//  perubahan diberi tag [V4-xx].
// =============================================================================
#include "shared.h"

#include <math.h>
#include <string.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// ============================================================================
//  MODE helper (dipakai semua modul)
// ============================================================================
const char* mode_name(Mode m) {
    switch (m) {
        case Mode::RAW_ANGLES: return "RAW_ANGLES";
        case Mode::FALLEN:     return "FALLEN/FREE";
        case Mode::RESET:      return "RESET/HOME";
        case Mode::READY:      return "READY";
        case Mode::BALANCE:    return "BALANCE";
        case Mode::WALK_INIT:  return "WALK_INIT";
        case Mode::WALK_HOLD:  return "WALK_HOLD";
        case Mode::WALK_STEP:  return "WALK_STEP";
        case Mode::WALK_END:   return "WALK_END";
        case Mode::POSE_HOLD:  return "POSE_HOLD";
        case Mode::ANGLE_DISP: return "ANGLE_DISP";
        case Mode::MIRROR_DBG: return "MIRROR_DBG";
    }
    return "UNKNOWN";
}
const char* mode_short_name(Mode m) {
    switch (m) {
        case Mode::RAW_ANGLES: return "RAW";
        case Mode::FALLEN:     return "FALLEN/FREE";
        case Mode::RESET:      return "RESET/HOME";
        case Mode::READY:      return "READY";
        case Mode::BALANCE:    return "BALANCE";
        case Mode::WALK_INIT:  return "WALK INIT";
        case Mode::WALK_HOLD:  return "WALK HOLD";
        case Mode::WALK_STEP:  return "WALK STEP";
        case Mode::WALK_END:   return "WALK END";
        case Mode::POSE_HOLD:  return "POSE HOLD";
        case Mode::ANGLE_DISP: return "ANGLE";
        case Mode::MIRROR_DBG: return "MIRROR";
    }
    return "UNKNOWN";
}
bool mode_is_gait(Mode m) {
    return m == Mode::WALK_INIT || m == Mode::WALK_HOLD || m == Mode::WALK_STEP || m == Mode::WALK_END;
}
bool mode_persist_safe(Mode m) {
    // Tulis flash menghentikan cache kedua core beberapa-puluh ms. Hanya saat robot
    // diam (READY/debug tampilan) atau servo free (FALLEN).
    return m == Mode::READY || m == Mode::FALLEN || m == Mode::ANGLE_DISP || m == Mode::RAW_ANGLES;
}

RuntimeConfig runtime_config_default() {
    RuntimeConfig c{};
    c.fallAngleRad = FALL_ANGLE_RAD; c.rearmAngleRad = REARM_ANGLE_RAD;
    c.imuStaleUs = (uint32_t)IMU_STALE_US; c.servoMaxStep = SERVO_MAX_STEP;
    c.triggerRollRad = 0.033f; c.triggerPitchRad = 0.044f;
    c.footLiftMax = 35.0f; c.swayBase = 17.0f; c.swayGain = 17.0f;
    c.imuRollSign = IMU_ROLL_SIGN; c.imuPitchSign = IMU_PITCH_SIGN;
    c.imuSwap = IMU_SWAP_ROLL_PITCH != 0;
    return c;
}

void runtime_config_sanitize(RuntimeConfig& c) {
    const RuntimeConfig d = runtime_config_default();
    auto fin = [](float v, float def) { return isFiniteVal(v) ? v : def; };
    c.fallAngleRad    = clampf(fin(c.fallAngleRad, d.fallAngleRad), 8.0f / kRad2Deg, 60.0f / kRad2Deg);
    c.rearmAngleRad   = clampf(fin(c.rearmAngleRad, d.rearmAngleRad), 3.0f / kRad2Deg, 30.0f / kRad2Deg);
    c.imuStaleUs      = (uint32_t)clampi((int32_t)c.imuStaleUs, 20000, 500000);
    c.servoMaxStep    = (int16_t)clampi(c.servoMaxStep, 20, 1000);
    c.triggerRollRad  = clampf(fin(c.triggerRollRad, d.triggerRollRad), 0.2f / kRad2Deg, 15.0f / kRad2Deg);
    c.triggerPitchRad = clampf(fin(c.triggerPitchRad, d.triggerPitchRad), 0.2f / kRad2Deg, 15.0f / kRad2Deg);
    c.footLiftMax     = clampf(fin(c.footLiftMax, d.footLiftMax), 0.0f, 60.0f);
    c.swayBase        = clampf(fin(c.swayBase, d.swayBase), 0.0f, 45.0f);
    c.swayGain        = clampf(fin(c.swayGain, d.swayGain), 0.0f, 45.0f);
    c.imuRollSign     = (c.imuRollSign < 0) ? -1 : +1;
    c.imuPitchSign    = (c.imuPitchSign < 0) ? -1 : +1;
    c.imuSwap         = c.imuSwap ? true : false;
    if (c.rearmAngleRad >= c.fallAngleRad) c.rearmAngleRad = 0.5f * c.fallAngleRad;
}

static inline int16_t f2i16(float v, int16_t fallback = 0) {
    if (!isFiniteVal(v)) return fallback;
    return (int16_t)clampf(v, -32000.0f, 32000.0f);
}

// ============================================================================
//  DATA KONTROL (hanya disentuh task kontrol)
// ============================================================================
struct WalkingState {
    Mode    mode;
    int16_t motCt, walkCt;
    float   fwct, fwctEnd, fwctUp;
    float   dxi, dyi, dxis, dyis, dxib, dyib;
    float   autoH, fh, fhMax;
    float   sw, swx, swy, swMax;
    float   waist;
    int16_t landF, landB;
    int8_t  jikuasi;
    uint8_t  fallCt;
    uint16_t uprightCt;
};
struct ImuData {
    float    roll, pitch, rawRoll, rawPitch, offRoll, offPitch;
    bool     initOk;
    int64_t  lastUs;
    uint32_t seq;          // naik tiap sampel baru yang dipakai kontrol
    uint32_t srcSeq;       // seq terakhir dari task IMU
};
struct ImuCal { bool done; uint16_t n; float sumR, sumP, minR, maxR, minP, maxP; uint32_t lastSeq; };

static ServoPositions servos;
static ImuData        imu;
static WalkingState   walk;
static ImuCal         cal;
static RuntimeConfig  g_cfg;
static int16_t        g_trim[2][SERVO_SLOTS];     // [V4-TRIM] offset permanen, disimpan NVS
static uint32_t       g_cfgGen = 0;               // [V4-NVS] generasi cfg untuk penyimpanan tertunda

static const ServoPositions HOME_POSE = {
    {661, 661}, {0, 0}, {0, 0}, {1322, 1322}, {661, 661}, {0, 0},
    {-2700, -2760}, {0, 0}, {0, 0}, {0, 0}, 0, 0
};

static QueueHandle_t g_cmdQ = nullptr;
static QueueHandle_t g_keyQ = nullptr;
static TaskHandle_t  g_ctrlTask = nullptr;

// ============================================================================
//  BUS SERVO ICS 3.5  [V4-ICS]
// ============================================================================
struct Joint { uint8_t id; int16_t center; int8_t sign; int16_t* src; int16_t lo, hi; };

// center = konstanta V3. lo/hi WAJIB dipersempit sesuai batas mekanik per sendi.
static const Joint JOINTS_R[SERVO_SLOTS] = {
    {  0, 7500, +1, &servos.waist,  ICS_POS_MIN, ICS_POS_MAX },   // pinggang
    {  1, 4735, -1, &servos.U0[0],  ICS_POS_MIN, ICS_POS_MAX },   // shoulder pitch
    {  2,10110, -1, &servos.U1[0],  ICS_POS_MIN, ICS_POS_MAX },   // shoulder roll
    {  3, 7500, -1, &servos.U2[0],  ICS_POS_MIN, ICS_POS_MAX },   // shoulder yaw
    {  4, 4800, +1, &servos.E[0],   ICS_POS_MIN, ICS_POS_MAX },   // siku
    {  5, 7500, -1, &servos.K2[0],  ICS_POS_MIN, ICS_POS_MAX },   // hip yaw
    {  6, 7440, -1, &servos.K1[0],  ICS_POS_MIN, ICS_POS_MAX },   // hip roll
    {  7, 7450, -1, &servos.K0[0],  ICS_POS_MIN, ICS_POS_MAX },   // hip pitch
    {  8, 9320, -1, &servos.H[0],   ICS_POS_MIN, ICS_POS_MAX },   // lutut
    {  9, 7920, +1, &servos.A0[0],  ICS_POS_MIN, ICS_POS_MAX },   // ankle pitch
    { 10, 7440, +1, &servos.A1[0],  ICS_POS_MIN, ICS_POS_MAX },   // ankle roll
};
static const Joint JOINTS_L[SERVO_SLOTS] = {
    {  0, 7500, +1, &servos.head,   ICS_POS_MIN, ICS_POS_MAX },   // kepala
    {  1, 9260, +1, &servos.U0[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  2, 4850, +1, &servos.U1[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  3, 7500, +1, &servos.U2[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  4,10150, -1, &servos.E[1],   ICS_POS_MIN, ICS_POS_MAX },
    {  5, 7410, +1, &servos.K2[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  6, 7560, +1, &servos.K1[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  7, 7500, +1, &servos.K0[1],  ICS_POS_MIN, ICS_POS_MAX },
    {  8, 5650, +1, &servos.H[1],   ICS_POS_MIN, ICS_POS_MAX },
    {  9, 7100, -1, &servos.A0[1],  ICS_POS_MIN, ICS_POS_MAX },
    { 10, 7500, -1, &servos.A1[1],  ICS_POS_MIN, ICS_POS_MAX },
};
static inline const Joint& jointAt(uint8_t b, uint8_t s) { return b == 0 ? JOINTS_R[s] : JOINTS_L[s]; }

struct ServoFeedback { int16_t pos; int64_t lastUs; uint32_t okCount, missCount; bool valid; };
static int16_t       g_lastOut[2][SERVO_SLOTS];
static ServoFeedback g_servoFb[2][SERVO_SLOTS];

static const uart_port_t ICS_PORT[2] = { UART_NUM_1, UART_NUM_2 };
static const int ICS_TXP[2] = { BUS_R_TX, BUS_L_TX };
static const int ICS_RXP[2] = { BUS_R_RX, BUS_L_RX };
[[maybe_unused]] static const int ICS_ENP[2] = { BUS_R_EN, BUS_L_EN };

static inline void icsDirTx(uint8_t b, bool tx) {
#if ICS_DIR_MODE == 1
    gpio_set_level((gpio_num_t)ICS_ENP[b], tx ? ICS_EN_TX_LEVEL : !ICS_EN_TX_LEVEL);
#else
    (void)b; (void)tx;
#endif
}

static bool icsInit() {
    bool ok = true;
    for (uint8_t b = 0; b < 2; ++b) {
        const uart_port_t p = ICS_PORT[b];
        uart_config_t c = {};
        c.baud_rate = (int)ICS_BAUD;
        c.data_bits = UART_DATA_8_BITS;
        c.parity    = UART_PARITY_EVEN;
        c.stop_bits = UART_STOP_BITS_1;
        c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        c.source_clk = UART_SCLK_DEFAULT;
        // RX ring 256 B, TX tanpa buffer (3 byte langsung masuk FIFO), tanpa event queue.
        ok &= uart_driver_install(p, 256, 0, 0, nullptr, 0) == ESP_OK;
        ok &= uart_param_config(p, &c) == ESP_OK;
#if ICS_DIR_MODE == 2
        ok &= uart_set_pin(p, ICS_TXP[b], ICS_RXP[b], ICS_ENP[b], UART_PIN_NO_CHANGE) == ESP_OK;
        ok &= uart_set_mode(p, UART_MODE_RS485_HALF_DUPLEX) == ESP_OK;
#if ICS_RS485_INVERT_RTS
        ok &= uart_set_line_inverse(p, UART_SIGNAL_RTS_INV) == ESP_OK;
#endif
#else
        ok &= uart_set_pin(p, ICS_TXP[b], ICS_RXP[b], UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK;
#endif
        gpio_set_pull_mode((gpio_num_t)ICS_RXP[b], GPIO_PULLUP_ONLY);
        // Pindahkan byte ke ring buffer secepatnya: 3 byte reply atau 2 simbol idle.
        uart_set_rx_full_threshold(p, 3);
        uart_set_rx_timeout(p, 2);
#if ICS_DIR_MODE == 1
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << ICS_ENP[b];
        io.mode = GPIO_MODE_OUTPUT;
        ok &= gpio_config(&io) == ESP_OK;
        icsDirTx(b, false);
#endif
    }
    return ok;
}

static inline void icsBuild(uint8_t* f, uint8_t id, uint16_t pos) {
    f[0] = (uint8_t)(0x80 | (id & 0x1F));
    f[1] = (uint8_t)((pos >> 7) & 0x7F);
    f[2] = (uint8_t)(pos & 0x7F);
}

// [V4-ICS] Parser presisi: reply harus tepat di offset 0 (tanpa echo) atau 3 (setelah
// echo yang IDENTIK dengan frame terkirim). Scanner V3 bisa salah-cocok pada byte
// echo + byte pertama reply bila early-exit dipakai.
static bool icsParseReply(const uint8_t* buf, uint8_t n, const uint8_t* sent, int16_t* pos) {
    uint8_t off = 0;
    if (n >= 3 && buf[0] == sent[0] && buf[1] == sent[1] && buf[2] == sent[2]) off = 3;
    if (n < off + 3) return false;
    const uint8_t h = buf[off];
    if (h != (uint8_t)(sent[0] & 0x7F)) return false;      // reply = cmd dengan bit7 = 0
    if ((buf[off + 1] & 0x80) || (buf[off + 2] & 0x80)) return false;
    const int16_t v = (int16_t)(((uint16_t)buf[off + 1] << 7) | buf[off + 2]);
    if (v < ICS_FB_MIN || v > ICS_FB_MAX) return false;
    *pos = v;
    return true;
}

static uint32_t g_servoPhaseUs = 0;

// Kirim slot kanan+kiri bersamaan, tunggu TX selesai (polling, core1 khusus kontrol),
// pindah ke RX, lalu kumpulkan reply dengan EARLY-EXIT (bukan delay tetap 200 us).
static void icsSendSlot(uint8_t slot, bool freeTorque) {
    uint8_t frame[2][3];
    for (uint8_t b = 0; b < 2; ++b) {
        const Joint& j = jointAt(b, slot);
        icsBuild(frame[b], j.id, freeTorque ? ICS_POS_FREE
                                            : (uint16_t)clampi(g_lastOut[b][slot], ICS_POS_MIN, ICS_POS_MAX));
        uart_flush_input(ICS_PORT[b]);
        icsDirTx(b, true);
    }
    for (uint8_t b = 0; b < 2; ++b) uart_write_bytes(ICS_PORT[b], frame[b], 3);
    for (uint8_t b = 0; b < 2; ++b) {
        uart_wait_tx_idle_polling(ICS_PORT[b]);
        icsDirTx(b, false);
    }

    uint8_t rx[2][12];
    uint8_t n[2] = {0, 0};
    bool    got[2] = {false, false};
    int16_t pos[2] = {0, 0};
    const int64_t deadline = esp_timer_get_time() + ICS_REPLY_TIMEOUT_US;
    while (!(got[0] && got[1]) && esp_timer_get_time() < deadline) {
        for (uint8_t b = 0; b < 2; ++b) {
            if (got[b]) continue;
            size_t avail = 0;
            if (uart_get_buffered_data_len(ICS_PORT[b], &avail) != ESP_OK || avail == 0) continue;
            const size_t room = sizeof(rx[b]) - n[b];
            if (room == 0) continue;
            const int r = uart_read_bytes(ICS_PORT[b], rx[b] + n[b], (uint32_t)(avail < room ? avail : room), 0);
            if (r > 0) n[b] = (uint8_t)(n[b] + r);
            got[b] = icsParseReply(rx[b], n[b], frame[b], &pos[b]);
        }
    }

    const int64_t now = esp_timer_get_time();
    for (uint8_t b = 0; b < 2; ++b) {
        ServoFeedback& fb = g_servoFb[b][slot];
        if (got[b]) { fb.pos = pos[b]; fb.lastUs = now; fb.okCount++; fb.valid = true; }
        else {
            fb.missCount++;
            if (fb.lastUs == 0 || (now - fb.lastUs) > SERVO_FB_VALID_US) fb.valid = false;
        }
    }
}

static int16_t jointTarget(uint8_t b, uint8_t s) {
    const Joint& j = jointAt(b, s);
    const int32_t v = (int32_t)j.center + (int32_t)j.sign * ((int32_t)(*j.src) + (int32_t)g_trim[b][s]);
    return (int16_t)clampi(v, j.lo, j.hi);
}
static void primeOutputs() {
    for (uint8_t b = 0; b < 2; ++b)
        for (uint8_t s = 0; s < SERVO_SLOTS; ++s) g_lastOut[b][s] = jointTarget(b, s);
}

static void sendAllServoPositions() {
    const int64_t t0 = esp_timer_get_time();
    for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        for (uint8_t b = 0; b < 2; ++b) {
            const int16_t tgt = jointTarget(b, s), last = g_lastOut[b][s];
            g_lastOut[b][s] = (int16_t)(last + clampi((int32_t)tgt - last, -g_cfg.servoMaxStep, g_cfg.servoMaxStep));
        }
        icsSendSlot(s, false);
    }
    g_servoPhaseUs = (uint32_t)(esp_timer_get_time() - t0);
}
static void sendAllServoFree() {
    const int64_t t0 = esp_timer_get_time();
    for (uint8_t s = 0; s < SERVO_SLOTS; ++s) icsSendSlot(s, true);
    g_servoPhaseUs = (uint32_t)(esp_timer_get_time() - t0);
}

// Baseline dari feedback aktual (re-arm & boot). Memperhitungkan trim.
static uint8_t syncStateFromServoFeedback() {
    uint8_t synced = 0;
    const int64_t now = esp_timer_get_time();
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const ServoFeedback& fb = g_servoFb[b][s];
        const Joint& j = jointAt(b, s);
        if (!fb.valid || fb.lastUs == 0 || (now - fb.lastUs) > SERVO_FB_VALID_US) continue;
        const int16_t raw = (int16_t)clampi(fb.pos, j.lo, j.hi);
        g_lastOut[b][s] = raw;
        const int32_t offs = (int32_t)j.sign * ((int32_t)raw - (int32_t)j.center) - g_trim[b][s];
        *j.src = (int16_t)clampi(offs, -32000, 32000);
        ++synced;
    }
    return synced;
}

// ============================================================================
//  UTILITAS GERAK
// ============================================================================
static void movSv(int16_t* cur, int16_t target) {
    if (walk.motCt <= 0) *cur = target;
    else *cur = (int16_t)(*cur + (int16_t)((target - *cur) / walk.motCt));
}
static void rampTo(const ServoPositions& t) {
    for (int s = 0; s < 2; ++s) {
        movSv(&servos.K0[s], t.K0[s]); movSv(&servos.K1[s], t.K1[s]);
        movSv(&servos.K2[s], t.K2[s]); movSv(&servos.H[s],  t.H[s]);
        movSv(&servos.A0[s], t.A0[s]); movSv(&servos.A1[s], t.A1[s]);
        movSv(&servos.U0[s], t.U0[s]); movSv(&servos.U1[s], t.U1[s]);
        movSv(&servos.U2[s], t.U2[s]); movSv(&servos.E[s],  t.E[s]);
    }
    movSv(&servos.head, t.head);
    movSv(&servos.waist, t.waist);
}
static void initWalkState() {
    walk.dxi = walk.dyi = walk.dxis = walk.dyis = walk.dxib = walk.dyib = 0;
    walk.landF = walk.landB = 0;
    walk.fwctEnd = 18; walk.walkCt = 0; walk.fwctUp = 1; walk.fwct = 1;
    walk.autoH = HEIGHT; walk.sw = walk.swx = walk.swy = 0;
    walk.fh = 0; walk.waist = 0;
    walk.fallCt = 0; walk.uprightCt = 0;
}

// ============================================================================
//  IMU di sisi kontrol: ambil sampel terbaru dari task IMU (core0)  [V4-IMU]
// ============================================================================
static inline bool imuFresh() {
    return imu.lastUs != 0 && (esp_timer_get_time() - imu.lastUs) < (int64_t)g_cfg.imuStaleUs;
}
static void imuPull() {
    imu.initOk = imu_is_ok();
    ImuSample s;
    if (!imu_get_latest(&s) || s.seq == imu.srcSeq) return;
    imu.srcSeq = s.seq;
    float r = s.roll, p = s.pitch;
    if (g_cfg.imuSwap) { const float t = r; r = p; p = t; }
    imu.rawRoll  = r * (float)g_cfg.imuRollSign;
    imu.rawPitch = p * (float)g_cfg.imuPitchSign;
    imu.roll     = imu.rawRoll  - imu.offRoll;
    imu.pitch    = imu.rawPitch - imu.offPitch;
    imu.lastUs   = s.tUs;
    imu.seq++;
}

static void imuCalReset() {
    cal.n = 0; cal.sumR = cal.sumP = 0;
    cal.minR = cal.minP = 1.0e9f; cal.maxR = cal.maxP = -1.0e9f;
}
static void imuCalStep() {
#if IMU_AUTO_CALIBRATE
    if (cal.done) return;
    if (!imuFresh()) { imuCalReset(); return; }
    if (imu.seq == cal.lastSeq) return;
    cal.lastSeq = imu.seq;
    cal.sumR += imu.rawRoll; cal.sumP += imu.rawPitch;
    if (imu.rawRoll  < cal.minR) cal.minR = imu.rawRoll;
    if (imu.rawRoll  > cal.maxR) cal.maxR = imu.rawRoll;
    if (imu.rawPitch < cal.minP) cal.minP = imu.rawPitch;
    if (imu.rawPitch > cal.maxP) cal.maxP = imu.rawPitch;
    if (++cal.n < IMU_CAL_SAMPLES) return;
    if ((cal.maxR - cal.minR) <= IMU_CAL_MAX_SPREAD && (cal.maxP - cal.minP) <= IMU_CAL_MAX_SPREAD) {
        imu.offRoll = cal.sumR / (float)cal.n; imu.offPitch = cal.sumP / (float)cal.n;
        imu.roll = imu.rawRoll - imu.offRoll;  imu.pitch = imu.rawPitch - imu.offPitch;
        cal.done = true;
        logPrintf("IMU kalibrasi OK: offset R=%.2f P=%.2f deg\r\n",
                  (double)(imu.offRoll * kRad2Deg), (double)(imu.offPitch * kRad2Deg));
    } else {
        logPrintf("IMU kalibrasi diulang: robot bergerak/bergetar\r\n");
        imuCalReset();
    }
#endif
}
static void imuCalRestart() { cal.done = false; imu.offRoll = imu.offPitch = 0; imuCalReset(); }

// ============================================================================
//  KESELAMATAN
// ============================================================================
static bool isWalkingMode(Mode m) { return m == Mode::BALANCE || mode_is_gait(m); }
// [V4-SAFE] POSE_HOLD & MIRROR_DBG juga menggerakkan kaki ber-torsi -> ikut dipantau.
static bool fallCheckMode(Mode m) {
    return isWalkingMode(m) || m == Mode::READY || m == Mode::POSE_HOLD || m == Mode::MIRROR_DBG;
}
static uint32_t g_freeTick = 0;

static void enterFallen(const char* why) {
    if (walk.mode == Mode::FALLEN) return;
    walk.mode = Mode::FALLEN;
    walk.fallCt = 0; walk.uprightCt = 0; g_freeTick = 0;
    logPrintf("\r\n!!! %s - SERVO FREE. Tegakkan robot lalu tekan 'r'.\r\n", why);
}
static void startReset(const char* why, int16_t cycles = 80) {
    walk.motCt = cycles;
    walk.mode  = Mode::RESET;
    logPrintf("RESET: %s\r\n", why);
}
static bool manualReset(const char* why) {
    if (walk.mode == Mode::FALLEN) {
        if (!imuFresh() || fabsf(imu.roll) >= g_cfg.rearmAngleRad || fabsf(imu.pitch) >= g_cfg.rearmAngleRad) {
            logPrintf("RESET ditolak: robot FALLEN belum tegak / IMU tidak fresh\r\n");
            return false;
        }
        const uint8_t n = syncStateFromServoFeedback();
        logPrintf("Re-arm sync: %u/22 feedback servo dipakai sebagai baseline\r\n", (unsigned)n);
    }
    startReset(why);
    return true;
}
static void detAng() {
    if (!imuFresh()) {
        if (isWalkingMode(walk.mode)) startReset("data IMU basi");
        return;
    }
    const bool tilted = fabsf(imu.pitch) >= g_cfg.fallAngleRad || fabsf(imu.roll) >= g_cfg.fallAngleRad;
    if (!tilted) { walk.fallCt = 0; return; }
    if (++walk.fallCt >= FALL_CONFIRM_CYCLES) enterFallen("FALL DETECTED");
}
static bool walkStateFinite() {
    const float v[] = { walk.dxi, walk.dyi, walk.dxis, walk.dyis, walk.dxib, walk.dyib, walk.autoH,
                        walk.fh, walk.fhMax, walk.sw, walk.swx, walk.swy, walk.swMax, walk.waist,
                        walk.fwct, walk.fwctEnd, walk.fwctUp };
    for (float x : v) if (!isFiniteVal(x)) return false;
    return true;
}

// ============================================================================
//  UVC & KINEMATIKA (identik V3)
// ============================================================================
static void uvcSub() {
    if (walk.fwct <= walk.landF) {
        const float k = walk.dyi / fmaxf(11.0f - walk.fwct, 1.0f);
        walk.dyi -= k; walk.dyis += k;
        if (walk.dxi > RST_F_LEG)       { walk.dxi -= RST_F_LEG; walk.dxis -= RST_F_LEG; }
        else if (walk.dxi < -RST_F_LEG) { walk.dxi += RST_F_LEG; walk.dxis += RST_F_LEG; }
        else                            { walk.dxis -= walk.dxi; walk.dxi = 0; }
    }
    walk.dyis = clampf(walk.dyis, -70.0f, 70.0f);
    walk.dxis = clampf(walk.dxis, -70.0f, 70.0f);
    if (HEIGHT > walk.autoH) walk.autoH += (HEIGHT - walk.autoH) * 0.07f;
    else                     walk.autoH = HEIGHT;
    const float supportRoll = (walk.jikuasi == 0) ? -imu.roll : imu.roll;
    if (walk.fwct > walk.fwctEnd - walk.landB && supportRoll > 0)
        walk.autoH -= (fabsf(walk.dyi - walk.dyib) + fabsf(walk.dxi - walk.dxib)) * 0.02f;
    walk.autoH = clampf(walk.autoH, 140.0f, HEIGHT);
}
static void uvcSub2() {
    const float den = fmaxf(walk.fwctEnd - walk.fwct + 1.0f, 1.0f);
    const float k1 = walk.dyi / den; walk.dyi -= k1;
    const float k0 = walk.dxi / den; walk.dxi -= k0;
    walk.waist -= walk.waist / den;
    servos.K2[0] = f2i16(walk.waist,  servos.K2[0]);
    servos.K2[1] = f2i16(-walk.waist, servos.K2[1]);
    if (walk.fwct <= walk.landF) { walk.dyis += k1; walk.dxis -= k0; }
    else { walk.dyis -= walk.dyis / den; walk.dxis -= walk.dxis / den; }
    walk.autoH += (HEIGHT - walk.autoH) / den;
    walk.dyis = clampf(walk.dyis, -70.0f, 70.0f);
    walk.dxis = clampf(walk.dxis, -70.0f, 70.0f);
}
static void uvc() {
    const float rollt  = (walk.jikuasi == 0) ? -imu.roll : imu.roll;
    const float pitcht = imu.pitch;
    if (walk.fwct > walk.landF && walk.fwct <= walk.fwctEnd - walk.landB) {
        float k  = atan2f(walk.dyi - walk.sw, walk.autoH);
        float kl = walk.autoH / cosf(k);
        float ks = k + rollt;
        walk.dyi   = kl * sinf(ks) + walk.sw;
        walk.autoH = kl * cosf(ks);
        k  = atan2f(walk.dxi, walk.autoH);
        kl = walk.autoH / cosf(k);
        ks = k + pitcht;
        walk.dxi   = kl * sinf(ks);
        walk.autoH = kl * cosf(ks);
        walk.dyi = clampf(walk.dyi, 0.0f, 45.0f);
        walk.dxi = clampf(walk.dxi, -45.0f, 45.0f);
        walk.dyis = walk.dyi;
        walk.dxis = -walk.dxi;
    }
}
static void footUp() {
    const float span = (float)(walk.fwctEnd - walk.landF - walk.landB);
    if (walk.fwct > walk.landF && walk.fwct <= (walk.fwctEnd - walk.landB) && span >= 1.0f) {
        const float t = (walk.fwct - walk.landF) / span;
        walk.fh = walk.fhMax * 0.5f * (1.0f - cosf(2.0f * kPi * t));
    } else walk.fh = 0;
}
static void swCont() {
    const float k = walk.swMax * sinf(kPi * walk.fwct / fmaxf(walk.fwctEnd, 1.0f));
#if SWCONT_LEGACY
    const float t = atanf((fabsf(walk.dxi) + fabsf(21.5f * sinf(walk.fwct))) /
                          (walk.dyi + 21.5f * cosf(walk.fwct) - walk.fwct));
#else
    const float theta = walk.waist / CHG_SVA;
    const float t = atan2f(fabsf(walk.dxi) + fabsf(21.5f * sinf(theta)), walk.dyi + 21.5f * cosf(theta));
#endif
    walk.swx = (walk.dxi > 0) ? k * sinf(t) : -k * sinf(t);
    walk.swy = k * cosf(t);
}
static void armCont() {
    servos.U1[0] = f2i16(clampf(510.0f * walk.dyis / 70.0f, 0.0f, 1000.0f), servos.U1[0]);
    servos.U1[1] = servos.U1[0];
}
static void footCont(float x, float y, float h, int s) {
    const float temp1 = sqrtf(y * y + h * h) - LEG_OFFSET;
    float k = sqrtf(x * x + temp1 * temp1);
    if (k > OFS_L) k = OFS_L;
    k = fmaxf(k, 1.0f);
    const float x_angle = CHG_SVA * asinf(clampf(x / k, -1.0f, 1.0f));
    float knee = CHG_SVA * acosf(clampf(k / LEG_ACOS_DEN, -1.0f, 1.0f));
    knee = clampf(knee, 0.0f, 1800.0f);
    const float a = KNEE_LPF_ALPHA;
    knee = (servos.H[s] / 2.0f) * (1.0f - a) + knee * a;
    servos.H[s]  = f2i16(knee * 2.0f, servos.H[s]);
    servos.K0[s] = f2i16(knee + x_angle, servos.K0[s]);
    servos.A0[s] = f2i16(knee - x_angle, servos.A0[s]);
    float k1 = CHG_SVA * atan2f(y, fmaxf(h, 1.0f));
    k1 = (float)servos.K1[s] * (1.0f - a) + k1 * a;
    servos.K1[s] = f2i16(k1,  servos.K1[s]);
    servos.A1[s] = f2i16(-k1, servos.A1[s]);
}
static float maxReachHeight(float x, float y) {
    const float xx = clampf(x, -OFS_L, OFS_L);
    const float radial = sqrtf(fmaxf(OFS_L * OFS_L - xx * xx, 0.0f)) + LEG_OFFSET;
    return sqrtf(fmaxf(radial * radial - y * y, 0.0f));
}
static void feetCont1(float x0, float y0, float x1, float y1, int s) {
    float wk = 0;
    if (s == 1) {
        float wt;
        if (walk.jikuasi == 0) { wt = 0.5f * atan2f(x0,  y0 + 21.5f); wk = fabsf(15.0f * x0 / 45.0f); }
        else                   { wt = 0.5f * atan2f(-x1, y1 + 21.5f); wk = fabsf(15.0f * x1 / 45.0f); }
        walk.waist = clampf(wt * CHG_SVA, -HIP_YAW_MAX, HIP_YAW_MAX);
        servos.K2[0] = f2i16(walk.waist,  servos.K2[0]);
        servos.K2[1] = f2i16(-walk.waist, servos.K2[1]);
    }
    const float lim0 = maxReachHeight(x0, y0 - wk) + ((walk.jikuasi == 1) ? walk.fh : 0.0f);
    const float lim1 = maxReachHeight(x1, y1 - wk) + ((walk.jikuasi == 0) ? walk.fh : 0.0f);
    const float safeH = clampf(fminf(lim0, lim1), 140.0f, HEIGHT);
    if (walk.autoH > safeH) walk.autoH = safeH;
    if (walk.jikuasi == 0) {
        footCont(x0, y0 - wk, walk.autoH, 0);
        footCont(x1, y1 - wk, walk.autoH - walk.fh, 1);
    } else {
        footCont(x0, y0 - wk, walk.autoH - walk.fh, 0);
        footCont(x1, y1 - wk, walk.autoH, 1);
    }
}
static void feetCont2(int s) {
    if (walk.jikuasi == 0)
        feetCont1(walk.dxi - walk.swx, walk.dyi - walk.swy, walk.dxis - walk.swx, walk.dyis + walk.swy, s);
    else
        feetCont1(walk.dxis - walk.swx, walk.dyis + walk.swy, walk.dxi - walk.swx, walk.dyi - walk.swy, s);
}
static void counterCont() {
    if (walk.fwct >= walk.fwctEnd) {
        walk.jikuasi ^= 1;
        walk.fwct = 0; walk.fh = 0;
        float k = walk.dyis; walk.dyis = walk.dyi; walk.dyib = walk.dyi; walk.dyi = k;
        k = walk.dxis; walk.dxis = walk.dxi; walk.dxib = walk.dxi; walk.dxi = k;
    } else {
        walk.fwct += walk.fwctUp;
        if (walk.fwct > walk.fwctEnd) walk.fwct = walk.fwctEnd;
    }
}

// ============================================================================
//  STATE MACHINE (identik V3)
// ============================================================================
static void cmdGo() {
    if (walk.mode != Mode::READY)        { logPrintf("g: hanya dari mode 720\r\n"); return; }
    if (!imuFresh())                     { logPrintf("g ditolak: data IMU tidak valid\r\n"); return; }
    if (IMU_AUTO_CALIBRATE && !cal.done) { logPrintf("g ditolak: kalibrasi IMU belum selesai\r\n"); return; }
    walk.fwct = 1; walk.mode = Mode::BALANCE;
    logPrintf("  Go -> mode=730\r\n");
}

static void walkControl() {
    static uint8_t dispDiv = 0;
    switch (walk.mode) {
        case Mode::RESET:
            rampTo(HOME_POSE);
            if (walk.motCt > 0) walk.motCt--;
            else {
                initWalkState();
                walk.jikuasi = 1; footCont(0, 0, HEIGHT, 0);
                walk.jikuasi = 0; footCont(0, 0, HEIGHT, 1);
                walk.mode = Mode::READY;
                logPrintf("mode=720 [Ready]\r\n");
            }
            break;
        case Mode::READY: imuCalStep(); break;
        case Mode::BALANCE:
            feetCont2(1);
            if (imuFresh() && (fabsf(imu.roll) > g_cfg.triggerRollRad || fabsf(imu.pitch) > g_cfg.triggerPitchRad)) {
                walk.jikuasi = (imu.roll > 0) ? 1 : 0;
                walk.fwct = 1; walk.mode = Mode::WALK_INIT;
                logPrintf("mode=740 [Walk Init]\r\n");
            }
            break;
        case Mode::WALK_INIT:
            uvc(); uvcSub(); footUp(); feetCont2(1); armCont(); counterCont();
            if (walk.fwct == 0) { walk.mode = Mode::WALK_HOLD; logPrintf("mode=750\r\n"); }
            break;
        case Mode::WALK_HOLD:
            feetCont2(1);
            if (walk.fwct > 30) {
                walk.fwct = 1;
                const float k = sqrtf(0.5f * walk.dxis * walk.dxis + walk.dyis * walk.dyis);
                walk.swMax = g_cfg.swayBase + g_cfg.swayGain * k / 45.0f;
                walk.mode = Mode::WALK_STEP;
                logPrintf("mode=760 swMax=%.1f\r\n", (double)walk.swMax);
            } else walk.fwct++;
            break;
        case Mode::WALK_STEP:
            walk.landF = 25; walk.fwctEnd = (float)(walk.landF + 25);
            uvcSub2(); footUp(); swCont(); feetCont2(0); armCont(); counterCont();
            if (walk.fwct == 0) {
                walk.landF = walk.landB = 0;
                walk.dxi = walk.dyi = walk.dxis = walk.dyis = walk.dxib = walk.dyib = 0;
                walk.fwctEnd = 18; walk.walkCt = 0; walk.fwctUp = 1; walk.fwct = 1;
                walk.autoH = HEIGHT; walk.sw = walk.swx = walk.swy = 0; walk.jikuasi = 1;
                walk.mode = Mode::WALK_END; logPrintf("mode=770\r\n");
            }
            break;
        case Mode::WALK_END:
            feetCont2(0);
            if (walk.fwct > 50) { walk.fwct = 1; walk.mode = Mode::BALANCE; logPrintf("mode=730\r\n"); }
            else walk.fwct++;
            break;
        case Mode::POSE_HOLD: feetCont2(1); break;
        case Mode::ANGLE_DISP:
            if (++dispDiv >= 5) { dispDiv = 0;
                logPrintf("P:%.2f R:%.2f deg\r\n", (double)(imu.pitch * kRad2Deg), (double)(imu.roll * kRad2Deg)); }
            break;
        case Mode::MIRROR_DBG:
            walk.dxis = -walk.dxi; walk.dyis = walk.dyi; feetCont2(1);
            break;
        case Mode::RAW_ANGLES:
            if (++dispDiv >= 5) { dispDiv = 0;
                logPrintf("R:%d P:%d (x1000 rad)\r\n", (int)(imu.roll * 1000.0f), (int)(imu.pitch * 1000.0f)); }
            break;
        case Mode::FALLEN:
#if AUTO_REARM_AFTER_FALL
            if (imuFresh() && fabsf(imu.roll) < g_cfg.rearmAngleRad && fabsf(imu.pitch) < g_cfg.rearmAngleRad) {
                if (++walk.uprightCt >= REARM_STABLE_CYCLES) {
                    const uint8_t n = syncStateFromServoFeedback();
                    logPrintf("Auto re-arm sync: %u/22 feedback servo\r\n", (unsigned)n);
                    startReset("robot tegak stabil, re-arm otomatis");
                }
            } else walk.uprightCt = 0;
#endif
            break;
    }
}

// ============================================================================
//  PARAMETER & TRIM  [V4-TRIM][V4-SAFE]
// ============================================================================
static void applyRuntimeParam(ParamId id, float v) {
    if (!isFiniteVal(v)) { logPrintf("Parameter ditolak: NaN/Inf\r\n"); return; }
    const bool frameParam = id == ParamId::SIGN_ROLL || id == ParamId::SIGN_PITCH || id == ParamId::IMU_SWAP;
    // Ganti frame IMU saat robot berdiri ber-torsi/berjalan = lonjakan roll/pitch -> tolak.
    if (frameParam && !(walk.mode == Mode::READY || walk.mode == Mode::FALLEN ||
                        walk.mode == Mode::ANGLE_DISP || walk.mode == Mode::RAW_ANGLES)) {
        logPrintf("Parameter frame IMU ditolak: hanya di READY/FALLEN\r\n");
        return;
    }
    RuntimeConfig c = g_cfg;
    switch (id) {
        case ParamId::FALL_DEG:          c.fallAngleRad = v / kRad2Deg; break;
        case ParamId::REARM_DEG:         c.rearmAngleRad = v / kRad2Deg; break;
        case ParamId::IMU_STALE_MS:      c.imuStaleUs = (uint32_t)(clampf(v, 20.0f, 500.0f) * 1000.0f); break;
        case ParamId::SERVO_STEP:        c.servoMaxStep = (int16_t)clampf(v, 20.0f, 1000.0f); break;
        case ParamId::TRIGGER_ROLL_DEG:  c.triggerRollRad = v / kRad2Deg; break;
        case ParamId::TRIGGER_PITCH_DEG: c.triggerPitchRad = v / kRad2Deg; break;
        case ParamId::FOOT_LIFT:         c.footLiftMax = v; break;
        case ParamId::SWAY_BASE:         c.swayBase = v; break;
        case ParamId::SWAY_GAIN:         c.swayGain = v; break;
        case ParamId::SIGN_ROLL:         c.imuRollSign = (v < 0) ? -1 : +1; break;
        case ParamId::SIGN_PITCH:        c.imuPitchSign = (v < 0) ? -1 : +1; break;
        case ParamId::IMU_SWAP:          c.imuSwap = v >= 0.5f; break;
    }
    runtime_config_sanitize(c);
    const bool frameChanged = c.imuRollSign != g_cfg.imuRollSign || c.imuPitchSign != g_cfg.imuPitchSign ||
                              c.imuSwap != g_cfg.imuSwap;
    if (memcmp(&c, &g_cfg, sizeof(c)) == 0) return;
    g_cfg = c;
    walk.fhMax = g_cfg.footLiftMax;
    if (frameChanged) { imuCalRestart(); cal.done = !IMU_AUTO_CALIBRATE; imu.srcSeq = 0; }
    ++g_cfgGen;
}

static bool setTrim(uint8_t bus, uint8_t slot, int32_t value) {
    if (bus > 1 || slot >= SERVO_SLOTS) return false;
    if (walk.mode != Mode::READY) { logPrintf("trim hanya di mode 720\r\n"); return false; }
    const int16_t v = (int16_t)clampi(value, -TRIM_LIMIT, TRIM_LIMIT);
    if (g_trim[bus][slot] != v) { g_trim[bus][slot] = v; ++g_cfgGen; }
    return true;
}

// Peta trim konsol (grup, indeks) -> slot bus. side 0 = kanan (bus R), 1 = kiri (bus L).
static int trimSlot(char grp, uint8_t idx) {
    switch (grp) {
        case 'k': return idx == 0 ? 7 : idx == 1 ? 6 : idx == 2 ? 5 : -1;    // hip pitch/roll/yaw
        case 'a': return idx == 0 ? 9 : idx == 1 ? 10 : -1;                  // ankle pitch/roll
        case 'u': return idx == 0 ? 1 : idx == 1 ? 2 : idx == 2 ? 3 : -1;    // shoulder p/r/y
        case 'h': return idx == 0 ? 8 : -1;                                  // lutut
        case 'e': return idx == 0 ? 4 : -1;                                  // siku
        default:  return -1;
    }
}

// ============================================================================
//  PERINTAH SERIAL (karakter datang dari task konsol lewat queue)
// ============================================================================
enum KeyState : uint8_t { KS_IDLE, KS_PARAM, KS_TRIM };
static bool isDebugSafe() {
    return walk.mode == Mode::READY || walk.mode == Mode::POSE_HOLD || walk.mode == Mode::ANGLE_DISP ||
           walk.mode == Mode::MIRROR_DBG || walk.mode == Mode::RAW_ANGLES;
}
static void printHelp() {
    logPrintf("r reset | g jalan (720->730) | x STOP servo free | c kalibrasi IMU | ? bantuan\r\n");
    logPrintf("t sudut(deg) | y mirror-debug | z sudut x1000 | (t,y,z hanya dari mode 720)\r\n");
    logPrintf("p<0-2>+/- parameter | k/u/a/h/e <r|l|b> <0-2> +/- trim permanen (mode 720) | spasi batal\r\n");
}

static void keyCont() {
    static KeyState st = KS_IDLE;
    static int8_t   paramSel = 0;
    static char     grp = 0;
    static int8_t   side = 0;
    static uint8_t  idx = 0;
    char c;
    for (uint8_t n = 0; n < 16 && xQueueReceive(g_keyQ, &c, 0) == pdTRUE; ++n) {
        if (c == '\r' || c == '\n') continue;
        logPrintf("> %c\r\n", c);
        if (c == ' ') { st = KS_IDLE; continue; }
        if (st == KS_IDLE) {
            switch (c) {
                case 'r': manualReset("serial r"); break;
                case 'g': cmdGo(); break;
                case 'x': enterFallen("STOP (x)"); break;
                case 'c':
                    if (walk.mode == Mode::READY) { imuCalRestart(); logPrintf("Kalibrasi IMU dimulai (jaga robot diam)\r\n"); }
                    else logPrintf("c: hanya dari mode 720\r\n");
                    break;
                case 't': if (isDebugSafe()) { walk.mode = Mode::ANGLE_DISP; logPrintf("  Angle Disp\r\n"); } else logPrintf("t: hanya dari mode 720\r\n"); break;
                case 'y': if (isDebugSafe()) { walk.mode = Mode::MIRROR_DBG; logPrintf("  Debug\r\n"); }   else logPrintf("y: hanya dari mode 720\r\n"); break;
                case 'z': if (isDebugSafe()) { walk.mode = Mode::RAW_ANGLES; logPrintf("  Raw angle\r\n"); } else logPrintf("z: hanya dari mode 720\r\n"); break;
                case 'p': st = KS_PARAM; break;
                case 'k': case 'u': case 'a': case 'h': case 'e':
                    grp = c; side = 0; idx = 0; st = KS_TRIM;
                    logPrintf("trim %c: r/l/b, angka=sendi, +/- (30 hitungan), spasi=keluar\r\n", grp);
                    break;
                case '?': printHelp(); break;
                default: break;
            }
        } else if (st == KS_PARAM) {
            if (c >= '0' && c <= '9') paramSel = (int8_t)(c - '0');
            else if (c == '+' || c == '-') {
                const float d = (c == '+') ? 1.0f : -1.0f;
                switch (paramSel) {
                    case 0: walk.dxi = clampf(walk.dxi + d, -45.0f, 45.0f); break;
                    case 1: walk.dyi = clampf(walk.dyi + d, -45.0f, 45.0f); break;
                    case 2: applyRuntimeParam(ParamId::SWAY_BASE, g_cfg.swayBase + d); break;
                    default: break;
                }
                logPrintf("No:%d dx:%.0f dy:%.0f swayBase:%.1f\r\n", paramSel,
                          (double)walk.dxi, (double)walk.dyi, (double)g_cfg.swayBase);
            }
        } else {
            if (c == 'r') side = 0; else if (c == 'l') side = 1; else if (c == 'b') side = 2;
            else if (c == 'k' || c == 'u' || c == 'a' || c == 'h' || c == 'e') { grp = c; idx = 0; }
            else if (c >= '0' && c <= '9') idx = (uint8_t)(c - '0');
            else if (c == '+' || c == '-') {
                const int sl = trimSlot(grp, idx);
                if (sl < 0) { logPrintf("sendi %c%d tidak ada\r\n", grp, idx); continue; }
                const int16_t d = (c == '+') ? 30 : -30;
                for (uint8_t sd = 0; sd < 2; ++sd) {
                    if (side != 2 && side != (int8_t)sd) continue;
                    setTrim(sd, (uint8_t)sl, (int32_t)g_trim[sd][sl] + d);
                }
                logPrintf("trim %c%d R=%d L=%d (disimpan otomatis)\r\n", grp, idx, g_trim[0][sl], g_trim[1][sl]);
            }
        }
    }
}

static void processRobotCommands() {
    RobotCommand c;
    for (uint8_t n = 0; n < 12 && xQueueReceive(g_cmdQ, &c, 0) == pdTRUE; ++n) {
        switch (c.type) {
            case RobotCmdType::RESET: manualReset("web command"); break;
            case RobotCmdType::GO:    cmdGo(); break;
            case RobotCmdType::STOP:  enterFallen("STOP from web"); break;
            case RobotCmdType::CALIBRATE:
                if (walk.mode == Mode::READY) { imuCalRestart(); logPrintf("Kalibrasi IMU dimulai dari web\r\n"); }
                else logPrintf("Web calibrate ditolak: hanya READY\r\n");
                break;
            case RobotCmdType::SET_PARAM: applyRuntimeParam(c.param, c.value); break;
            case RobotCmdType::SET_TRIM:  setTrim(c.bus, c.slot, (int32_t)lroundf(clampf(c.value, -30000.0f, 30000.0f))); break;
        }
    }
}

// ============================================================================
//  TELEMETRY (spinlock; salinan ~800 B ~ 2 us)
// ============================================================================
static uint32_t g_loopAvgUs = 0, g_loopMaxUs = 0, g_loopOverruns = 0;
static TelemetrySnapshot g_tele{};
static portMUX_TYPE g_teleMux = portMUX_INITIALIZER_UNLOCKED;

static void publishTelemetry() {
    static int64_t lastPubUs = 0;
    const int64_t nowUs = esp_timer_get_time();
    if (nowUs - lastPubUs < (int64_t)TELE_PUBLISH_MS * 1000) return;
    lastPubUs = nowUs;

    static TelemetrySnapshot t;       // static: jangan taruh ~800 B di stack tiap siklus
    t.seq = g_tele.seq + 1; t.ms = (uint32_t)(nowUs / 1000); t.mode = walk.mode;
    t.roll = imu.roll; t.pitch = imu.pitch; t.rawRoll = imu.rawRoll; t.rawPitch = imu.rawPitch;
    t.offRoll = imu.offRoll; t.offPitch = imu.offPitch;
    t.imuAgeMs = imu.lastUs ? (int32_t)((nowUs - imu.lastUs) / 1000) : -1;
    t.imuHz = imu_rate_hz();
    t.imuFreshFlag = imuFresh(); t.imuInit = imu.initOk; t.calDone = cal.done;
    t.fwct = walk.fwct; t.fwctEnd = walk.fwctEnd; t.dxi = walk.dxi; t.dyi = walk.dyi;
    t.dxis = walk.dxis; t.dyis = walk.dyis; t.autoH = walk.autoH; t.fh = walk.fh;
    t.swx = walk.swx; t.swy = walk.swy; t.swMax = walk.swMax; t.waist = walk.waist;
    t.jikuasi = walk.jikuasi; t.kin = servos; t.cfg = g_cfg; t.cfgGen = g_cfgGen;
    t.loopAvgUs = g_loopAvgUs; t.loopMaxUs = g_loopMaxUs; t.loopOverruns = g_loopOverruns;
    t.servoPhaseUs = g_servoPhaseUs;
    t.ctrlStackFree = (uint32_t)uxTaskGetStackHighWaterMark(nullptr);
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s) {
        const ServoFeedback& fb = g_servoFb[b][s];
        t.cmd[b][s] = g_lastOut[b][s]; t.fb[b][s] = fb.pos; t.trim[b][s] = g_trim[b][s];
        t.fbValid[b][s] = fb.valid ? 1 : 0; t.fbMiss[b][s] = fb.missCount;
        const int64_t age = fb.lastUs ? (nowUs - fb.lastUs) / 1000 : 65535;
        t.fbAgeMs[b][s] = (uint16_t)(age > 65535 ? 65535 : age);
    }
    portENTER_CRITICAL(&g_teleMux);
    g_tele = t;
    portEXIT_CRITICAL(&g_teleMux);
}

TelemetrySnapshot telemetry_copy() {
    TelemetrySnapshot t;
    portENTER_CRITICAL(&g_teleMux);
    t = g_tele;
    portEXIT_CRITICAL(&g_teleMux);
    return t;
}

// ============================================================================
//  SIKLUS KONTROL
// ============================================================================
static void housekeeping() {
    const uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
    const bool on = (walk.mode == Mode::FALLEN) ? ((ms / 100) & 1) : ((ms / 500) & 1);
    gpio_set_level((gpio_num_t)PIN_LED1, on ? 1 : 0);
    static uint32_t lastLog = 0;
    if (ms - lastLog >= TELE_LOG_MS) {
        lastLog = ms;
        const int age = imu.lastUs ? (int)((esp_timer_get_time() - imu.lastUs) / 1000) : -1;
        logPrintf("TELE m=%u R=%.1f P=%.1f fwct=%.0f jiku=%d | loop avg=%uus max=%uus ics=%uus ovr=%u | imu=%dms %uHz cal=%d\r\n",
                  (unsigned)walk.mode, (double)(imu.roll * kRad2Deg), (double)(imu.pitch * kRad2Deg),
                  (double)walk.fwct, (int)walk.jikuasi, (unsigned)g_loopAvgUs, (unsigned)g_loopMaxUs,
                  (unsigned)g_servoPhaseUs, (unsigned)g_loopOverruns, age, (unsigned)imu_rate_hz(), (int)cal.done);
        g_loopMaxUs = 0;
    }
}

static void controlStep() {
    const int64_t t0 = esp_timer_get_time();
    gpio_set_level((gpio_num_t)PIN_T2, 1);
    imuPull();
    keyCont();
    processRobotCommands();
    if (fallCheckMode(walk.mode)) detAng();
    walkControl();
    if (!walkStateFinite()) { initWalkState(); startReset("nilai NaN/Inf pada state jalan"); }

    if (walk.mode == Mode::FALLEN) {
        if ((g_freeTick++ % 10) == 0) sendAllServoFree();     // ~10 Hz + feedback posisi
    } else {
        g_freeTick = 0;
        sendAllServoPositions();
    }
    housekeeping();
    gpio_set_level((gpio_num_t)PIN_T2, 0);
    const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    g_loopAvgUs = (g_loopAvgUs * 15u + dt) / 16u;
    if (dt > g_loopMaxUs) g_loopMaxUs = dt;
    publishTelemetry();
}

// [V4-BOOT] ICS servo menyala dalam keadaan FREE. Baca posisi aktual dengan frame FREE,
// jadikan baseline, lalu ramp ke HOME. V3 mengasumsikan servo sudah di HOME sehingga
// frame pertama bisa melompat ke HOME dengan kecepatan penuh servo.
static void bootSync() {
#if BOOT_SYNC_FROM_FEEDBACK
    for (int i = 0; i < BOOT_SYNC_CYCLES; ++i) { sendAllServoFree(); vTaskDelay(pdMS_TO_TICKS(10)); }
    const uint8_t n = syncStateFromServoFeedback();
    logPrintf("Boot sync: %u/22 posisi servo terbaca%s\r\n", (unsigned)n,
              n < 22 ? " (sisanya diasumsikan HOME - cek jalur RX/EN!)" : "");
#endif
    startReset("boot", 100);
}

static void controlTask(void*) {
    if (!icsInit()) logPrintf("GAGAL inisialisasi UART ICS!\r\n");   // ISR UART ikut di core1
    bootSync();
    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    const int64_t periodUs = (int64_t)CONTROL_PERIOD_MS * 1000;
    for (;;) {
        const int64_t t0 = esp_timer_get_time();
        controlStep();
        if (esp_timer_get_time() - t0 >= periodUs) {
            ++g_loopOverruns;
            lastWake = xTaskGetTickCount();          // jangan mengejar dengan burst
            vTaskDelay(1);
        } else {
            xTaskDelayUntil(&lastWake, period);
        }
    }
}

// ============================================================================
//  API PUBLIK
// ============================================================================
bool control_queue_cmd(const RobotCommand& c) {
    return g_cmdQ && xQueueSend(g_cmdQ, &c, 0) == pdTRUE;
}
bool control_push_key(char c) {
    return g_keyQ && xQueueSend(g_keyQ, &c, 0) == pdTRUE;
}

void control_init(const RuntimeConfig& cfg, const int16_t trim[2][SERVO_SLOTS]) {
    g_cfg = cfg;
    runtime_config_sanitize(g_cfg);
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        g_trim[b][s] = (int16_t)clampi(trim[b][s], -TRIM_LIMIT, TRIM_LIMIT);

    walk = WalkingState();
    imu = ImuData();
    servos = HOME_POSE;
    walk.mode = Mode::RESET; walk.motCt = 100;
    walk.fhMax = g_cfg.footLiftMax; walk.swMax = 25; walk.fwctUp = 1;
    initWalkState();
    primeOutputs();
    imuCalReset();
    cal.done = !IMU_AUTO_CALIBRATE; cal.lastSeq = 0;

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_LED1) | (1ULL << PIN_T2);
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level((gpio_num_t)PIN_LED1, 0);
    gpio_set_level((gpio_num_t)PIN_T2, 0);

    g_cmdQ = xQueueCreate(32, sizeof(RobotCommand));
    g_keyQ = xQueueCreate(64, sizeof(char));
}

bool control_start() {
    if (!g_cmdQ || !g_keyQ) { logPrintf("GAGAL membuat queue kontrol!\r\n"); return false; }
    return xTaskCreatePinnedToCore(controlTask, "WalkCtrl", STACK_CONTROL, nullptr, PRIO_CONTROL,
                                   &g_ctrlTask, CORE_CONTROL) == pdPASS;
}

#ifdef HOST_TEST
// Akses internal untuk simulasi host (tidak dikompilasi di firmware).
void host_control_step() { controlStep(); }
void host_inject_icsInit() { icsInit(); }
#endif
#ifdef HOST_TEST
void host_boot() { icsInit(); bootSync(); }
#endif
#ifdef HOST_TEST
void legacy_out(int16_t o[2][SERVO_SLOTS], uint16_t* m, ServoPositions* k, float* roll, float* pitch) {
    memcpy(o, g_lastOut, sizeof(g_lastOut)); *m = (uint16_t)walk.mode; *k = servos; *roll = imu.roll; *pitch = imu.pitch;
}
#endif
