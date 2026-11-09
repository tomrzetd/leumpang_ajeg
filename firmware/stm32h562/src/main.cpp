// =============================================================================
//  Leumpang Ajeg - firmware STM32H562 (bare-metal, tanpa RTOS)
//
//  Model eksekusi (prioritas NVIC, angka kecil = lebih tinggi):
//    SysTick (0)  HAL tick
//    TIM2    (1)  overflow jam mikrodetik 64-bit
//    USB     (3)  TinyUSB (hanya menandai event; diproses di main loop)
//    TIM7    (5)  500 Hz: baca ICM-20948 + Mahony
//    TIM6    (6)  100 Hz: la::Controller::step + bus ICS (polling USART, FIFO 8 B)
//    main loop    tud_task, SerialLink (konsol + LAP via USB CDC), SD (blackbox,
//                 motion, config), heartbeat
//  Pertukaran data lintas konteks: SpscRing (lock-free) & MotionStore (atomic).
//  Telemetry dibaca main dengan TIM6 di-mask beberapa mikrodetik.
// =============================================================================
#include "board.h"
#include "app.h"
#include "tusb.h"
#include "leumpang/controller.h"
#include "leumpang/lap_serial.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {

struct LogLine { char t[160]; };

la::Controller                g_ctrl;
Stm32IcsBus                   g_bus;
la::BlackboxRing              g_bb;
la::SpscRing<la::Command, 32> g_cmdQ;       // main -> ISR kontrol
la::SpscRing<char, 64>        g_keyQ;       // main -> ISR kontrol
la::SpscRing<LogLine, 16>     g_isrLog;     // ISR kontrol -> main
la::Telemetry                 g_tele{};
la::lap::SerialLink           g_link;
volatile bool                 g_booted = false;
volatile uint8_t              g_bootCycles = 0;
volatile uint8_t              g_pubDiv = 0;
uint32_t                      g_savedGen = 0;

struct RingInbox : la::Inbox {
    bool nextKey(char& c) override { return g_keyQ.pop(c); }
    bool nextCommand(la::Command& c) override { return g_cmdQ.pop(c); }
} g_inbox;

int64_t platNow(void*) { return la_micros(); }
void platLog(void*, const char* line) {           // dipanggil dari ISR kontrol
    LogLine l;
    strncpy(l.t, line, sizeof(l.t) - 1); l.t[sizeof(l.t) - 1] = 0;
    g_isrLog.push(l);
}

void teleCopy(la::Telemetry& t) {
    NVIC_DisableIRQ(TIM6_IRQn);
    t = g_tele;
    NVIC_EnableIRQ(TIM6_IRQn);
}

// ---- callback LAP / konsol (main loop) ----
void ioWrite(void*, const uint8_t* d, size_t n) {
    if (!tud_cdc_connected()) return;
    while (n) {
        const uint32_t w = tud_cdc_write(d, (uint32_t)n);
        if (!w) { tud_cdc_write_flush(); tud_task(); if (!tud_cdc_write_available()) return; continue; }
        d += w; n -= w;
    }
}
void ioKey(void*, char c) { g_keyQ.push(c); }
bool epSubmit(void*, const la::Command& c) { return g_cmdQ.push(c); }
void epSnapshot(void*, la::Telemetry& t) { teleCopy(t); }
uint32_t epSaved(void*) { return g_savedGen; }
void epMotionSaved(void*, int slot, const uint8_t* d, size_t n) { storage_save_motion(slot, d, n); }

void gpioOut(GPIO_TypeDef* port, uint16_t pin) {
    GPIO_InitTypeDef g = {};
    g.Pin = pin; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

void usbInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();
    GPIO_InitTypeDef g = {};
    g.Pin = GPIO_PIN_11 | GPIO_PIN_12; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, PRIO_USB, 0);
    tusb_rhport_init_t rh = {};
    rh.role = TUSB_ROLE_DEVICE;
    rh.speed = TUSB_SPEED_FULL;
    tusb_init(0, &rh);
}

void timerStart(TIM_TypeDef* t, IRQn_Type irq, uint32_t prio, uint32_t periodUs) {
    t->CR1 = 0;
    t->PSC = (uint32_t)(HAL_RCC_GetPCLK1Freq() / 1000000u) - 1u;
    t->ARR = periodUs - 1u;
    t->EGR = TIM_EGR_UG;
    t->SR = 0;
    t->DIER = TIM_DIER_UIE;
    HAL_NVIC_SetPriority(irq, prio, 0);
    HAL_NVIC_EnableIRQ(irq);
    t->CR1 = TIM_CR1_CEN;
}

}  // namespace

void la_log_main(const char* fmt, ...) {
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_link.logLine(buf);
}

// ---- ISR ----------------------------------------------------------------
extern "C" void la_tim7_isr(void) { imu_isr_tick(); }

extern "C" void la_tim6_isr(void) {
    const int64_t t0 = la_micros();
    T2_PORT->BSRR = T2_PIN;
    if (!g_booted) {                      // boot: baca posisi aktual dengan frame FREE (120 ms)
        g_ctrl.bootFreeCycle(g_bus);
        if (++g_bootCycles >= la::BOOT_SYNC_CYCLES) { g_ctrl.bootFinish(true); g_booted = true; }
    } else {
        la::ImuInput in{};
        imu_latest(in);
        g_ctrl.step(in, g_bus, g_inbox);
    }
    if (++g_pubDiv >= 5) { g_pubDiv = 0; g_ctrl.snapshot(g_tele); }     // telemetry 20 Hz
    T2_PORT->BRR = T2_PIN;
    if (la_micros() - t0 >= (int64_t)la::CONTROL_PERIOD_MS * 1000) g_ctrl.noteOverrun();
}

// ---- main ---------------------------------------------------------------
int main() {
    HAL_Init();
    la_clock_init();
    la_timebase_init();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpioOut(LED_PORT, LED_PIN);
    gpioOut(T2_PORT, T2_PIN);

    usbInit();
    la::lap::Hello self;
    self.role = la::lap::ROLE_ROBOT; self.board = la::lap::BOARD_STM32H562;
    self.fwMajor = la::CORE_VER_MAJOR; self.fwMinor = la::CORE_VER_MINOR; self.fwPatch = la::CORE_VER_PATCH;
    strncpy(self.name, LA_BOARD_NAME, 16);
    la::lap::LapEndpoint::Host h;
    h.submit = epSubmit; h.snapshot = epSnapshot; h.savedGen = epSaved;
    h.motions = &g_ctrl.motions(); h.motionSaved = epMotionSaved;
    h.reboot = [](void*) { NVIC_SystemReset(); };
    la::lap::SerialLink::Io io; io.write = ioWrite; io.key = ioKey;

    // SD dulu: config & motion tersimpan di kartu (STM32 ini tidak memakai flash internal untuk data)
    g_link.init(io, h, self);                       // log tersedia sejak awal (USB mungkin belum enumerasi)
    const bool sd = storage_mount();
    h.files = storage_files();                      // FatFs; dipanggil dari main loop seperti operasi SD lain
    self.caps = la::lap::CAP_USB_CDC | la::lap::CAP_MOTION | la::lap::CAP_GAIT_INTENT | la::lap::CAP_LIVE_POSE | la::lap::CAP_PERCEPTION | la::lap::CAP_SERVO_TOOL |
                (sd ? (uint32_t)(la::lap::CAP_SD | la::lap::CAP_FILES | la::lap::CAP_PROFILE) : 0u);
    g_link.init(io, h, self);                       // ulang dengan caps final
    la_log_main("\r\n=== Leumpang Ajeg (%s) core %u.%u.%u - ketik '?' ===\r\n", LA_BOARD_NAME,
                la::CORE_VER_MAJOR, la::CORE_VER_MINOR, la::CORE_VER_PATCH);

    la::RuntimeConfig cfg = la::runtimeConfigDefault();
    int16_t trim[2][la::SERVO_SLOTS] = {};
    la_log_main(storage_load_cfg(&cfg, trim) ? "SD: konfigurasi + trim dimuat\r\n" : "Konfigurasi default\r\n");
    la::Platform pf{ platNow, platLog, nullptr };
    la::Controller::Options opt;
    opt.profile = storage_profile();                         // nullptr = bawaan KHR-3HV
    g_ctrl.init(pf, cfg, trim, opt);
    if (sd) { storage_begin(g_ctrl.motions(), &g_bb); g_ctrl.setBlackbox(&g_bb); }

    const bool imuOk = imu_init();
    g_bus.init();
    if (imuOk) { __HAL_RCC_TIM7_CLK_ENABLE(); timerStart(TIM7, TIM7_IRQn, PRIO_IMU, 2000); }      // 500 Hz
    __HAL_RCC_TIM6_CLK_ENABLE();
    timerStart(TIM6, TIM6_IRQn, PRIO_CONTROL, la::CONTROL_PERIOD_MS * 1000);                       // 100 Hz

    uint8_t rx[64];
    uint32_t lastCfgCheck = 0, cfgStableSince = 0, lastGen = 0;
    for (;;) {
        tud_task();
        const uint32_t ms = HAL_GetTick();
        while (tud_cdc_available()) {
            const uint32_t n = tud_cdc_read(rx, sizeof(rx));
            if (!n) break;
            g_link.onRx(rx, n, ms);
        }
        LogLine l;
        for (int i = 0; i < 8 && g_isrLog.pop(l); ++i) g_link.logLine(l.t);
        g_link.poll(ms);
        tud_cdc_write_flush();
        storage_service(ms);

        // simpan config/trim ke SD setelah stabil 1 s (SDMMC tidak mengganggu ISR kontrol)
        if (ms - lastCfgCheck >= 250) {
            lastCfgCheck = ms;
            static la::Telemetry t;
            teleCopy(t);
            if (t.cfgGen != lastGen) { lastGen = t.cfgGen; cfgStableSince = ms; }
            if (t.cfgGen != g_savedGen && ms - cfgStableSince >= 1000 && storage_ok()) {
                if (storage_save_cfg(t.cfg, t.trim)) { g_savedGen = t.cfgGen; la_log_main("SD: konfigurasi + trim disimpan\r\n"); }
            }
        }
        const bool fallen = g_booted && g_tele.mode == la::Mode::FALLEN;
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, ((ms / (fallen ? 100 : 500)) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
