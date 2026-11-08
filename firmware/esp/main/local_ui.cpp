// =============================================================================
//  UI lokal (core0): ST7789 via esp_lcd + SPI DMA, WS2812 via RMT
//  [V4-UI] V3 (Adafruit_GFX) menggambar piksel/garis lewat banyak transaksi SPI kecil
//  yang di-polling CPU. V4 merender tiap region ke canvas RAM internal lalu mengirim
//  satu transfer DMA per region: CPU core0 bebas selama transfer.
// =============================================================================
#include "shared.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/rmt_tx.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "font5x7.h"

namespace {

// ---- Warna RGB565 ---------------------------------------------------------
constexpr uint16_t UI_BG = 0x0000, UI_GREEN = 0x07E0, UI_GREEN2 = 0x05A0, UI_DIM = 0x02A0;
constexpr uint16_t UI_YELLOW = 0xFFE0, UI_RED = 0xF800, UI_CYAN = 0x07FF, UI_HDR = 0x0140;

constexpr int CANVAS_PIXELS = 320 * 75;           // 48 KB DMA RAM internal

// ============================================================================
//  Canvas: semua primitif memakai koordinat layar absolut dan di-clip ke region.
// ============================================================================
class Canvas {
public:
    uint16_t* px = nullptr;
    int16_t ox = 0, oy = 0, w = 0, h = 0;

    bool alloc() {
        px = (uint16_t*)heap_caps_malloc(CANVAS_PIXELS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        return px != nullptr;
    }
    void begin(int16_t x, int16_t y, int16_t ww, int16_t hh, uint16_t bg) {
        ox = x; oy = y; w = ww; h = hh;
        const uint16_t c = sw(bg);
        for (int i = 0; i < w * h; ++i) px[i] = c;
    }
    inline void pixel(int x, int y, uint16_t c) {
        x -= ox; y -= oy;
        if ((unsigned)x < (unsigned)w && (unsigned)y < (unsigned)h) px[y * w + x] = sw(c);
    }
    void fillRect(int x, int y, int ww, int hh, uint16_t c) {
        int x0 = x - ox, y0 = y - oy, x1 = x0 + ww, y1 = y0 + hh;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > w) x1 = w;
        if (y1 > h) y1 = h;
        const uint16_t s = sw(c);
        for (int yy = y0; yy < y1; ++yy) for (int xx = x0; xx < x1; ++xx) px[yy * w + xx] = s;
    }
    void hline(int x, int y, int len, uint16_t c) { fillRect(x, y, len, 1, c); }
    void vline(int x, int y, int len, uint16_t c) { fillRect(x, y, 1, len, c); }
    void rect(int x, int y, int ww, int hh, uint16_t c) {
        hline(x, y, ww, c); hline(x, y + hh - 1, ww, c); vline(x, y, hh, c); vline(x + ww - 1, y, hh, c);
    }
    void line(int x0, int y0, int x1, int y1, uint16_t c) {
        int dx = x1 > x0 ? x1 - x0 : x0 - x1, sx = x0 < x1 ? 1 : -1;
        int dy = y1 > y0 ? y0 - y1 : y1 - y0, sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        for (int guard = 0; guard < 2000; ++guard) {
            pixel(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;
            const int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    void circle(int cx, int cy, int r, uint16_t c) {
        int x = r, y = 0, e = 1 - r;
        while (x >= y) {
            pixel(cx + x, cy + y, c); pixel(cx + y, cy + x, c); pixel(cx - y, cy + x, c); pixel(cx - x, cy + y, c);
            pixel(cx - x, cy - y, c); pixel(cx - y, cy - x, c); pixel(cx + y, cy - x, c); pixel(cx + x, cy - y, c);
            ++y;
            if (e < 0) e += 2 * y + 1; else { --x; e += 2 * (y - x) + 1; }
        }
    }
    // Teks 5x7 dalam sel 6x8, skala s. Mengembalikan x berikutnya.
    int text(int x, int y, const char* str, uint16_t c, int s = 1) {
        for (const char* p = str; *p; ++p) {
            unsigned ch = (unsigned char)*p;
            if (ch < 0x20 || ch > 0x7F) ch = '?';
            const uint8_t* g = FONT5X7[ch - 0x20];
            for (int col = 0; col < 5; ++col)
                for (int row = 0; row < 7; ++row)
                    if (g[col] & (1u << row)) {
                        if (s == 1) pixel(x + col, y + row, c);
                        else fillRect(x + col * s, y + row * s, s, s, c);
                    }
            x += 6 * s;
        }
        return x;
    }
private:
    static inline uint16_t sw(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }   // SPI = big-endian
};

Canvas s_cv;
esp_lcd_panel_io_handle_t s_io = nullptr;
esp_lcd_panel_handle_t    s_panel = nullptr;
SemaphoreHandle_t         s_dmaDone = nullptr;
bool                      s_lcdOk = false;
uint32_t                  s_uiFrameUs = 0;

bool IRAM_ATTR onColorDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void*) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dmaDone, &woken);
    return woken == pdTRUE;
}

void flush() {
    if (!s_lcdOk) return;
    esp_lcd_panel_draw_bitmap(s_panel, s_cv.ox, s_cv.oy, s_cv.ox + s_cv.w, s_cv.oy + s_cv.h, s_cv.px);
    xSemaphoreTake(s_dmaDone, pdMS_TO_TICKS(200));          // canvas dipakai ulang -> tunggu DMA
}

bool lcdInit() {
    s_dmaDone = xSemaphoreCreateBinary();
    if (!s_dmaDone || !s_cv.alloc()) { logPrintf("LCD: alokasi DMA RAM gagal\r\n"); return false; }
    spi_bus_config_t bus = {};
    bus.sclk_io_num = LCD_SCK; bus.mosi_io_num = LCD_MOSI; bus.miso_io_num = -1;
    bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
    bus.max_transfer_sz = CANVAS_PIXELS * 2 + 16;
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    esp_lcd_panel_io_spi_config_t io = {};
    io.cs_gpio_num = LCD_CS; io.dc_gpio_num = LCD_DC;
    io.spi_mode = 0; io.pclk_hz = LCD_PCLK_HZ; io.trans_queue_depth = 4;
    io.on_color_trans_done = onColorDone; io.user_ctx = nullptr;
    io.lcd_cmd_bits = 8; io.lcd_param_bits = 8;
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io, &s_io) != ESP_OK) return false;

    esp_lcd_panel_dev_config_t pc = {};
    pc.reset_gpio_num = LCD_RST;
    pc.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    pc.bits_per_pixel = 16;
    if (esp_lcd_new_panel_st7789(s_io, &pc, &s_panel) != ESP_OK) return false;
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, LCD_INVERT);
    esp_lcd_panel_swap_xy(s_panel, LCD_SWAP_XY);
    esp_lcd_panel_mirror(s_panel, LCD_MIRROR_X, LCD_MIRROR_Y);
    esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_disp_on_off(s_panel, true);
    s_lcdOk = true;
    return true;
}

// ============================================================================
//  WS2812 via RMT (tanpa komponen eksternal)
// ============================================================================
rmt_channel_handle_t s_rmt = nullptr;
rmt_encoder_handle_t s_rmtEnc = nullptr;

bool rgbInit() {
    rmt_tx_channel_config_t tc = {};
    tc.gpio_num = (gpio_num_t)PIN_RGB;
    tc.clk_src = RMT_CLK_SRC_DEFAULT;
    tc.resolution_hz = 10 * 1000 * 1000;                     // 0,1 us/tick
    tc.mem_block_symbols = 48;
    tc.trans_queue_depth = 2;
    if (rmt_new_tx_channel(&tc, &s_rmt) != ESP_OK) return false;
    rmt_bytes_encoder_config_t ec = {};
    ec.bit0.level0 = 1; ec.bit0.duration0 = 4; ec.bit0.level1 = 0; ec.bit0.duration1 = 8;   // 0,4/0,8 us
    ec.bit1.level0 = 1; ec.bit1.duration0 = 8; ec.bit1.level1 = 0; ec.bit1.duration1 = 4;   // 0,8/0,4 us
    ec.flags.msb_first = 1;
    if (rmt_new_bytes_encoder(&ec, &s_rmtEnc) != ESP_OK) return false;
    return rmt_enable(s_rmt) == ESP_OK;
}

void rgbSet(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_rmt) return;
    static uint8_t grb[3];
    grb[0] = (uint8_t)((g * RGB_BRIGHTNESS) / 255);
    grb[1] = (uint8_t)((r * RGB_BRIGHTNESS) / 255);
    grb[2] = (uint8_t)((b * RGB_BRIGHTNESS) / 255);
    rmt_transmit_config_t t = {};
    t.loop_count = 0;
    rmt_transmit(s_rmt, s_rmtEnc, grb, sizeof(grb), &t);
    rmt_tx_wait_all_done(s_rmt, 10);
}

void updateRgb(const TelemetrySnapshot& t, const NetStatus& n, uint32_t ms) {
    const bool blink = ((ms / 180u) & 1u) == 0u;
    if (t.seq == 0)                         { const uint8_t v = ((ms / 250u) & 1u) ? 20 : 120; rgbSet(v, v, v); }
    else if (t.mode == Mode::FALLEN)        rgbSet(blink ? 255 : 20, 0, 0);
    else if (!t.imuInit || !t.imuFreshFlag) rgbSet(blink ? 255 : 20, blink ? 20 : 0, 0);
    else if (!t.calDone)                    { const uint8_t v = ((ms / 250u) & 1u) ? 180 : 45; rgbSet(v, v, 0); }
    else if (t.mode == Mode::RESET)         rgbSet(255, 70, 0);
    else if (t.mode == Mode::READY)         { if (n.staConnected || n.apClients) rgbSet(0, 210, 90); else rgbSet(0, 220, 30); }
    else if (t.mode == Mode::BALANCE)       rgbSet(0, 70, 255);
    else if (mode_is_gait(t.mode))          rgbSet(190, 0, 255);
    else                                    rgbSet(0, 160, 160);
}

// ============================================================================
//  Layout (sama dengan V3)
// ============================================================================
void drawStatic(Canvas& c) {
    c.rect(1, 1, 318, 238, UI_DIM);
    c.fillRect(2, 2, 316, 20, UI_HDR);
    c.text(7, 5, "LEUMPANG AJEG", UI_GREEN, 2);
    c.text(229, 8, LA_BOARD_ID == 2 ? "ESP32-P4" : "ESP32-S3", UI_CYAN);
    c.rect(4, 27, 151, 177, UI_DIM);
    c.rect(160, 27, 156, 177, UI_DIM);
    c.hline(161, 47, 154, UI_DIM);
    c.vline(238, 48, 155, UI_DIM);
    c.text(8, 30, "TERMINAL STATUS", UI_GREEN2);
    c.text(165, 30, "POSE ESTIMATOR", UI_GREEN2);
    c.text(176, 39, "SIDE", UI_GREEN2);
    c.text(263, 39, "FRONT", UI_GREEN2);
    c.rect(4, 208, 312, 27, UI_DIM);
    c.text(8, 212, "NET/ICS", UI_DIM);
}
void staticScreen() {
    for (int y = 0; y < LCD_H; y += 75) {
        const int hh = (LCD_H - y) < 75 ? (LCD_H - y) : 75;
        s_cv.begin(0, (int16_t)y, LCD_W, (int16_t)hh, UI_BG);
        drawStatic(s_cv);
        flush();
    }
}

void line(Canvas& c, int y, const char* key, const char* val, uint16_t col) {
    const int x = c.text(8, y + 2, key, UI_DIM);
    c.text(x, y + 2, val, col);
}

uint8_t fbOkCount(const TelemetrySnapshot& t) {
    uint8_t n = 0;
    for (uint8_t b = 0; b < 2; ++b) for (uint8_t s = 0; s < SERVO_SLOTS; ++s)
        if (t.fbValid[b][s] && t.fbAgeMs[b][s] < 700) ++n;
    return n;
}

void drawStatus(const TelemetrySnapshot& t, const NetStatus& n) {
    char buf[40];
    s_cv.begin(5, 48, 149, 155, UI_BG);
    Canvas& c = s_cv;
    uint16_t mc = UI_GREEN;
    if (t.mode == Mode::FALLEN) mc = UI_RED; else if (mode_is_gait(t.mode)) mc = UI_CYAN; else if (t.mode == Mode::RESET) mc = UI_YELLOW;
    line(c, 51, "MODE  : ", t.seq ? mode_short_name(t.mode) : "BOOT/WAIT", t.seq ? mc : UI_YELLOW);
    snprintf(buf, sizeof(buf), "%+.2f\x7F", (double)(t.roll * kRad2Deg));
    line(c, 66, "ROLL  : ", buf, fabsf(t.roll) > t.cfg.fallAngleRad * 0.75f ? UI_YELLOW : UI_GREEN);
    snprintf(buf, sizeof(buf), "%+.2f\x7F", (double)(t.pitch * kRad2Deg));
    line(c, 81, "PITCH : ", buf, fabsf(t.pitch) > t.cfg.fallAngleRad * 0.75f ? UI_YELLOW : UI_GREEN);
    snprintf(buf, sizeof(buf), "%s %ldms %uHz", t.imuFreshFlag ? "OK" : "STALE", (long)t.imuAgeMs, (unsigned)t.imuHz);
    line(c, 96, "IMU   : ", buf, t.imuFreshFlag ? UI_GREEN : UI_RED);
    const uint8_t fb = fbOkCount(t);
    snprintf(buf, sizeof(buf), "%u/22 FB", (unsigned)fb);
    line(c, 111, "SERVO : ", buf, fb >= 20 ? UI_GREEN : (fb ? UI_YELLOW : UI_RED));
    snprintf(buf, sizeof(buf), "%lu/%luus", (unsigned long)t.loopAvgUs, (unsigned long)t.loopMaxUs);
    line(c, 126, "LOOP  : ", buf, t.loopMaxUs < CONTROL_PERIOD_MS * 1000u ? UI_GREEN : UI_RED);
    snprintf(buf, sizeof(buf), "%s AP:%u", n.staConnected ? "STA+AP" : "AP", (unsigned)n.apClients);
    line(c, 141, "WIFI  : ", buf, UI_CYAN);
    snprintf(buf, sizeof(buf), "FW:%4.0f H:%3.0f", (double)t.fwct, (double)t.autoH);
    line(c, 156, "GAIT  : ", buf, UI_GREEN2);
    snprintf(buf, sizeof(buf), "ICS %luus", (unsigned long)t.servoPhaseUs);
    line(c, 171, "BUS   : ", buf, UI_GREEN2);
    snprintf(buf, sizeof(buf), "UI %luus", (unsigned long)s_uiFrameUs);
    line(c, 186, "LOCAL : ", buf, s_uiFrameUs < 50000u ? UI_GREEN2 : UI_YELLOW);
    flush();
}

struct Foot { float x, y, z; };
void feet(const TelemetrySnapshot& t, Foot& r, Foot& l) {
    if (t.jikuasi == 0) { r = {t.dxi - t.swx, t.dyi - t.swy, t.autoH}; l = {t.dxis - t.swx, t.dyis + t.swy, t.autoH - t.fh}; }
    else                { r = {t.dxis - t.swx, t.dyis + t.swy, t.autoH - t.fh}; l = {t.dxi - t.swx, t.dyi - t.swy, t.autoH}; }
}
int ci(float v, int lo, int hi) { return (int)clampi((int32_t)lroundf(isFiniteVal(v) ? v : 0.0f), lo, hi); }

void sideLeg(Canvas& c, int hx, int hy, const Foot& f, int bias, float sc, uint16_t col) {
    const int fx = ci((float)hx + f.x * sc, hx - 34, hx + 34), fy = ci((float)hy + f.z * sc, hy + 20, 195);
    const int kx = (hx + fx) / 2 + bias, ky = (hy + fy) / 2;
    c.line(hx, hy, kx, ky, col); c.line(kx, ky, fx, fy, col); c.hline(fx - 5, fy, 11, col);
}
void frontLeg(Canvas& c, int cx, int hy, const Foot& f, int sgn, float sc, uint16_t col) {
    const int hx = cx + sgn * 6;
    const int fx = ci((float)hx + (float)sgn * f.y * sc, cx - 31, cx + 31), fy = ci((float)hy + f.z * sc, hy + 20, 195);
    const int kx = (hx + fx) / 2 + sgn * 6, ky = (hy + fy) / 2;
    c.line(hx, hy, kx, ky, col); c.line(kx, ky, fx, fy, col); c.hline(fx - 5, fy, 11, col);
}
void torso(Canvas& c, int x, int hy, float a) {
    const int sx = x + (int)lroundf(sinf(a) * 29.0f), sy = hy - (int)lroundf(cosf(a) * 35.0f);
    c.line(x, hy, sx, sy, UI_GREEN);
    c.circle(sx + (int)lroundf(sinf(a) * 5.0f), sy - (int)lroundf(cosf(a) * 7.0f), 5, UI_GREEN);
}

void drawPose(const TelemetrySnapshot& t) {
    s_cv.begin(161, 48, 154, 155, UI_BG);
    Canvas& c = s_cv;
    c.vline(238, 48, 155, UI_DIM);
    Foot r{}, l{};
    feet(t, r, l);
    constexpr float sc = 0.53f;
    constexpr int hy = 100;            // V3: 91 -> kepala terpotong di tepi region
    torso(c, 199, hy, isFiniteVal(t.pitch) ? t.pitch : 0.0f);
    sideLeg(c, 199, hy, r, -6, sc, UI_GREEN);
    sideLeg(c, 199, hy, l, +6, sc, UI_GREEN2);
    torso(c, 277, hy, isFiniteVal(t.roll) ? t.roll : 0.0f);
    frontLeg(c, 277, hy, r, -1, sc, UI_GREEN);
    frontLeg(c, 277, hy, l, +1, sc, UI_GREEN2);
    c.hline(166, 197, 67, UI_DIM);
    c.hline(244, 197, 66, UI_DIM);
    flush();
}

void drawFooter(const TelemetrySnapshot& t, const NetStatus& n) {
    char buf[64];
    s_cv.begin(57, 209, 257, 24, UI_BG);
    Canvas& c = s_cv;
    int x = c.text(60, 212, n.staConnected ? n.staIp : n.apIp, UI_CYAN);
    x = c.text(x, 212, "  FB:", UI_DIM);
    const uint8_t fb = fbOkCount(t);
    snprintf(buf, sizeof(buf), "%u", (unsigned)fb);
    x = c.text(x, 212, buf, fb >= 20 ? UI_GREEN : UI_YELLOW);
    x = c.text(x, 212, "  OVR:", UI_DIM);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)t.loopOverruns);
    c.text(x, 212, buf, t.loopOverruns ? UI_YELLOW : UI_GREEN);
    const uint32_t saved = persist_saved_gen();
    c.text(60, 223, t.cfgGen != saved ? "NVS: pending (tunggu READY)" : "NVS: saved", t.cfgGen != saved ? UI_YELLOW : UI_DIM);
    flush();
}

void uiTask(void*) {
    if (!rgbInit()) logPrintf("RGB: init RMT gagal\r\n");
    rgbSet(80, 80, 80);
    if (lcdInit()) staticScreen(); else logPrintf("LCD: init ST7789 gagal\r\n");
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastSeq = 0;
    for (;;) {
        static TelemetrySnapshot t;
        t = telemetry_copy();
        const NetStatus n = net_status_get();
        const uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000);
        updateRgb(t, n, ms);
        if (s_lcdOk && (t.seq != lastSeq || t.seq == 0)) {
            lastSeq = t.seq;
            const int64_t t0 = esp_timer_get_time();
            drawStatus(t, n);
            drawPose(t);
            drawFooter(t, n);
            s_uiFrameUs = (uint32_t)(esp_timer_get_time() - t0);
        }
        if (s_uiFrameUs >= LOCAL_UI_PERIOD_MS * 1000u) { lastWake = xTaskGetTickCount(); vTaskDelay(pdMS_TO_TICKS(5)); }
        else xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOCAL_UI_PERIOD_MS));
    }
}

}  // namespace

void local_ui_start() {
    if (xTaskCreatePinnedToCore(uiTask, "LocalUI", STACK_UI, nullptr, PRIO_UI, nullptr, CORE_SYSTEM) != pdPASS)
        logPrintf("GAGAL membuat task LCD/RGB!\r\n");
}
