// =============================================================================
//  Leumpang Radio (Fase 4B): stik Bluetooth -> robot lewat UART (LAP PAD 50 Hz).
//  Kode bersama ESP32 klasik dan Pico 2 W; berjalan di run loop BTstack (satu thread).
//
//  - Satu stik saja yang mengendalikan robot (stik pertama yang siap; lainnya ditolak).
//  - Klik touchpad DS4/DS5 datang lewat "mouse virtual" anak stik (Bluepad32): diteruskan
//    sebagai PAD_TOUCH (E-STOP di robot).
//  - Robot membalas PAD_FEEDBACK: warna lightbar per mode, getar saat jatuh/E-STOP.
//    Robot diam > 2 s: lightbar putih redup (radio hidup, robot tidak menjawab).
//  - LED board: berkedip = mencari stik, menyala = stik tersambung.
//  Pemetaan tombol -> aksi robot ada di core (Controller::padInput), bukan di sini.
// =============================================================================
#include <btstack_run_loop.h>
#include <string.h>
#include <uni.h>

#include "leumpang/lap.h"
#include "leumpang/pad.h"
#include "radio_hal.h"

namespace {

constexpr uint32_t TICK_MS = 20;              // 50 Hz
constexpr uint32_t ROBOT_SILENT_MS = 2000;

uni_hid_device_t*   s_pad = nullptr;
la::PadState        s_state{};
bool                s_touch = false;
la::lap::Decoder    s_dec;
uint8_t             s_txSeq = 0;
uint32_t            s_tick = 0, s_lastFbMs = 0;
bool                s_robotOk = false;
uint8_t             s_rgb[3] = {0, 0, 0};
uint8_t             s_leds = 0xFF;
btstack_timer_source_t s_timer;

int8_t axis8(int32_t v) {                      // Bluepad32 -512..511 -> -127..127
    v /= 4;
    return (int8_t)(v > 127 ? 127 : (v < -127 ? -127 : v));
}
uint8_t trig8(int32_t v) {                     // 0..1023 -> 0..255
    v /= 4;
    return (uint8_t)(v > 255 ? 255 : (v < 0 ? 0 : v));
}

uint32_t mapButtons(const uni_gamepad_t& g) {
    uint32_t b = 0;
    if (g.buttons & BUTTON_A)          b |= la::PAD_CROSS;
    if (g.buttons & BUTTON_B)          b |= la::PAD_CIRCLE;
    if (g.buttons & BUTTON_X)          b |= la::PAD_SQUARE;
    if (g.buttons & BUTTON_Y)          b |= la::PAD_TRIANGLE;
    if (g.buttons & BUTTON_SHOULDER_L) b |= la::PAD_L1;
    if (g.buttons & BUTTON_SHOULDER_R) b |= la::PAD_R1;
    if (g.buttons & BUTTON_TRIGGER_L)  b |= la::PAD_L2;
    if (g.buttons & BUTTON_TRIGGER_R)  b |= la::PAD_R2;
    if (g.buttons & BUTTON_THUMB_L)    b |= la::PAD_L3;
    if (g.buttons & BUTTON_THUMB_R)    b |= la::PAD_R3;
    if (g.misc_buttons & MISC_BUTTON_SELECT) b |= la::PAD_SHARE;
    if (g.misc_buttons & MISC_BUTTON_START)  b |= la::PAD_OPTIONS;
    if (g.misc_buttons & MISC_BUTTON_SYSTEM) b |= la::PAD_PS;
    if (g.dpad & DPAD_UP)    b |= la::PAD_UP;
    if (g.dpad & DPAD_DOWN)  b |= la::PAD_DOWN;
    if (g.dpad & DPAD_RIGHT) b |= la::PAD_RIGHT;
    if (g.dpad & DPAD_LEFT)  b |= la::PAD_LEFT;
    return b;
}

void sendPad() {
    la::lap::Packet p;
    p.type = la::lap::PAD; p.seq = s_txSeq++;
    la::PadState st = s_state;
    if (s_pad) {
        st.flags = la::PAD_CONNECTED;
        if (s_touch) st.buttons |= la::PAD_TOUCH;
    } else {
        st = la::PadState{};
    }
    st.seq = p.seq;
    la::padEncode(st, p.body);
    p.len = la::PAD_BODY_LEN;
    uint8_t f[48];
    const size_t n = la::lap::encodeFrame(p, f, sizeof f);
    if (n) radio_hal_uart_write(f, n);
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_pad || !s_pad->report_parser.set_lightbar_color) return;
    if (s_rgb[0] == r && s_rgb[1] == g && s_rgb[2] == b) return;
    s_rgb[0] = r; s_rgb[1] = g; s_rgb[2] = b;
    s_pad->report_parser.set_lightbar_color(s_pad, r, g, b);
}

void applyFeedback(const uint8_t* b) {
    s_lastFbMs = radio_hal_ms();
    s_robotOk = true;
    if (!s_pad) return;
    setColor(b[0], b[1], b[2]);
    if (b[3] && s_pad->report_parser.play_dual_rumble)
        s_pad->report_parser.play_dual_rumble(s_pad, 0, (uint16_t)(b[3] * 10), b[4], b[5]);
    if (b[6] != s_leds && s_pad->report_parser.set_player_leds) {
        s_leds = b[6];
        s_pad->report_parser.set_player_leds(s_pad, b[6]);
    }
}

void tick(btstack_timer_source_t* ts) {
    uint8_t buf[64];
    la::lap::Packet p;
    int n;
    while ((n = radio_hal_uart_read(buf, sizeof buf)) > 0)
        for (int i = 0; i < n; ++i)
            if (s_dec.feed(buf[i], p) && p.type == la::lap::PAD_FEEDBACK && p.len >= la::PAD_FEEDBACK_LEN)
                applyFeedback(p.body);
    const uint32_t now = radio_hal_ms();
    if (s_robotOk && now - s_lastFbMs > ROBOT_SILENT_MS) {
        s_robotOk = false;
        logi("radio: robot tidak menjawab\n");
    }
    if (!s_robotOk) setColor(12, 12, 12);

    ++s_tick;
    if (s_pad || (s_tick % 5) == 0) sendPad();           // tanpa stik: 10 Hz "terputus"
    radio_hal_led(s_pad ? 1 : (int)((s_tick / 12) & 1));

    btstack_run_loop_set_timer(ts, TICK_MS);
    btstack_run_loop_add_timer(ts);
}

// ---- callback platform Bluepad32 ----------------------------------------------------------
void plInit(int, const char**) { logi("Leumpang Radio (%s)\n", radio_hal_board()); }

void plInitComplete(void) {
    uni_bt_start_scanning_and_autoconnect_unsafe();       // kunci pairing disimpan: DS4 cukup tekan PS
    btstack_run_loop_set_timer_handler(&s_timer, tick);
    btstack_run_loop_set_timer(&s_timer, TICK_MS);
    btstack_run_loop_add_timer(&s_timer);
}

uni_error_t plDiscovered(bd_addr_t, const char*, uint16_t cod, uint8_t) {
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD)
        return UNI_ERROR_IGNORE_DEVICE;                    // keyboard: bukan untuk robot
    return UNI_ERROR_SUCCESS;
}

void plConnected(uni_hid_device_t*) {}

void plDisconnected(uni_hid_device_t* d) {
    if (d == s_pad) {
        s_pad = nullptr; s_state = la::PadState{}; s_touch = false;
        s_rgb[0] = s_rgb[1] = s_rgb[2] = 0; s_leds = 0xFF;
        logi("radio: stik terputus\n");
    } else if (s_pad && d->parent == s_pad) {
        s_touch = false;
    }
}

uni_error_t plReady(uni_hid_device_t* d) {
    if (uni_hid_device_is_virtual_device(d)) return (d->parent == s_pad) ? UNI_ERROR_SUCCESS : UNI_ERROR_IGNORE_DEVICE;
    if (s_pad && s_pad != d) {
        logi("radio: stik kedua ditolak (hanya satu pengendali)\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }
    s_pad = d;
    s_state = la::PadState{};
    s_rgb[0] = s_rgb[1] = s_rgb[2] = 0; s_leds = 0xFF;
    logi("radio: stik siap\n");
    if (d->report_parser.play_dual_rumble) d->report_parser.play_dual_rumble(d, 0, 150, 120, 0);
    return UNI_ERROR_SUCCESS;
}

void plData(uni_hid_device_t* d, uni_controller_t* ctl) {
    if (ctl->klass == UNI_CONTROLLER_CLASS_MOUSE) {
        if (s_pad && d->parent == s_pad) s_touch = ctl->mouse.buttons != 0;   // klik touchpad
        return;
    }
    if (d != s_pad || ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) return;
    const uni_gamepad_t& g = ctl->gamepad;
    s_state.battery = ctl->battery;
    s_state.buttons = mapButtons(g);
    s_state.lx = axis8(g.axis_x);  s_state.ly = axis8(-g.axis_y);
    s_state.rx = axis8(g.axis_rx); s_state.ry = axis8(-g.axis_ry);
    s_state.l2 = trig8(g.brake);   s_state.r2 = trig8(g.throttle);
}

const uni_property_t* plProperty(uni_property_idx_t) { return nullptr; }

void plOob(uni_platform_oob_event_t, void*) {}

}  // namespace

extern "C" struct uni_platform* radio_platform(void) {
    static struct uni_platform plat = {};
    plat.name = "Leumpang Radio";
    plat.init = plInit;
    plat.on_init_complete = plInitComplete;
    plat.on_device_discovered = plDiscovered;
    plat.on_device_connected = plConnected;
    plat.on_device_disconnected = plDisconnected;
    plat.on_device_ready = plReady;
    plat.on_controller_data = plData;
    plat.get_property = plProperty;
    plat.on_oob_event = plOob;
    return &plat;
}
