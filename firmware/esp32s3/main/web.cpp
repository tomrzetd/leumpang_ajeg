// =============================================================================
//  Wi-Fi (SoftAP + STA opsional) + esp_http_server + dashboard (core0)
// =============================================================================
#include "shared.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "mbedtls/base64.h"
#if APP_USE_MDNS
#include "mdns.h"
#endif

extern const char DASHBOARD_HTML[];   // definisi di akhir file

// Field SSID/password wifi_config_t berukuran tetap dan TIDAK wajib diakhiri NUL.
template <size_t N>
static void copyField(uint8_t (&dst)[N], const char* src) {
    memset(dst, 0, N);
    const size_t n = src ? strnlen(src, N) : 0;
    memcpy(dst, src, n);
}

namespace {

esp_netif_t*     s_apNetif = nullptr;
esp_netif_t*     s_staNetif = nullptr;
httpd_handle_t   s_server = nullptr;
esp_timer_handle_t s_reconnTimer = nullptr;
portMUX_TYPE     s_netMux = portMUX_INITIALIZER_UNLOCKED;
NetStatus        s_net{};
bool             s_staWanted = false;
uint32_t         s_backoffMs = 3000;
char             s_authExpected[96];

// ============================================================================
//  Wi-Fi
// ============================================================================
void reconnCb(void*) { if (s_staWanted) esp_wifi_connect(); }

void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            if (s_staWanted) esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            portENTER_CRITICAL(&s_netMux);
            s_net.staConnected = false; strcpy(s_net.staIp, "-");
            portEXIT_CRITICAL(&s_netMux);
            // Backoff: scan STA berulang mengganggu klien SoftAP (radio berpindah kanal).
            if (s_staWanted && s_reconnTimer) {
                esp_timer_stop(s_reconnTimer);
                esp_timer_start_once(s_reconnTimer, (uint64_t)s_backoffMs * 1000);
                s_backoffMs = s_backoffMs * 2 > 60000 ? 60000 : s_backoffMs * 2;
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t* e = (const ip_event_got_ip_t*)data;
        portENTER_CRITICAL(&s_netMux);
        s_net.staConnected = true;
        snprintf(s_net.staIp, sizeof(s_net.staIp), IPSTR, IP2STR(&e->ip_info.ip));
        portEXIT_CRITICAL(&s_netMux);
        s_backoffMs = 3000;
        logPrintf("WIFI STA terhubung: %s\r\n", s_net.staIp);
    }
}

void wifiInit() {
    esp_netif_init();
    esp_event_loop_create_default();
    s_apNetif  = esp_netif_create_default_wifi_ap();
    s_staNetif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    esp_wifi_set_storage(WIFI_STORAGE_RAM);          // [V4-NVS] driver Wi-Fi tidak menulis flash
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, nullptr, nullptr);

    esp_timer_create_args_t ta = {};
    ta.callback = reconnCb; ta.name = "wifi_reconn";
    esp_timer_create(&ta, &s_reconnTimer);

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_net.apSsid, sizeof(s_net.apSsid), "LEUMPANG-AJEG-%02X%02X", mac[4], mac[5]);
    strcpy(s_net.staIp, "-");

    wifi_config_t ap = {};
    copyField(ap.ap.ssid, s_net.apSsid);
    ap.ap.ssid_len = (uint8_t)strlen(s_net.apSsid);
    copyField(ap.ap.password, WIFI_AP_PASS);
    ap.ap.channel = WIFI_AP_CHANNEL;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    char ssid[33] = {0}, pass[65] = {0};
    if (persist_load_wifi(ssid, sizeof(ssid), pass, sizeof(pass)) && ssid[0]) {
        wifi_config_t sta = {};
        copyField(sta.sta.ssid, ssid);
        copyField(sta.sta.password, pass);
        esp_wifi_set_config(WIFI_IF_STA, &sta);
        s_staWanted = true;
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);                   // latensi dashboard rendah

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(s_apNetif, &ip) == ESP_OK)
        snprintf(s_net.apIp, sizeof(s_net.apIp), IPSTR, IP2STR(&ip.ip));

#if APP_USE_MDNS
    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(MDNS_NAME);
        mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    }
#endif
    logPrintf("WEB AP: %s  IP=%s  http://%s.local  user=%s\r\n", s_net.apSsid, s_net.apIp, MDNS_NAME, DASH_USER);
    if (strcmp(WIFI_AP_PASS, "KHRrobot2026") == 0 || strcmp(DASH_PASS, "robot2026") == 0)
        logPrintf("PERINGATAN: password AP/dashboard masih default - ganti di app_config.h\r\n");
}

// ============================================================================
//  HTTP helper
// ============================================================================
struct JsonOut {
    char* b; size_t cap; size_t n;
    void f(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (n >= cap) return;
        va_list ap; va_start(ap, fmt);
        const int r = vsnprintf(b + n, cap - n, fmt, ap);
        va_end(ap);
        if (r > 0) n = (n + (size_t)r < cap) ? n + (size_t)r : cap - 1;
    }
    void i16arr(const char* key, const int16_t* a, size_t cnt) {
        f(",\"%s\":[", key);
        for (size_t i = 0; i < cnt; ++i) f("%s%d", i ? "," : "", (int)a[i]);
        f("]");
    }
    void u32arr(const char* key, const uint32_t* a, size_t cnt) {
        f(",\"%s\":[", key);
        for (size_t i = 0; i < cnt; ++i) f("%s%lu", i ? "," : "", (unsigned long)a[i]);
        f("]");
    }
    void u16arr(const char* key, const uint16_t* a, size_t cnt) {
        f(",\"%s\":[", key);
        for (size_t i = 0; i < cnt; ++i) f("%s%u", i ? "," : "", (unsigned)a[i]);
        f("]");
    }
    void u8arr(const char* key, const uint8_t* a, size_t cnt) {
        f(",\"%s\":[", key);
        for (size_t i = 0; i < cnt; ++i) f("%s%u", i ? "," : "", (unsigned)a[i]);
        f("]");
    }
};

bool ctEq(const char* a, const char* b) {               // perbandingan waktu-konstan
    const size_t la = strlen(a), lb = strlen(b);
    unsigned char d = (unsigned char)(la != lb);
    for (size_t i = 0; i < la && i < lb; ++i) d |= (unsigned char)(a[i] ^ b[i]);
    return d == 0;
}

bool auth(httpd_req_t* req) {
    char h[128];
    const size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (len > 0 && len < sizeof(h) && httpd_req_get_hdr_value_str(req, "Authorization", h, sizeof(h)) == ESP_OK &&
        ctEq(h, s_authExpected))
        return true;
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Leumpang Ajeg\"");
    httpd_resp_send(req, "auth required", HTTPD_RESP_USE_STRLEN);
    return false;
}

// [V4-SEC] Basic-auth dicache browser -> halaman lain bisa POST form lintas-origin (CSRF).
// Header custom memaksa preflight CORS yang tidak pernah dijawab server ini.
bool csrfOk(httpd_req_t* req) {
    char v[8];
    if (httpd_req_get_hdr_value_str(req, "X-KHR", v, sizeof(v)) == ESP_OK && strcmp(v, "1") == 0) return true;
    httpd_resp_set_status(req, "403 Forbidden");
    httpd_resp_send(req, "missing X-KHR header", HTTPD_RESP_USE_STRLEN);
    return false;
}

int readBody(httpd_req_t* req, char* buf, size_t cap) {
    if (req->content_len >= cap) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_send(req, "body too large", HTTPD_RESP_USE_STRLEN);
        return -1;
    }
    size_t got = 0;
    int retries = 0;
    while (got < req->content_len) {
        const int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++retries < 3) continue;
        if (r <= 0) { httpd_resp_send_500(req); return -1; }
        got += (size_t)r;
    }
    buf[got] = 0;
    return (int)got;
}

void urlDecode(char* s) {
    char* o = s;
    for (char* p = s; *p; ++p) {
        if (*p == '+') *o++ = ' ';
        else if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
            const char hx[3] = { p[1], p[2], 0 };
            *o++ = (char)strtol(hx, nullptr, 16);
            p += 2;
        } else *o++ = *p;
    }
    *o = 0;
}

bool formGet(const char* body, const char* key, char* out, size_t outLen) {
    if (httpd_query_key_value(body, key, out, outLen) != ESP_OK) return false;
    urlDecode(out);
    return true;
}

esp_err_t sendJson(httpd_req_t* req, const char* status, const char* json) {
    if (status) httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ============================================================================
//  Handler
// ============================================================================
esp_err_t hRoot(httpd_req_t* req) {
    if (!auth(req)) return ESP_OK;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t hHealth(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", 2);
}

esp_err_t hState(httpd_req_t* req) {
    if (!auth(req)) return ESP_OK;
    static char buf[6144];                         // handler httpd berjalan serial (1 task)
    static TelemetrySnapshot t;
    t = telemetry_copy();
    const NetStatus ns = net_status_get();
    JsonOut j{buf, sizeof(buf), 0};
    j.f("{\"seq\":%lu,\"ms\":%lu,\"mode\":%u,\"modeName\":\"%s\"", (unsigned long)t.seq, (unsigned long)t.ms,
        (unsigned)t.mode, mode_name(t.mode));
    j.f(",\"imu\":{\"roll\":%.3f,\"pitch\":%.3f,\"rawRoll\":%.3f,\"rawPitch\":%.3f,\"offRoll\":%.3f,\"offPitch\":%.3f",
        (double)(t.roll * kRad2Deg), (double)(t.pitch * kRad2Deg), (double)(t.rawRoll * kRad2Deg),
        (double)(t.rawPitch * kRad2Deg), (double)(t.offRoll * kRad2Deg), (double)(t.offPitch * kRad2Deg));
    j.f(",\"ageMs\":%ld,\"hz\":%u,\"fresh\":%s,\"init\":%s,\"cal\":%s}", (long)t.imuAgeMs, (unsigned)t.imuHz,
        t.imuFreshFlag ? "true" : "false", t.imuInit ? "true" : "false", t.calDone ? "true" : "false");
    j.f(",\"walk\":{\"fwct\":%.2f,\"fwctEnd\":%.2f,\"jiku\":%d,\"dxi\":%.2f,\"dyi\":%.2f,\"dxis\":%.2f,\"dyis\":%.2f",
        (double)t.fwct, (double)t.fwctEnd, (int)t.jikuasi, (double)t.dxi, (double)t.dyi, (double)t.dxis, (double)t.dyis);
    j.f(",\"autoH\":%.2f,\"fh\":%.2f,\"swx\":%.2f,\"swy\":%.2f,\"swMax\":%.2f,\"waist\":%.2f}",
        (double)t.autoH, (double)t.fh, (double)t.swx, (double)t.swy, (double)t.swMax, (double)t.waist);
    const ServoPositions& k = t.kin;
    j.f(",\"kin\":{\"K0\":[%d,%d],\"K1\":[%d,%d],\"K2\":[%d,%d],\"H\":[%d,%d],\"A0\":[%d,%d],\"A1\":[%d,%d]",
        k.K0[0], k.K0[1], k.K1[0], k.K1[1], k.K2[0], k.K2[1], k.H[0], k.H[1], k.A0[0], k.A0[1], k.A1[0], k.A1[1]);
    j.f(",\"U0\":[%d,%d],\"U1\":[%d,%d],\"U2\":[%d,%d],\"E\":[%d,%d],\"head\":%d,\"waist\":%d}",
        k.U0[0], k.U0[1], k.U1[0], k.U1[1], k.U2[0], k.U2[1], k.E[0], k.E[1], k.head, k.waist);
    j.f(",\"servo\":{\"n\":%u", (unsigned)SERVO_SLOTS);
    j.i16arr("cmdR", t.cmd[0], SERVO_SLOTS);   j.i16arr("cmdL", t.cmd[1], SERVO_SLOTS);
    j.i16arr("fbR", t.fb[0], SERVO_SLOTS);     j.i16arr("fbL", t.fb[1], SERVO_SLOTS);
    j.i16arr("trimR", t.trim[0], SERVO_SLOTS); j.i16arr("trimL", t.trim[1], SERVO_SLOTS);
    j.u8arr("validR", t.fbValid[0], SERVO_SLOTS); j.u8arr("validL", t.fbValid[1], SERVO_SLOTS);
    j.u16arr("ageR", t.fbAgeMs[0], SERVO_SLOTS);  j.u16arr("ageL", t.fbAgeMs[1], SERVO_SLOTS);
    j.u32arr("missR", t.fbMiss[0], SERVO_SLOTS);  j.u32arr("missL", t.fbMiss[1], SERVO_SLOTS);
    j.f("}");
    const RuntimeConfig& c = t.cfg;
    j.f(",\"cfg\":{\"fallDeg\":%.2f,\"rearmDeg\":%.2f,\"imuStale\":%lu,\"servoStep\":%d,\"trigR\":%.2f,\"trigP\":%.2f",
        (double)(c.fallAngleRad * kRad2Deg), (double)(c.rearmAngleRad * kRad2Deg), (unsigned long)(c.imuStaleUs / 1000),
        (int)c.servoMaxStep, (double)(c.triggerRollRad * kRad2Deg), (double)(c.triggerPitchRad * kRad2Deg));
    j.f(",\"footLift\":%.1f,\"swayBase\":%.1f,\"swayGain\":%.1f,\"rollSign\":%d,\"pitchSign\":%d,\"imuSwap\":%s}",
        (double)c.footLiftMax, (double)c.swayBase, (double)c.swayGain, (int)c.imuRollSign, (int)c.imuPitchSign,
        c.imuSwap ? "true" : "false");
    const uint32_t saved = persist_saved_gen();
    j.f(",\"persist\":{\"gen\":%lu,\"saved\":%lu,\"pending\":%s}", (unsigned long)t.cfgGen, (unsigned long)saved,
        t.cfgGen != saved ? "true" : "false");
    j.f(",\"perf\":{\"avgUs\":%lu,\"maxUs\":%lu,\"overruns\":%lu,\"icsUs\":%lu,\"stack\":%lu,\"heap\":%lu,\"psram\":%lu}",
        (unsigned long)t.loopAvgUs, (unsigned long)t.loopMaxUs, (unsigned long)t.loopOverruns,
        (unsigned long)t.servoPhaseUs, (unsigned long)t.ctrlStackFree,
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    j.f(",\"net\":{\"apSsid\":\"%s\",\"apIp\":\"%s\",\"sta\":%s,\"staIp\":\"%s\",\"rssi\":%d,\"clients\":%u}}",
        ns.apSsid, ns.apIp, ns.staConnected ? "true" : "false", ns.staIp, (int)ns.rssi, (unsigned)ns.apClients);
    return sendJson(req, nullptr, buf);
}

esp_err_t hCmd(httpd_req_t* req) {
    if (!auth(req) || !csrfOk(req)) return ESP_OK;
    char body[128], name[16];
    if (readBody(req, body, sizeof(body)) < 0) return ESP_OK;
    if (!formGet(body, "name", name, sizeof(name))) return sendJson(req, "400 Bad Request", "{\"ok\":false,\"error\":\"name\"}");
    RobotCommand c{};
    if      (!strcmp(name, "reset")) c.type = RobotCmdType::RESET;
    else if (!strcmp(name, "go"))    c.type = RobotCmdType::GO;
    else if (!strcmp(name, "stop"))  c.type = RobotCmdType::STOP;
    else if (!strcmp(name, "cal"))   c.type = RobotCmdType::CALIBRATE;
    else return sendJson(req, "400 Bad Request", "{\"ok\":false,\"error\":\"bad command\"}");
    const bool ok = control_queue_cmd(c);
    return sendJson(req, ok ? nullptr : "503 Service Unavailable", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"queue full\"}");
}

esp_err_t hConfig(httpd_req_t* req) {
    if (!auth(req) || !csrfOk(req)) return ESP_OK;
    char body[512], v[24];
    if (readBody(req, body, sizeof(body)) < 0) return ESP_OK;
    struct Map { const char* key; ParamId id; };
    static const Map map[] = {
        {"fallDeg", ParamId::FALL_DEG}, {"rearmDeg", ParamId::REARM_DEG}, {"imuStale", ParamId::IMU_STALE_MS},
        {"servoStep", ParamId::SERVO_STEP}, {"trigR", ParamId::TRIGGER_ROLL_DEG}, {"trigP", ParamId::TRIGGER_PITCH_DEG},
        {"footLift", ParamId::FOOT_LIFT}, {"swayBase", ParamId::SWAY_BASE}, {"swayGain", ParamId::SWAY_GAIN},
        {"rollSign", ParamId::SIGN_ROLL}, {"pitchSign", ParamId::SIGN_PITCH}, {"imuSwap", ParamId::IMU_SWAP},
    };
    bool ok = true;
    for (const Map& m : map) {
        if (!formGet(body, m.key, v, sizeof(v))) continue;
        char* end = nullptr;
        const float f = strtof(v, &end);
        if (end == v || !isFiniteVal(f)) { ok = false; continue; }
        RobotCommand c{}; c.type = RobotCmdType::SET_PARAM; c.param = m.id; c.value = f;
        ok &= control_queue_cmd(c);
    }
    return sendJson(req, ok ? nullptr : "503 Service Unavailable",
                    ok ? "{\"ok\":true,\"note\":\"diterapkan; disimpan ke NVS saat robot READY/FALLEN\"}"
                       : "{\"ok\":false,\"error\":\"nilai tidak valid / queue penuh\"}");
}

esp_err_t hTrim(httpd_req_t* req) {
    if (!auth(req) || !csrfOk(req)) return ESP_OK;
    char body[96], b[8], s[8], v[16];
    if (readBody(req, body, sizeof(body)) < 0) return ESP_OK;
    if (!formGet(body, "bus", b, sizeof(b)) || !formGet(body, "slot", s, sizeof(s)) || !formGet(body, "value", v, sizeof(v)))
        return sendJson(req, "400 Bad Request", "{\"ok\":false,\"error\":\"bus/slot/value\"}");
    const int bus = atoi(b), slot = atoi(s);
    if (bus < 0 || bus > 1 || slot < 0 || slot >= SERVO_SLOTS)
        return sendJson(req, "400 Bad Request", "{\"ok\":false,\"error\":\"range\"}");
    RobotCommand c{}; c.type = RobotCmdType::SET_TRIM; c.bus = (uint8_t)bus; c.slot = (uint8_t)slot;
    c.value = strtof(v, nullptr);
    const bool ok = isFiniteVal(c.value) && control_queue_cmd(c);
    return sendJson(req, ok ? nullptr : "503 Service Unavailable",
                    ok ? "{\"ok\":true,\"note\":\"trim hanya diterima di mode READY\"}" : "{\"ok\":false}");
}

esp_err_t hWifi(httpd_req_t* req) {
    if (!auth(req) || !csrfOk(req)) return ESP_OK;
    char body[256], ssid[33] = {0}, pass[65] = {0};
    if (readBody(req, body, sizeof(body)) < 0) return ESP_OK;
    if (!formGet(body, "ssid", ssid, sizeof(ssid))) return sendJson(req, "400 Bad Request", "{\"ok\":false,\"error\":\"ssid\"}");
    formGet(body, "pass", pass, sizeof(pass));
    persist_request_wifi(ssid, pass);   // kosong = pertahankan password lama
    return sendJson(req, nullptr, "{\"ok\":true,\"note\":\"STA reconnect; AP tetap aktif; NVS disimpan saat robot aman\"}");
}

void httpStart() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = CORE_SYSTEM;
    cfg.stack_size = STACK_HTTPD;
    cfg.max_uri_handlers = 12;
    cfg.max_open_sockets = 5;
    cfg.lru_purge_enable = true;
    if (httpd_start(&s_server, &cfg) != ESP_OK) { logPrintf("GAGAL start HTTP server\r\n"); return; }
    struct R { const char* uri; httpd_method_t m; esp_err_t (*h)(httpd_req_t*); };
    static const R routes[] = {
        {"/", HTTP_GET, hRoot}, {"/health", HTTP_GET, hHealth}, {"/api/state", HTTP_GET, hState},
        {"/api/cmd", HTTP_POST, hCmd}, {"/api/config", HTTP_POST, hConfig},
        {"/api/trim", HTTP_POST, hTrim}, {"/api/wifi", HTTP_POST, hWifi},
    };
    for (const R& r : routes) {
        httpd_uri_t u = {};
        u.uri = r.uri; u.method = r.m; u.handler = r.h; u.user_ctx = nullptr;
        httpd_register_uri_handler(s_server, &u);
    }
}

}  // namespace

// ============================================================================
//  API publik
// ============================================================================
void web_start() {
    char up[64];
    snprintf(up, sizeof(up), "%s:%s", DASH_USER, DASH_PASS);
    size_t olen = 0;
    strcpy(s_authExpected, "Basic ");
    mbedtls_base64_encode((unsigned char*)s_authExpected + 6, sizeof(s_authExpected) - 7, &olen,
                          (const unsigned char*)up, strlen(up));
    s_authExpected[6 + olen] = 0;
    wifiInit();
    httpStart();
}

NetStatus net_status_get() {
    NetStatus n;
    portENTER_CRITICAL(&s_netMux);
    n = s_net;
    portEXIT_CRITICAL(&s_netMux);
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) n.apClients = (uint8_t)list.num;
    wifi_ap_record_t ap;
    n.rssi = (n.staConnected && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;
    return n;
}

void wifi_set_sta(const char* ssid, const char* pass) {
    wifi_config_t sta = {};
    esp_wifi_get_config(WIFI_IF_STA, &sta);
    copyField(sta.sta.ssid, ssid);
    if (pass && pass[0]) copyField(sta.sta.password, pass);
    s_staWanted = ssid[0] != 0;
    s_backoffMs = 3000;
    if (s_reconnTimer) esp_timer_stop(s_reconnTimer);
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &sta);
    if (s_staWanted) esp_wifi_connect();
}

// ============================================================================
//  DASHBOARD
// ============================================================================
const char DASHBOARD_HTML[] = R"HTML(<!doctype html><html lang="id"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Leumpang Ajeg // ESP32-S3</title><style>
:root{--g:#35ff75;--g2:#a5ffc0;--dim:#4c8d60;--line:#164724}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#06200e,#010402 55%);color:var(--g);font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:14px}.wrap{max-width:1500px;margin:auto;padding:12px}.hdr{display:flex;gap:12px;align-items:center;justify-content:space-between;border:1px solid var(--line);background:#020b05;padding:12px;box-shadow:0 0 24px #00ff4422}.title{font-weight:800;letter-spacing:2px}.blink{animation:b 1s steps(2) infinite}@keyframes b{50%{opacity:.2}}.grid{display:grid;grid-template-columns:repeat(12,1fr);gap:10px;margin-top:10px}.card{grid-column:span 3;border:1px solid var(--line);background:#031008dd;padding:10px;min-width:0}.wide{grid-column:span 6}.full{grid-column:1/-1}h3{margin:0 0 8px;color:var(--g2);font-size:13px;text-transform:uppercase;letter-spacing:1px}.kv{display:grid;grid-template-columns:1fr auto;gap:5px;border-bottom:1px dashed #16472455;padding:3px 0}button,input,select{font:inherit;background:#020704;color:var(--g);border:1px solid var(--dim);padding:8px}button{cursor:pointer}button:hover{background:#0a2712}.danger{border-color:#a33;color:#ff8989}.go{border-color:#35ff75;color:#b8ffca}.btns{display:flex;flex-wrap:wrap;gap:8px}.form{display:grid;grid-template-columns:repeat(4,minmax(120px,1fr));gap:8px}.field{display:flex;flex-direction:column;gap:3px;color:var(--dim)}canvas{width:100%;height:300px;border:1px solid #153d21;background:#010603}table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:5px;border-bottom:1px solid #10351b;text-align:right}th:first-child,td:first-child{text-align:left}.ok{color:#7dff9f}.bad{color:#ff7373}.warn{color:#ffd166}.note{color:var(--dim);font-size:12px;line-height:1.5}.scroll{overflow:auto;max-height:440px}@media(max-width:900px){.card,.wide{grid-column:span 6}.form{grid-template-columns:repeat(2,1fr)}}@media(max-width:600px){.card,.wide{grid-column:1/-1}.form{grid-template-columns:1fr 1fr}.hdr{align-items:flex-start;flex-direction:column}canvas{height:250px}}
</style></head><body><div class="wrap"><div class="hdr"><div><div class="title">[ LEUMPANG AJEG // WALKING STEADILY // ESP32-S3 ]</div><div class="note">Mahony IMU + ICS feedback + trim NVS · <span class="blink">_</span></div></div><div id="conn">LINK: --</div></div>
<div class="grid">
<div class="card"><h3>MODE</h3><div class="kv"><span>state</span><span id="mode">--</span></div><div class="kv"><span>fwct</span><span id="fwct">--</span></div><div class="kv"><span>support</span><span id="jiku">--</span></div><div class="kv"><span>NVS</span><span id="nvs">--</span></div></div>
<div class="card"><h3>IMU</h3><div class="kv"><span>roll</span><span id="roll">--</span></div><div class="kv"><span>pitch</span><span id="pitch">--</span></div><div class="kv"><span>age / rate</span><span id="imuage">--</span></div><div class="kv"><span>cal</span><span id="cal">--</span></div></div>
<div class="card"><h3>REALTIME core1</h3><div class="kv"><span>loop avg/max</span><span id="loop">--</span></div><div class="kv"><span>ICS phase</span><span id="ics">--</span></div><div class="kv"><span>overrun</span><span id="over">--</span></div><div class="kv"><span>stack free</span><span id="stk">--</span></div></div>
<div class="card"><h3>NETWORK</h3><div class="kv"><span>AP</span><span id="ap">--</span></div><div class="kv"><span>STA IP</span><span id="ip">--</span></div><div class="kv"><span>RSSI / clients</span><span id="rssi">--</span></div><div class="kv"><span>heap / PSRAM</span><span id="heap">--</span></div></div>
<div class="card wide"><h3>IMU TRACE</h3><canvas id="chart" width="700" height="300"></canvas></div>
<div class="card wide"><h3>POSE ESTIMATOR · target kinematics</h3><canvas id="pose" width="700" height="300"></canvas><div class="note">Target x/y/h gait + tilt IMU, bukan motion-capture. Posisi aktual servo ada di tabel ICS.</div></div>
<div class="card full"><h3>CONTROL</h3><div class="btns"><button class="go" onclick="cmd('go')">ARM / BALANCE</button><button onclick="cmd('reset')">RESET / HOME</button><button onclick="cmd('cal')">CAL IMU</button><button class="danger" onclick="cmd('stop')">STOP / SERVO FREE</button></div><div id="msg" class="note" style="margin-top:8px"></div></div>
<div class="card wide"><h3>RUNTIME TUNING · NVS (ditunda sampai robot aman)</h3><div class="form" id="cfgForm">
<label class="field">Fall angle °<input name="fallDeg" type="number" step="0.1"></label><label class="field">Re-arm °<input name="rearmDeg" type="number" step="0.1"></label><label class="field">IMU stale ms<input name="imuStale" type="number" step="1"></label><label class="field">Servo max step<input name="servoStep" type="number" step="1"></label>
<label class="field">Trigger roll °<input name="trigR" type="number" step="0.1"></label><label class="field">Trigger pitch °<input name="trigP" type="number" step="0.1"></label><label class="field">Foot lift mm<input name="footLift" type="number" step="0.5"></label><label class="field">Sway base<input name="swayBase" type="number" step="0.5"></label>
<label class="field">Sway gain<input name="swayGain" type="number" step="0.5"></label><label class="field">Roll sign<select name="rollSign"><option value="1">+1</option><option value="-1">-1</option></select></label><label class="field">Pitch sign<select name="pitchSign"><option value="1">+1</option><option value="-1">-1</option></select></label><label class="field">Swap roll/pitch<select name="imuSwap"><option value="0">NO</option><option value="1">YES</option></select></label>
</div><button style="margin-top:9px" onclick="saveCfg()">APPLY + SAVE</button><button style="margin-top:9px" onclick="firstCfg=true">RELOAD</button></div>
<div class="card wide"><h3>WIFI STA</h3><div class="form"><label class="field">SSID<input id="ssid" maxlength="32"></label><label class="field">Password (kosong = tetap)<input id="wpass" type="password" maxlength="64"></label></div><button style="margin-top:9px" onclick="saveWifi()">SAVE + RECONNECT</button><div class="note">SoftAP tetap aktif. Bila router di kanal lain, AP ikut pindah kanal dan klien AP putus sesaat.</div>
<h3 style="margin-top:14px">TRIM SENDI · hanya mode READY</h3><div class="form"><label class="field">Sendi<select id="tj"></select></label><label class="field">Trim (hitungan, ±1000)<input id="tv" type="number" step="10" value="0"></label></div><button style="margin-top:9px" onclick="setTrim()">SET TRIM</button></div>
<div class="card full"><h3>SERVO BUS · command vs reply ICS</h3><div class="note">Reply ICS3.5 = posisi saat command diterima (tertinggal ~1 siklus wajar). Age = umur reply terakhir.</div><div class="scroll"><table><thead><tr><th>Joint</th><th>CMD</th><th>FB</th><th>Err</th><th>Trim</th><th>Age ms</th><th>Reply</th><th>Miss</th></tr></thead><tbody id="servoRows"></tbody></table></div></div>
</div></div>
<script>
const $=id=>document.getElementById(id),histR=[],histP=[];let firstCfg=true;
const namesR=['WAIST','R SHOULDER P','R SHOULDER R','R SHOULDER Y','R ELBOW','R HIP Y','R HIP R','R HIP P','R KNEE','R ANKLE P','R ANKLE R'];
const namesL=['HEAD','L SHOULDER P','L SHOULDER R','L SHOULDER Y','L ELBOW','L HIP Y','L HIP R','L HIP P','L KNEE','L ANKLE P','L ANKLE R'];
(function(){let h='';[namesR,namesL].forEach((a,b)=>a.forEach((n,i)=>h+=`<option value="${b},${i}">${n}</option>`));$('tj').innerHTML=h})();
async function post(url,obj){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-KHR':'1'},body:new URLSearchParams(obj).toString()});const t=await r.text();if(!r.ok)throw Error(t);return t}
async function cmd(name){if(name==='go'&&!confirm('ARM/BALANCE servo sekarang?'))return;if(name==='stop'&&!confirm('FREE semua servo? Robot dapat jatuh bila tidak disangga.'))return;try{$('msg').textContent=await post('/api/cmd',{name})}catch(e){$('msg').textContent='ERR '+e}}
async function saveCfg(){let o={};document.querySelectorAll('#cfgForm [name]').forEach(x=>o[x.name]=x.value);try{$('msg').textContent=await post('/api/config',o);firstCfg=true}catch(e){$('msg').textContent='ERR '+e}}
async function saveWifi(){try{$('msg').textContent=await post('/api/wifi',{ssid:$('ssid').value,pass:$('wpass').value});$('wpass').value=''}catch(e){$('msg').textContent='ERR '+e}}
async function setTrim(){const [bus,slot]=$('tj').value.split(',');try{$('msg').textContent=await post('/api/trim',{bus,slot,value:$('tv').value})}catch(e){$('msg').textContent='ERR '+e}}
function setCfg(c){if(!firstCfg)return;firstCfg=false;for(const k of ['fallDeg','rearmDeg','imuStale','servoStep','trigR','trigP','footLift','swayBase','swayGain','rollSign','pitchSign']){const x=document.querySelector('[name='+k+']');if(x)x.value=c[k]}document.querySelector('[name=imuSwap]').value=c.imuSwap?1:0}
function drawChart(){const c=$('chart'),x=c.getContext('2d'),w=c.width,h=c.height;x.clearRect(0,0,w,h);x.strokeStyle='#123a1e';x.lineWidth=1;for(let i=0;i<5;i++){let y=i*h/4;x.beginPath();x.moveTo(0,y);x.lineTo(w,y);x.stroke()}const d=(a,col)=>{x.strokeStyle=col;x.lineWidth=2;x.beginPath();a.forEach((v,i)=>{let px=i*w/119,py=h/2-v*(h/90);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()};d(histR,'#35ff75');d(histP,'#a5ffc0');x.fillStyle='#35ff75';x.fillText('R',8,14);x.fillStyle='#a5ffc0';x.fillText('P',28,14)}
function drawPose(d){const c=$('pose'),g=c.getContext('2d'),w=c.width,h=c.height;g.clearRect(0,0,w,h);g.strokeStyle='#133d20';g.beginPath();g.moveTo(w/2,0);g.lineTo(w/2,h);g.stroke();g.fillStyle='#4c8d60';g.fillText('SIDE / X-Z',10,16);g.fillText('FRONT / Y-Z',w/2+10,16);const sc=.95,hipY=60,W=d.walk;let R,L;if(W.jiku===0){R={x:W.dxi-W.swx,y:W.dyi-W.swy,z:W.autoH};L={x:W.dxis-W.swx,y:W.dyis+W.swy,z:W.autoH-W.fh}}else{R={x:W.dxis-W.swx,y:W.dyis+W.swy,z:W.autoH-W.fh};L={x:W.dxi-W.swx,y:W.dyi-W.swy,z:W.autoH}}
function body(cx,a){g.strokeStyle='#35ff75';g.lineWidth=3;g.beginPath();g.moveTo(cx,hipY);g.lineTo(cx+Math.sin(a)*50,hipY-55*Math.cos(a));g.stroke();g.beginPath();g.arc(cx+Math.sin(a)*58,hipY-65*Math.cos(a),9,0,Math.PI*2);g.stroke()}
function legS(cx,F,off){const fx=cx+F.x*sc,fy=hipY+F.z*sc,mx=(cx+fx)/2+off,my=(hipY+fy)/2;g.beginPath();g.moveTo(cx,hipY);g.lineTo(mx,my);g.lineTo(fx,fy);g.stroke();g.strokeRect(fx-8,fy-2,16,4)}
function legF(cx,F,s){const hx=cx+s*11,fx=hx+s*F.y*sc,fy=hipY+F.z*sc,mx=(hx+fx)/2+s*10,my=(hipY+fy)/2;g.beginPath();g.moveTo(hx,hipY);g.lineTo(mx,my);g.lineTo(fx,fy);g.stroke();g.strokeRect(fx-8,fy-2,16,4)}
const s=w/4,f=3*w/4;body(s,d.imu.pitch*Math.PI/180);legS(s,R,-10);legS(s,L,10);body(f,d.imu.roll*Math.PI/180);legF(f,R,-1);legF(f,L,1)}
function servoTable(d){const S=d.servo;let h='';for(let b=0;b<2;b++){const nm=b?namesL:namesR,cmd=b?S.cmdL:S.cmdR,fb=b?S.fbL:S.fbR,v=b?S.validL:S.validR,m=b?S.missL:S.missR,tr=b?S.trimL:S.trimR,ag=b?S.ageL:S.ageR;for(let i=0;i<S.n;i++){const ok=!!v[i],err=ok?fb[i]-cmd[i]:'--';h+=`<tr><td>${nm[i]}</td><td>${cmd[i]}</td><td>${ok?fb[i]:'--'}</td><td class="${ok&&Math.abs(err)>300?'warn':''}">${err}</td><td>${tr[i]}</td><td>${ag[i]>=65535?'--':ag[i]}</td><td class="${ok?'ok':'bad'}">${ok?'OK':'NO'}</td><td>${m[i]}</td></tr>`}}$('servoRows').innerHTML=h}
let busy=false;async function poll(){if(busy)return;busy=true;try{const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error(r.status);const d=await r.json();$('conn').textContent='LINK: ONLINE / seq '+d.seq;$('conn').className='ok';$('mode').textContent=d.modeName+' ['+d.mode+']';$('fwct').textContent=d.walk.fwct.toFixed(1)+'/'+d.walk.fwctEnd.toFixed(1);$('jiku').textContent=d.walk.jiku?'LEFT':'RIGHT';$('nvs').textContent=d.persist.pending?'PENDING':'SAVED';$('nvs').className=d.persist.pending?'warn':'ok';$('roll').textContent=d.imu.roll.toFixed(2)+'°';$('pitch').textContent=d.imu.pitch.toFixed(2)+'°';$('imuage').textContent=d.imu.ageMs+' ms '+(d.imu.fresh?'OK':'STALE')+' / '+d.imu.hz+' Hz';$('cal').textContent=d.imu.cal?'OK':'WAIT';$('loop').textContent=d.perf.avgUs+' / '+d.perf.maxUs+' us';$('ics').textContent=d.perf.icsUs+' us';$('over').textContent=d.perf.overruns;$('stk').textContent=d.perf.stack+' B';$('heap').textContent=Math.round(d.perf.heap/1024)+' / '+Math.round(d.perf.psram/1024)+' KB';$('ap').textContent=d.net.apSsid+' @ '+d.net.apIp;$('ip').textContent=d.net.staIp;$('rssi').textContent=(d.net.sta?d.net.rssi+' dBm':'AP only')+' / '+d.net.clients;histR.push(d.imu.roll);histP.push(d.imu.pitch);if(histR.length>120){histR.shift();histP.shift()}drawChart();drawPose(d);servoTable(d);setCfg(d.cfg)}catch(e){$('conn').textContent='LINK: OFFLINE';$('conn').className='bad'}busy=false}
setInterval(poll,250);poll();
</script></body></html>
)HTML";
