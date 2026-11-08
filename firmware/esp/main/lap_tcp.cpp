// =============================================================================
//  Server LAP via TCP (Wi-Fi, port 5760) - core0, prioritas rendah
//  Satu klien aktif; klien baru menggantikan yang lama. Tulis non-blocking:
//  bila buffer TCP penuh, frame telemetry dibuang (bukan menahan task).
// =============================================================================
#include "shared.h"
#include "leumpang/motion.h"

#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "leumpang/lap_endpoint.h"

namespace {

constexpr size_t LOG_LINE = 160;
struct LogLine { char t[LOG_LINE]; };
QueueHandle_t s_logQ = nullptr;
int s_client = -1;
la::lap::LapEndpoint s_ep;

void epWrite(void*, const uint8_t* d, size_t n) {
    if (s_client < 0) return;
    const int r = send(s_client, d, n, MSG_DONTWAIT);
    if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) { close(s_client); s_client = -1; }
}
bool epSubmit(void*, const la::Command& c) { return control_queue_cmd(c); }
void epSnapshot(void*, la::Telemetry& t) { t = telemetry_copy(); }
uint32_t epSaved(void*) { return persist_saved_gen(); }

uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

void lapTask(void*) {
    la::lap::Hello self;
    self.role = la::lap::ROLE_ROBOT; self.board = LA_BOARD_ID;
    self.fwMajor = la::CORE_VER_MAJOR; self.fwMinor = la::CORE_VER_MINOR; self.fwPatch = la::CORE_VER_PATCH;
    self.caps = la::lap::CAP_WIFI | la::lap::CAP_LCD | la::lap::CAP_MOTION | la::lap::CAP_GAIT_INTENT | la::lap::CAP_LIVE_POSE |
                (storage_sd_ok() ? (uint32_t)la::lap::CAP_SD : 0u);
    strncpy(self.name, LA_BOARD_NAME, 16);
    la::lap::LapEndpoint::Host h;
    h.write = epWrite; h.submit = epSubmit; h.snapshot = epSnapshot; h.savedGen = epSaved;
    h.motions = &control_motions();
    h.motionSaved = [](void*, int slot, const uint8_t* d, size_t n) { storage_save_motion(slot, d, n); };

    const int lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int one = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a = {};
    a.sin_family = AF_INET; a.sin_port = htons(la::lap::TCP_PORT); a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(lsock, (sockaddr*)&a, sizeof(a)) != 0 || listen(lsock, 1) != 0) {
        logPrintf("LAP: bind/listen :%u gagal\r\n", (unsigned)la::lap::TCP_PORT);
        vTaskDelete(nullptr);
        return;
    }
    fcntl(lsock, F_SETFL, fcntl(lsock, F_GETFL, 0) | O_NONBLOCK);
    logPrintf("LAP: TCP :%u siap\r\n", (unsigned)la::lap::TCP_PORT);

    uint8_t buf[256];
    for (;;) {
        // klien baru?
        sockaddr_in ca; socklen_t cl = sizeof(ca);
        const int c = accept(lsock, (sockaddr*)&ca, &cl);
        if (c >= 0) {
            if (s_client >= 0) close(s_client);
            s_client = c;
            fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
            setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            s_ep.init(h, self);
            logPrintf("LAP: klien terhubung\r\n");
        }
        // terima
        if (s_client >= 0) {
            const int r = recv(s_client, buf, sizeof(buf), MSG_DONTWAIT);
            if (r > 0) s_ep.onBytes(buf, (size_t)r, nowMs());
            else if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) { close(s_client); s_client = -1; }
        }
        // log & stream
        LogLine ln;
        for (int i = 0; i < 8 && xQueueReceive(s_logQ, &ln, 0) == pdTRUE; ++i)
            if (s_client >= 0) s_ep.sendLog(ln.t);
        if (s_client >= 0) s_ep.poll(nowMs());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}  // namespace

void lap_forward_log(const char* line) {
    if (!s_logQ || !line) return;
    LogLine ln;
    strncpy(ln.t, line, LOG_LINE - 1); ln.t[LOG_LINE - 1] = 0;
    xQueueSend(s_logQ, &ln, 0);
}

void lap_tcp_start() {
    s_logQ = xQueueCreate(16, sizeof(LogLine));
    if (xTaskCreatePinnedToCore(lapTask, "LAP", STACK_LAP, nullptr, PRIO_LAP, nullptr, CORE_SYSTEM) != pdPASS)
        logPrintf("GAGAL membuat task LAP\r\n");
}
