// =============================================================================
//  LapEndpoint: sisi robot dari protokol LAP (dipakai semua board + simulator)
//  Satu instance per link (USB CDC, koneksi TCP). Dipanggil dari SATU thread
//  komunikasi; tidak pernah dari task kontrol.
//
//  Aturan link:
//   * Stream telemetry hanya aktif setelah host mengirim HELLO atau STREAM_CFG.
//   * Host wajib mengirim paket apa pun (mis. PING) minimal tiap 3 s; bila tidak,
//     stream berhenti (link dianggap putus). Fase 1: GAIT_INTENT ikut berhenti.
// =============================================================================
#pragma once
#include "leumpang/lap.h"
#include "leumpang/motion.h"
#include "leumpang/files.h"

namespace la {
namespace lap {

class LapEndpoint {
public:
    struct Host {
        void* ctx = nullptr;
        void     (*write)(void* ctx, const uint8_t* d, size_t n) = nullptr;
        void*    writeCtx = nullptr;                                       // nullptr = pakai ctx
        bool     (*submit)(void* ctx, const Command& c) = nullptr;       // false = antrean penuh
        void     (*snapshot)(void* ctx, Telemetry& t) = nullptr;
        uint32_t (*savedGen)(void* ctx) = nullptr;                        // boleh nullptr
        bool     (*simHook)(void* ctx, const Packet& p) = nullptr;        // SIM_*; nullptr = tidak didukung
        MotionStore* motions = nullptr;                                    // nullptr = MOTION_* UNSUPPORTED
        void     (*motionSaved)(void* ctx, int slot, const uint8_t* d, size_t n) = nullptr;  // simpan ke SD
        // Fase 3: Leumpang Bus. Modul HAL menerima topik EXT dari host (nullptr = EXT UNSUPPORTED).
        bool     (*extHook)(void* ctx, uint16_t topic, const uint8_t* d, size_t n) = nullptr;
        // Fase 4A: layanan berkas SD (nullptr = FILE_* UNSUPPORTED) dan reboot (profil baru berlaku saat boot)
        FileOps* files = nullptr;
        void     (*reboot)(void* ctx) = nullptr;
    };
    static constexpr uint32_t LINK_TIMEOUT_MS = 3000;

    void init(const Host& h, const Hello& self);
    void onBytes(const uint8_t* d, size_t n, uint32_t nowMs);
    void poll(uint32_t nowMs);              // panggil >= 50 Hz
    void sendLog(const char* line);         // kirim LOG bila link aktif & logOn
    bool sendExt(uint16_t topic, const uint8_t* d, size_t n);   // terbitkan topik EXT ke host
    bool linkActive() const { return active_; }
    bool rxRecent(uint32_t nowMs) const { return dec_.frames && nowMs - lastRxMs_ <= LINK_TIMEOUT_MS; }
    const Decoder& decoder() const { return dec_; }

private:
    void handle(const Packet& p, uint32_t nowMs);
    template <typename M> void send(uint8_t type, const M& m) {
        Packet p;
        if (makePacket(p, type, seq_++, m)) sendPacket(p);
    }
    void sendPacket(const Packet& p);
    void ack(const Packet& p, uint8_t status);
    void sendConfig();
    void sendMotionStatus();
    void handleFile(const Packet& p);
    uint32_t rebootAtMs_ = 0;

    Host         h_{};
    Hello        self_{};
    Decoder      dec_{};
    StreamCfgMsg stream_{};
    bool         active_ = false;
    uint8_t      seq_ = 0;
    uint32_t     lastRxMs_ = 0, lastTele_ = 0, lastServo_ = 0, lastGait_ = 0, lastCfgGen_ = 0xFFFFFFFF;
    Telemetry    snap_{};
    // unggah motion
    int          upSlot_ = -1;
    size_t       upTotal_ = 0, upGot_ = 0;
    uint8_t      upBuf_[MOTION_MAX_BYTES];
};

}  // namespace lap
}  // namespace la
