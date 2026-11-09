// =============================================================================
//  Model servo ICS 3.5 untuk simulator (studio SimRobot & leumpang_sim) dan tes.
//  32 ID per bus; ID >= SERVO_SLOTS tidak terpasang secara bawaan.
//  - exchange(): frame posisi (0 = free), seperti bus kontrol 100 Hz
//  - transact(): ID / parameter / EEPROM, dengan echo half-duplex (tx muncul lagi di rx)
//  Bukan untuk firmware.
// =============================================================================
#pragma once
#include "leumpang/ics.h"
#include "leumpang/controller.h"

#include <math.h>
#include <string.h>

namespace la {

struct IcsSimBus : ServoBus {
    float   pos[2][32];
    int     cmd[2][32];
    bool    freeT[2][32];
    bool    dead[2][32];
    uint8_t param[2][32][5];          // [sc] 1 stretch, 2 speed, 3 batas arus, 4 batas suhu
    float   speed = 120.0f;           // hitungan / 10 ms

    // startOffset(b, i): simpangan awal dari center (acak untuk uji boot sync, atau 0)
    template <typename F> void reset(F startOffset) {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
            const JointDef& j = JOINTS[b][i < SERVO_SLOTS ? i : 0];
            pos[b][i] = (float)j.center + startOffset(b, i);
            cmd[b][i] = 0; freeT[b][i] = true; dead[b][i] = i >= SERVO_SLOTS;
            param[b][i][0] = 0; param[b][i][1] = 60; param[b][i][2] = 127; param[b][i][3] = 63; param[b][i][4] = 80;
        }
    }

    void exchange(IcsExchange& x) override {
        for (int b = 0; b < 2; ++b) {
            const int id = x.tx[b][0] & 0x1F, p = (x.tx[b][1] << 7) | x.tx[b][2];
            x.got[b] = !dead[b][id];
            if (!x.got[b]) continue;
            x.pos[b] = (int16_t)lroundf(pos[b][id]);
            if (p == 0) freeT[b][id] = true; else { freeT[b][id] = false; cmd[b][id] = p; }
        }
    }

    int transact(uint8_t bus, const uint8_t* tx, uint8_t txLen, uint8_t* rx, uint8_t rxMax,
                 uint8_t want, uint32_t timeoutUs) override {
        (void)want; (void)timeoutUs;
        if (bus > 1 || txLen == 0 || rxMax < txLen) return 0;
        uint8_t n = 0;
        for (uint8_t i = 0; i < txLen; ++i) rx[n++] = tx[i];
        auto put = [&](uint8_t v) { if (n < rxMax) rx[n++] = v; };
        const uint8_t op = tx[0] & 0xE0, id = tx[0] & 0x1F;
        if (txLen == 4 && (tx[0] == 0xFF || op == 0xE0)) {             // baca / tulis ID
            int only = -1, alive = 0;
            for (int i = 0; i < 32; ++i) if (!dead[bus][i]) { ++alive; only = i; }
            if (alive != 1) return n;                                  // tabrakan / kosong
            if (tx[0] != 0xFF && id != only) {                         // pindahkan state ke ID baru
                pos[bus][id] = pos[bus][only]; cmd[bus][id] = cmd[bus][only]; freeT[bus][id] = freeT[bus][only];
                memcpy(param[bus][id], param[bus][only], sizeof(param[bus][id]));
                dead[bus][id] = false; dead[bus][only] = true;
            }
            put((uint8_t)(0xE0 | (tx[0] == 0xFF ? only : id)));
            return n;
        }
        if (dead[bus][id]) return n;
        if (op == 0xA0 && txLen == 2) {
            const uint8_t sc = tx[1];
            if (sc > 4) return n;
            put((uint8_t)(0x20 | id)); put(sc);
            if (sc == 0) { for (int i = 0; i < 64; ++i) put((uint8_t)((i * 7 + id) & 0x0F)); }
            else if (sc == 3) put(2);                                   // arus terukur (tanpa beban)
            else if (sc == 4) put(75);                                  // suhu terukur (nilai ICS)
            else put(param[bus][id][sc]);
            return n;
        }
        if (op == 0xC0 && txLen == 3 && tx[1] >= 1 && tx[1] <= 4) {
            param[bus][id][tx[1]] = tx[2];
            put((uint8_t)(0x40 | id)); put(tx[1]); put(tx[2]);
            return n;
        }
        if (op < 0xA0 && (tx[0] & 0x80) && txLen == 3) {               // frame posisi
            const int p = (tx[1] << 7) | tx[2];
            const int16_t v = (int16_t)lroundf(pos[bus][id]);
            if (p == 0) freeT[bus][id] = true; else { freeT[bus][id] = false; cmd[bus][id] = p; }
            put((uint8_t)(tx[0] & 0x7F)); put((uint8_t)((v >> 7) & 0x7F)); put((uint8_t)(v & 0x7F));
            return n;
        }
        return n;
    }

    void physics() {
        for (int b = 0; b < 2; ++b) for (int i = 0; i < 32; ++i) {
            if (freeT[b][i] || dead[b][i]) continue;
            const float d = (float)cmd[b][i] - pos[b][i];
            pos[b][i] += d > speed ? speed : (d < -speed ? -speed : d);
        }
    }
};

}  // namespace la
