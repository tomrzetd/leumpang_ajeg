# Arsitektur Leumpang Ajeg

```
 HP Android ("mata")              PC/Laptop x86 ("studio")
 kamera, depth, UWB, AI           pose kamera, digital twin, tuning servo
        │ USB-OTG (CDC)                 │ USB / TCP Wi-Fi
        └──────────┐          ┌─────────┘
                   ▼          ▼
               Protokol LAP v1 (docs/PROTOCOL.md)
                           │
                    LapEndpoint (core)
                           │  Command / Telemetry
                           ▼
        leumpang-core: Controller (UVC, IK, state machine, keselamatan)
                           │  ServoBus / ImuInput / Platform / Inbox
        ┌──────────────┬───┴──────────┬──────────────────┐
     HAL ESP32-S3   HAL ESP32-P4   HAL STM32H562     HAL simulasi
     (selesai)      (Fase 1)       (Fase 1)          (selesai; MuJoCo Fase 2)
```

## Prinsip

1. **Satu sumber logika.** Semua perilaku robot ada di `core/`. Board, simulator, dan digital twin
   memanggil `Controller::step()` yang sama. Bug yang ditemukan di twin = bug yang sama di robot.
2. **Core tanpa RTOS.** Core tidak tahu FreeRTOS, ThreadX, atau thread. HAL memberi:
   * `Platform` — jam mikrodetik + sink log
   * `ServoBus::exchange()` — kirim frame ICS R+L, kumpulkan reply (≤ 1 ms)
   * `ImuInput` — sampel roll/pitch terakhir
   * `Inbox` — tombol konsol & `Command` (dikuras di awal siklus)
3. **Telemetry keluar lewat salinan.** HAL memanggil `snapshot()` di thread kontrol, menyalinnya di
   bawah lock miliknya; thread lain (web, LCD, LAP) hanya membaca salinan.
4. **Robot = otoritas akhir.** HP dan PC mengirim niat/perintah; controller tetap memeriksa state,
   batas sendi, slew, jatuh, dan watchdog link.
5. **Tidak ada flash/SD write di jalur realtime.** Persistensi ditunda sampai `modePersistSafe()`.

## Struktur repo

| Folder | Isi |
|---|---|
| `core/` | leumpang-core: `types.h`, `ics.h`, `controller.*`, `lap.*`, `lap_endpoint.*` |
| `firmware/components/leumpang_core` | pembungkus komponen ESP-IDF untuk `core/` |
| `firmware/esp/` | firmware ESP-IDF untuk ESP32-S3 & ESP32-P4 (HAL: UART ICS, IMU, LCD, Wi-Fi, web, LAP TCP + serial, SD) |
| `sim/` | `leumpang_sim`: core + model servo + IMU sintetis + server LAP TCP |
| `tools/` | `lap.py` (protokol Python), `lapctl.py` (CLI) |
| `tests/` | unit, fuzz, skenario, **paritas V4↔core**, lintas-bahasa, stub IDF |
| `docs/` | arsitektur, protokol, roadmap |

## Jaminan yang diuji (ctest)

| Tes | Membuktikan |
|---|---|
| `test_parity` | core baru menghasilkan output servo, mode, kinematika, dan IMU **identik bit-per-bit** dengan firmware V4 lama selama 20.816 siklus (skenario + fuzz acak) |
| `test_controller` | boot sync, servo mati, READY→kalibrasi→BALANCE→langkah, jatuh→re-arm, IMU basi, parser ICS (termasuk jebakan echo V3) |
| `test_lap` | CRC/COBS vektor standar, 5000 roundtrip acak, resync setelah sampah, fuzz 2 juta byte, perilaku endpoint |
| `test_lap_python` | klien Python ↔ simulator C++ lewat TCP: handshake, stream, param, trim, jatuh, re-arm, robustness |
| `fw_s3_linkcheck` (build) | seluruh firmware ESP32-S3 + core ter-compile & ter-link terhadap stub API IDF |
| CI `firmware-esp32s3` | build ESP-IDF **asli** v5.3 & v5.4 |
