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
     FreeRTOS       FreeRTOS       bare-metal ISR    host / di dalam studio
     Wi-Fi + UART   Wi-Fi(C6)+USB  USB CDC
        └──────────────┴──────┬───────┘
                       SD card: motion LAM1, blackbox .lbb, (STM32) config
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
   Blackbox keluar dari thread kontrol lewat ring SPSC tanpa lock (`blackbox.h`); slot motion
   berpindah antar-thread lewat state atomik (`MotionStore`: EMPTY→WRITING→READY→PLAYING).
6. **Driver bersama bila bisa.** ICM-20948 + Mahony (`icm20948.h`) dan multiplexer konsol/LAP
   (`lap_serial.h`) ada di core; HAL hanya memberi fungsi baca/tulis I2C atau byte serial.

## Struktur repo

| Folder | Isi |
|---|---|
| `core/` | leumpang-core: `types.h`, `ics.h`, `controller.*`, `lap.*`, `lap_endpoint.*`, `lap_serial.*`, `motion.*`, `blackbox.h`, `icm20948.*` |
| `firmware/components/leumpang_core` | pembungkus komponen ESP-IDF untuk `core/` |
| `firmware/esp/` | firmware ESP-IDF untuk ESP32-S3 & ESP32-P4 (HAL: UART ICS, IMU, LCD, Wi-Fi, web, LAP TCP + serial, SD) |
| `firmware/stm32h562/` | firmware bare-metal STM32H562 (HAL ST, TinyUSB CDC, FatFs) |
| `firmware/third_party/esp_hosted` | submodule esp_hosted v2.13.1 (Wi-Fi P4 via C6) |
| `sim/` | `leumpang_sim`: core + model servo + IMU sintetis + server LAP TCP |
| `tools/` | `lap.py` (protokol Python), `lapctl.py` (CLI), `lam.py` (JSON→LAM1), `bbx2csv.py` |
| `motions/` | contoh template gerak (`lambai.json`) |
| `studio/` | Leumpang Studio: `lib/` tanpa Qt (FK, retarget, MotionDoc, LapClient, SimRobot, PoseOnnx), `app/` backend Qt, `qml/` UI |
| `model/` | mesh STL + URDF, dibangkitkan `tools/gen_model.py` (juga `studio/lib/robot_model_gen.h`) |
| `tests/` | unit, fuzz, skenario, **paritas V4↔core**, lintas-bahasa, stub IDF |
| `docs/` | arsitektur, protokol, roadmap |

## Jaminan yang diuji (ctest)

| Tes | Membuktikan |
|---|---|
| `test_parity` | core baru menghasilkan output servo, mode, kinematika, dan IMU **identik bit-per-bit** dengan firmware V4 lama selama 20.816 siklus (skenario + fuzz acak) |
| `test_controller` | boot sync, servo mati, READY→kalibrasi→BALANCE→langkah, jatuh→re-arm, IMU basi, parser ICS (termasuk jebakan echo V3) |
| `test_lap` | CRC/COBS vektor standar, 5000 roundtrip acak, resync setelah sampah, fuzz 2 juta byte, perilaku endpoint |
| `test_lap_python` | klien Python ↔ simulator C++ lewat TCP: handshake, stream, param, trim, jatuh, re-arm, robustness |
| `test_studio` | FK twin = IK core (BALANCE & WALK_CONT), retarget kamera 2D → sendi → FK (242/242), pratinjau gerak studio = pemutar firmware, RDP, MoveNet, LapClient ↔ SimRobot (unggah, putar, live pose) |
| CI `studio-linux` / `studio-windows` | build Qt 6.8 + `--selftest` (simulator, gerak 810, live pose 820, tangkapan layar 7 tab, nol warning QML) |
| `test_imu` | driver ICM-20948 (register sintetis) + konvergensi Mahony, bias gyro, penolakan impak |
| `fw_s3_linkcheck`, `fw_p4_linkcheck` (build) | seluruh firmware ESP + core ter-compile & ter-link terhadap stub API IDF |
| CI `firmware-esp` | build ESP-IDF **asli**: S3 v5.3.2 & v5.4.1, P4 v5.5.1 |
| CI `firmware-stm32` | build arm-none-eabi-gcc 13.3 → `.elf/.bin/.hex` |
