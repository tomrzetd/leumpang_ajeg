# Leumpang Ajeg

**Leumpang ajeg** (bahasa Sunda: *berjalan dengan mantap*) — kontroler keseimbangan dan langkah UVC
untuk robot humanoid KHR, dengan inti portabel untuk ESP32-S3, ESP32-P4, dan STM32H562,
simulator, serta protokol bersama untuk HP Android ("mata") dan studio desktop (digital twin).

> *Leumpang Ajeg — UVC balance & walking controller for a KHR humanoid. Portable C++ core, ESP-IDF firmware,
> host simulator, and the LAP protocol for Android perception and a desktop digital twin.*

## Status

| Bagian | Status |
|---|---|
| leumpang-core (UVC, IK, state machine, keselamatan) | ✅ terbukti identik dengan firmware V4 |
| Protokol LAP v1 (C++ / Python) | ✅ |
| Firmware ESP32-S3 (ESP-IDF) | ✅ kode; ⏳ uji hardware |
| Simulator host + `lapctl` | ✅ |
| Firmware ESP32-P4, STM32H562, SD card, gait kontinu | Fase 1 |
| Studio desktop + digital twin | Fase 2 |
| Aplikasi Android "mata" | Fase 3 |

Lihat [docs/ROADMAP.md](docs/ROADMAP.md), [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Mulai cepat (tanpa hardware)

```bash
cmake -B build && cmake --build build -j
ctest --test-dir build --output-on-failure        # semua tes

./build/leumpang_sim                              # simulator robot, LAP di 127.0.0.1:5760
python3 tools/lapctl.py --tcp 127.0.0.1 watch --servo
python3 tools/lapctl.py --tcp 127.0.0.1 cmd go
python3 tools/lapctl.py --tcp 127.0.0.1 sim-imu 0 40
```
Di terminal simulator Anda juga bisa mengetik perintah konsol robot (`r g x c t y z ?`).

## Firmware ESP32-S3

```bash
. $IDF_PATH/export.sh                             # ESP-IDF >= 5.3
cd firmware/esp32s3
idf.py set-target esp32s3
idf.py build flash monitor
```
SoftAP `LEUMPANG-AJEG-xxxx`, dashboard `http://192.168.4.1` / `http://leumpang-ajeg.local`,
LAP TCP `192.168.4.1:5760`. Detail wiring, temuan review, dan prosedur uji pertama:
[firmware/esp32s3/README.md](firmware/esp32s3/README.md).

⚠️ Ganti `WIFI_AP_PASS` dan `DASH_PASS` di `firmware/esp32s3/main/app_config.h`, dan persempit
batas `lo/hi` tiap sendi di `core/src/controller.cpp` (`JOINTS`) sebelum uji lantai.
