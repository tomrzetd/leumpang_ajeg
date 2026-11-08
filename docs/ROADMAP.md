# Roadmap

## Fase 0 — Fondasi ✅
* leumpang-core portabel, terbukti identik dengan firmware V4 (`test_parity`)
* Protokol LAP v1 (C++ + Python), endpoint robot bersama
* Firmware ESP32-S3 memakai core + server LAP TCP :5760
* Simulator host, `lapctl`, ctest, CI (host Linux/macOS + build ESP-IDF asli)

## Fase 1 — Firmware tiga board + SD + gait kontinu ✅ (kode; ⏳ uji hardware)
* **ESP32-P4 (FireBeetle 2)**: satu proyek ESP-IDF dengan S3 (`firmware/esp`, pin per board);
  Wi-Fi via ESP32-C6 (esp_hosted 2.13.1, SDIO); konsol + LAP lewat USB-Serial-JTAG; SD slot on-board.
* **STM32H562 (WeAct)**: bare-metal HAL (tanpa RTOS: TIM6 kontrol 100 Hz, TIM7 IMU 500 Hz);
  ICS USART half-duplex + EN; USB CDC (TinyUSB); SDMMC1 + FatFs; config/trim di SD.
* **Link serial bersama** di ketiga board (`SerialLink`): konsol teks dan LAP di port yang sama.
* **SD card**: template gerak `/…/slotN.lam` dimuat saat boot; blackbox 100 Hz (`.lbb`, `bbx2csv.py`).
* **Gait kontinu (mode 800, eksperimental, default mati)**: `GAIT_INTENT` vx (mm/langkah) dan yaw (°/langkah)
  dengan ttl, perubahan dihaluskan per langkah, berhenti otomatis saat niat kedaluwarsa.
* **Motion player (mode 810)**: format LAM1 (≤64 keyframe, cosine/linear/step), unggah lewat `MOTION_*`,
  setiap frame dicek terhadap batas sendi sebelum diputar.
* Driver ICM-20948 + Mahony dipindah ke core (dipakai ketiga board), CI membangun ketiga firmware.

**Ditunda dari Fase 1:** LCD ST7789 & RGB WS2812 di STM32; langkah menyamping (`vy`); `PERCEPTION`.

**Uji hardware yang dibutuhkan (urut):** pin & polaritas EN tiap board → `Boot sync 22/22` digantung →
tanda IMU → BALANCE → motion `lambai` dari READY → baru `gaitEnable=1` di lantai dengan tali pengaman.

## Fase 2 — Studio desktop x86
* Qt 6 (C++) + Qt Quick 3D: digital twin 3D, tuning per servo (trim, limit, center), editor gerakan + timeline.
* Fisika MuJoCo menjalankan **core yang sama** (software-in-the-loop).
* Capture pose webcam (ONNX Runtime: model pose 2D/3D) → retarget lengan/kepala langsung; kaki hanya lolos cek
  titik berat (CoM di dalam poligon tumpuan) di twin.
* Pemutaran ulang blackbox SD di twin.

## Fase 3 — "Mata" Android
* Kotlin + Jetpack Compose; CameraX; model AI di NPU/GPU (pose, deteksi objek, segmentasi lantai).
* Depth: ToF Camera2 `DEPTH16` / ARCore Depth (bila ada di HP); UWB (`androidx.core.uwb`) ke tag beacon.
* Fusi → `PERCEPTION` + `GAIT_INTENT` via USB-OTG (usb-serial-for-android, CDC-ACM).
* Watchdog, mode aman, dan perekaman sesi.

## Keputusan yang masih terbuka
1. Teknologi desktop final (usulan: Qt 6 C++).
2. Sumber model 3D (CAD/STL KHR-3HV asli atau model parametrik).
3. Model HP Android (menentukan ToF/UWB/NPU yang tersedia).
4. Lokasi pemasangan HP (dada/punggung disarankan; kepala menaikkan titik berat).
5. Lisensi repositori.
