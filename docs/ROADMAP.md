# Roadmap

## Fase 0 — Fondasi ✅
* leumpang-core portabel, terbukti identik dengan firmware V4 (`test_parity`)
* Protokol LAP v1 (C++ + Python), endpoint robot bersama
* Firmware ESP32-S3 memakai core + server LAP TCP :5760
* Simulator host, `lapctl`, ctest, CI (host Linux/macOS + build ESP-IDF asli)

## Fase 1 — Firmware tiga board + SD + gait kontinu
* **HAL ESP32-P4 (FireBeetle 2)**: ESP-IDF, berbagi ±80% kode S3; Wi-Fi via ESP32-C6 (esp_hosted); SD SDMMC 4-bit.
* **HAL STM32H562 (WeAct)**: STM32CubeH5 + ThreadX/FreeRTOS; ICS via USART half-duplex satu kabel;
  SDMMC1 (PC8–PC12, PD2), USB CDC (PA11/PA12); tanpa Wi-Fi (dashboard via USB/HP).
* **USB CDC device** di ketiga board (link HP OTG) memakai `LapEndpoint` yang sama.
* **SD card**: FATFS; pustaka gerakan, blackbox telemetry 100 Hz, backup config — ditulis dari task prioritas rendah.
* **Mode gait kontinu** (baru): langkah berulang dengan input `vx, vy, yaw` dari `GAIT_INTENT`,
  watchdog link, batas kecepatan; divalidasi di simulator/twin dulu.
* **Motion player**: format template keyframe + interpolasi, validasi batas posisi/kecepatan, `MOTION_*`.

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
