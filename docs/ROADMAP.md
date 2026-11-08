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

## Fase 2 — Studio desktop x86 ✅ (Windows + Linux, CI + selftest)
* **Leumpang Studio** (Qt 6.8, C++/QML, `studio/`): twin 3D Qt Quick 3D (bayangan, AO, hantu umpan balik),
  panel 22 servo (pose kerja, perintah vs aktual, trim), editor gerak (≤ 64 keyframe, pratinjau identik firmware,
  JSON/LAM1, unggah/putar), kamera → pose (MoveNet/ONNX Runtime) → lengan + kepala, rekam → keyframe (RDP),
  jalan kontinu, parameter, log. Koneksi: simulator di dalam studio, Wi-Fi, USB serial.
* **Model 3D parametrik** (`tools/gen_model.py` → `model/`): 24 link, 82 mesh STL, URDF, tabel FK C++ — satu sumber.
  FK terbukti cocok dengan IK core (tinggi kaki 185,01 mm; sol miring < 0,04° saat jalan).
* **Mode 820 LIVE_POSE** + `POSE_TARGET` di core: robot mengikuti pose studio/kamera dengan batas sendi,
  slew servo, deteksi jatuh, dan ttl.
* Paket Windows otomatis (artefak CI `LeumpangStudio-windows-x64`), lisensi MIT.

**Ditunda dari Fase 2:** fisika MuJoCo (URDF sudah siap), pemutaran ulang blackbox `.lbb` di twin,
cek titik berat untuk gerak kaki dari editor, model pose 3D (sekarang 2D + estimasi kedalaman dari pemendekan).

## Fase 3 — "Mata" Android (Xiaomi 12 Pro + Redmi Note 11 Pro 5G)
Fakta HP (lembar spesifikasi): **Xiaomi 12 Pro** — Snapdragon 8 Gen 1, USB-C 2.0 + OTG, 205 g,
163,6 × 74,6 × 8,2 mm, **tanpa ToF dan tanpa UWB**. **Redmi Note 11 Pro 5G** — Snapdragon 695, USB-C 2.0 + OTG,
202 g, juga tanpa ToF/UWB. Konsekuensinya "radar" dibuat dari AI, bukan sensor khusus:
* Xiaomi 12 Pro = HP utama: CameraX, pose/objek/segmentasi lantai di NPU/GPU (TFLite + delegate GPU/NNAPI atau QNN),
  **kedalaman monokular** (model depth) + ARCore Depth bila didukung, IMU HP sebagai referensi kedua.
* Redmi Note 11 Pro 5G = cadangan / kamera kedua (stereo kasar atau pengamat dari luar), lewat Wi-Fi.
* Fusi → `PERCEPTION` + `GAIT_INTENT` via USB-OTG (CDC-ACM, usb-serial-for-android); watchdog & mode aman.

**Soal kamera di kepala:** modul kamera internal HP tersambung kabel flex MIPI beberapa cm ke mainboard; memindahkannya
ke kepala praktis tidak mungkin tanpa merusak HP. Pilihan yang realistis:
1. **Kamera USB (UVC) kecil di kepala** + hub OTG yang mendukung pengisian (HP tetap di punggung): satu port USB 2.0 HP
   dibagi kamera MJPEG 720p30 dan serial robot. Android membaca UVC lewat pustaka UVC/`libuvc`.
2. HP dipasang tegak sebagai "wajah" (kamera belakang menghadap depan) — paling sederhana, tetapi 205 g di kepala
   menaikkan titik berat; perlu dicek di twin (opsi HP sudah ada di model).
3. Periskop cermin dari kamera HP di punggung ke atas — murah, tetapi rapuh dan gambar terbalik.

## Keputusan yang masih terbuka (Fase 3)
1. Opsi kamera kepala (1 / 2 / 3 di atas).
2. Hub OTG + catu: apakah HP ikut diisi dari baterai robot?
3. Prioritas persepsi pertama: hindari rintangan, ikuti orang, atau kenali objek/isyarat tangan.
