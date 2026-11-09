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

## Fase 3 — "Mata" Android ✅ (kode + CI; ⏳ uji di HP & robot)
* **Leumpang Mata** (`android/`, Kotlin, Compose): kamera kepala (kamera utama HP yang dipindah, CameraX) atau USB-UVC
  (bila HP mengekspos ke Camera2), MediaPipe (objek, isyarat, wajah + daftar nama), ucapan id-ID + TTS.
* **Hindari rintangan**: ruang bebas berbasis warna lantai + kotak objek → proyeksi lantai dengan tinggi/pitch kamera dari
  telemetry → peta 9 sektor → `PERCEPTION` (refleks firmware) + perencana VFH. **Ikuti orang** dengan jaga jarak 1 m.
  **Isyarat tangan** dan **perintah suara** → arbiter perilaku (satu-satunya yang menggerakkan robot).
* **Firmware core 0.8.0**: refleks rintangan (`obstacleStopMm`), `EXT` untuk modul/sensor tambahan.
* **Leumpang Bus** ([BUS.md](BUS.md)): pub/sub topik di HP, WebSocket JSON `:8765` untuk PC/skrip/board lain, EXT ke robot.
* Teruji: 18 tes Kotlin (termasuk integrasi lawan simulator C++: refleks rintangan nyata), 50 cek controller.

**Belum / berikutnya:** uji di Xiaomi 12 Pro + robot; jalur UVC berbasis libusb untuk HP yang tidak mengekspos kamera
eksternal; kedalaman monokular (model depth) untuk lantai bertekstur ramai; klasifikasi suara lingkungan; pasang
`extHook` sensor nyata di HAL ESP32/STM32; studio sebagai klien Leumpang Bus (lihat apa yang dilihat robot).

## Fase 4 — Platform servo controller (berjalan)
Keputusan: board utama **ESP32-P4**, servo **KRS-2552/3204/3304 (ICS 3.5)**, skrip **MicroPython** di SD,
stik **PS4 lewat ESP32 klasik** (Bluepad32), Wi-Fi hanya untuk desktop, HP hanya lewat USB OTG.

* **4A ✅ (kode + simulasi; ⏳ uji hardware)** — core 0.9.0
  * Profil robot `config/robot.json` ([PROFILE.md](PROFILE.md)): `humanoid-khr` atau `generic` (robot lain), ID/arah/batas/HOME per servo.
  * Layanan berkas SD lewat LAP (`FILE_*`), `CMD reboot`; ESP P4/S3 (VFS) dan STM32 (FatFs LFN).
  * Alat servo ICS 3.5 (mode 830, `SERVO_TOOL`): scan ID, baca/tulis ID, stretch/speed/batas arus/suhu, baca EEPROM, posisi.
  * `lapctl`: `ls/get/put/rm/mkdir/mv`, `profile get|put`, `servo scan|id|setid|get|set|eeprom|pos`.
  * Teruji: profil 19 cek, studio/LAP 68 cek, Python lintas-bahasa (lapctl lawan `leumpang_sim --sd`).
* **4B ✅ (kode + simulasi + build CI; ⏳ uji dengan stik)** — core 0.10.0 — Leumpang Radio ([RADIO.md](RADIO.md))
  * Firmware radio **ESP32 klasik** (WROOM-32 / PICO-D4) dan **Pico 2 W** (juga Pico W), Bluepad32 4.2.0, kode bersama.
  * LAP `PAD` 50 Hz / `PAD_FEEDBACK` (lightbar per mode, getar saat jatuh). UART radio di P4 (UART3), STM32 (USART1), S3 (opsional).
  * Pemetaan di core: PS/touchpad = E-STOP, L1 deadman, X = GO, stik = jalan/belok, segitiga/kotak/d-pad = motion 0..5.
  * `lapctl pad` (PC sebagai radio). Teruji: 17 cek skenario stik di simulator + uji lintas-bahasa.
  * Berikutnya: jembatan Wi-Fi lewat board radio untuk STM32.
* **4C** MicroPython tertanam + modul `leumpang` + `autorun.py` + Run/Stop/konsol dari desktop.
* **4D** Studio v2: pengelola berkas, wizard kalibrasi, editor skrip, editor node (→ MicroPython), simulasi gait realtime, profil dari robot.
* **4E** Indra HP (rintangan, orang, isyarat, suara) bisa dibaca skrip.
