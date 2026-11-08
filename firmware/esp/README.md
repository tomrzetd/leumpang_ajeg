# Leumpang Ajeg — firmware ESP32-S3 N16R8 (ESP-IDF)

> Sejak Fase 0, logika robot ada di `core/` (leumpang-core). Folder ini berisi HAL board:
> UART ICS, IMU ICM-20948, LCD, RGB, Wi-Fi/web, LAP TCP :5760, NVS.

Port dari `biped_esp32s3_n16r8_dashboard_v3.cpp` (Arduino) ke **ESP-IDF ≥ 5.3**.
Rumus gait UVC, IK, state machine, nilai mode (700…791), center sendi, dan pemetaan bus **tidak diubah**.
Perubahan diberi tag `[V4-xx]` di kode.

## 1. Apakah port ke ESP-IDF layak?

Ya, tapi alasannya bukan "C lebih cepat dari Arduino" (Arduino-ESP32 sendiri berjalan di atas IDF).
Keuntungan nyatanya adalah **determinisme waktu di core1**:

| Area | V3 (Arduino) | V4 (ESP-IDF) |
|---|---|---|
| IMU | Library DMP dipanggil di loop kontrol (≤4× `task()` + I2C, timeout Wire 5 ms) | Task IMU terpisah di core0, I2C berbasis interrupt, fusi Mahony 500 Hz; loop kontrol hanya menyalin sampel |
| Bus ICS | `flush()` + `delayMicroseconds(200)` tetap per slot | Tunggu TX via polling, **early-exit** begitu reply valid, batas 300 µs per slot |
| LCD | Adafruit GFX, banyak transaksi SPI kecil di-polling CPU | `esp_lcd` + canvas RAM + 1 transfer DMA per region |
| NVS | Ditulis langsung dari handler HTTP (bisa saat robot berjalan) | Ditunda sampai mode aman; driver Wi-Fi `WIFI_STORAGE_RAM` |
| Isolasi core | Wi-Fi/web/LCD di core0, tetapi IMU I2C ikut di loop core1 | Core1 hanya berisi task kontrol + ISR UART-nya sendiri |

Hasil simulasi host (model servo dengan latensi respons 60 µs, termasuk 1 servo mati yang kena timeout):
fase ICS 22 servo ≈ **1,5 ms**. Di V3, delay tetapnya saja sudah 11 × (26 µs TX + 200 µs) ≈ 2,5 ms,
ditambah waktu I2C DMP. **Angka hardware harus diukur**: dashboard menampilkan `loop avg/max`, `ICS phase`,
dan GPIO5 (`PIN_T2`) HIGH selama satu siklus untuk logic analyzer.

## 2. Temuan review kode V3 (dan perbaikannya)

| # | Tingkat | Masalah di V3 | Perbaikan V4 |
|---|---|---|---|
| 1 | Tinggi | `handleConfigPost`/`handleWifiPost` menulis NVS saat robot bisa sedang berjalan. Tulis/erase flash mematikan cache **kedua** core → task kontrol berhenti puluhan ms. | Persistensi ditunda (`cfgGen` vs `savedGen`) sampai READY/FALLEN; Wi-Fi memakai `WIFI_STORAGE_RAM` |
| 2 | Tinggi | `primeOutputs()` mengasumsikan servo sudah di HOME. Servo ICS menyala FREE di posisi sembarang → frame pertama melompat ke HOME dengan kecepatan penuh (slew limiter dihitung dari posisi *anggapan*). | `bootSync()`: frame FREE 120 ms untuk membaca posisi aktual → baseline → ramp 1 s ke HOME |
| 3 | Tinggi | Diagram menyambung EN_R/EN_L (GPIO6/7), tetapi default `ICS_USE_DIR_PINS 0` → EN tidak pernah dikendalikan, reply tidak terbaca. | `ICS_DIR_MODE 1` (GPIO EN) default; mode 2 = RS485 hardware (RTS) |
| 4 | Sedang | Trim (`k/u/a/h/e`) mengubah `servos.*` langsung → ditimpa `rampTo(HOME)` dan IK; tidak pernah tersimpan. | Tabel `g_trim[2][11]` terpisah, ditambahkan di tahap output, disimpan ke NVS, bisa diatur dari web |
| 5 | Sedang | Feedback di luar 3500..11500 ditolak; servo FREE yang digerakkan tangan bisa berada di luar itu → sync re-arm gagal. | Rentang feedback 1..16383, di-clamp ke limit sendi saat sync |
| 6 | Sedang | Parser reply "ambil match terakhir" bisa salah-cocok pada `echo[1..2] + reply[0]`. Laten di V3 (delay tetap), fatal bila early-exit. | Parser presisi: reply harus di offset 0 atau tepat setelah echo yang identik dengan frame terkirim |
| 7 | Sedang | CSRF: Basic auth di-cache browser, halaman lain bisa POST `/api/cmd` (mis. `go`). | POST wajib header `X-KHR: 1` (memicu preflight CORS yang tidak dijawab) |
| 8 | Sedang | Sign/swap IMU bisa diubah dari web saat BALANCE/berjalan → lonjakan roll/pitch → pemicu langkah/jatuh palsu. | Parameter frame IMU hanya diterima di READY/FALLEN |
| 9 | Rendah | Deteksi jatuh tidak aktif di POSE_HOLD dan MIRROR_DBG (kaki ber-torsi). | Ditambahkan (ANGLE/RAW tetap dikecualikan karena robot dipegang tangan) |
| 10 | Rendah | JSON dibangun dengan ratusan `String +=` (fragmentasi heap). | Satu buffer statis + `snprintf` |
| 11 | Rendah | STA gagal konek → scan berulang tanpa jeda, radio pindah kanal, klien SoftAP putus-putus. | Reconnect dengan backoff 3 s → 60 s |
| 12 | Rendah | Kepala figur pose LCD terpotong di tepi region. | Titik pinggul diturunkan |

Belum diubah (perlu keputusan Anda): `swCont()` masih **rekonstruksi** (lihat `SWCONT_LEGACY`), dan
batas `lo/hi` tiap sendi (`JOINTS` di `core/src/controller.cpp`) masih rentang penuh protokol — **wajib dipersempit** sebelum uji lantai.

## 3. Catatan desain hardware (dari diagram wiring)

1. **Tegangan servo.** Diagram menulis 5–6 V. Bila servo Anda seri HV (mis. KRS-2552RHV / KRS-3304), rating-nya 9–12 V; 5–6 V hanya untuk seri non-HV. Cek label servo.
2. **WS2812B di 5 V dengan data 3,3 V** berada di batas (VIH ≈ 0,7 × VDD = 3,5 V). Pakai buffer 74AHCT1G125, atau beri daya LED lewat dioda (≈4,3 V). Kapasitor 1000 µF berlebihan untuk 1 LED (100 nF + 10 µF cukup) — kapasitor besar (≥2200 µF low-ESR) seharusnya di catu servo.
3. **SPI LCD tidak di pin IOMUX.** FSPI IOMUX ESP32-S3: CS0=GPIO10, MOSI=GPIO11, CLK=GPIO12. Dengan SCK=10/CS=13 sinyal lewat GPIO matrix → aman di 40 MHz. Pindahkan SCK→12 dan CS→10 bila ingin 80 MHz.
4. **LCD 1.69"** umumnya 240×280, bukan 240×320: set `LCD_W 280` dan `LCD_GAP_X 20` di `app_config.h`.
5. **INT IMU** boleh disambung ke GPIO1 (bebas) lalu set `ICM_INT_GPIO 1` → sampel dipicu data-ready, bukan polling.
6. **Ground bersama**: jadikan satu titik bintang di terminal catu servo; arus balik servo jangan lewat jalur GND board ESP32/LCD. Pasang sekring di catu servo.
7. GPIO2 (heartbeat) dan GPIO5 (pin ukur jitter) dipakai kode tetapi tidak ada di diagram.
8. Saat hanya USB-C yang menyala (catu servo mati), TX/EN ESP32 masih aktif ke driver ICS — pastikan driver tidak mencatu-balik servo lewat pin sinyal.

## 4. Perubahan perilaku yang harus diketahui

* **IMU sekarang raw accel+gyro + Mahony**, bukan DMP. Konvensi sumbu sama (roll = sumbu X chip, pitch = sumbu Y), tetapi **ulangi uji tanda** (miringkan robot ke kanan/depan, cek tanda di dashboard; atur Roll/Pitch sign dan Swap di READY).
* Saat boot: robot harus **digantung/disangga**; servo dibaca dulu (log `Boot sync: n/22`). Bila `n<22`, cek jalur RX/EN.
* Trim tersimpan permanen; perubahan konfigurasi tampil `NVS: PENDING` sampai robot READY/FALLEN.
* Perintah serial sama (`r g x c t y z p k/u/a/h/e ?`) lewat `idf.py monitor`.

## 5. Build & flash

```bash
. $IDF_PATH/export.sh            # ESP-IDF v5.3 / v5.4 / v5.5
cd firmware/esp32s3
idf.py set-target esp32s3
idf.py build flash monitor
```
Ganti `WIFI_AP_PASS` dan `DASH_PASS` di `main/app_config.h` sebelum dipakai.
Komponen `espressif/mdns` diunduh otomatis; tanpa internet set `APP_USE_MDNS 0` dan hapus barisnya di `main/idf_component.yml`.

## 6. Uji pertama (urut)

1. Tanpa catu servo: cek log IMU (`ICM-20948 OK`, `500 Hz`), dashboard, LCD, warna RGB.
2. Logic analyzer di TX/RX/EN bus R: frame 3 byte 1,25 Mbps 8E1, EN HIGH hanya selama TX, reply 3 byte. Cek polaritas EN.
3. Robot digantung, catu servo ber-limit arus: boot → `Boot sync 22/22` → ramp halus ke HOME.
4. Uji tanda IMU, kalibrasi (`c`), persempit `lo/hi` per sendi di `JOINTS` (`core/src/controller.cpp`).
5. Ukur `loop max` dan `ICS phase` di dashboard; `overrun` harus tetap 0.
6. Baru kemudian BALANCE (`g`) di lantai dengan tali pengaman.

## 7. Status verifikasi

* **Belum dikompilasi dengan toolchain ESP-IDF asli** (tidak tersedia di lingkungan pembuatan) dan belum diuji di hardware.
* Lulus `g++`/`clang++ -Wall -Wextra` terhadap stub API IDF v5.3 (`tests/idf_stubs`), semua varian konfigurasi, dan link seluruh modul (`fw_s3_linkcheck`).
* CI GitHub Actions membangun firmware ini dengan ESP-IDF asli v5.3 & v5.4 (`.github/workflows/ci.yml`).
* Simulasi host (`tests/test_controller.cpp`, `tests/test_parity.cpp`, ASan+UBSan): boot-sync, servo mati, echo/tanpa echo, READY→kalibrasi→BALANCE→siklus langkah 740–770, jatuh→re-arm, trim, penolakan parameter frame IMU — semua lulus.
* Uji filter Mahony (konvergensi, integrasi gyro, penolakan impak 2 g) lulus; render LCD dicek visual.
* API yang paling perlu dicek saat build pertama: `uart_wait_tx_idle_polling` (ganti `uart_wait_tx_done(port, 1)` bila tidak ada), field `rgb_ele_order` LCD, dan orientasi/inversi panel (`LCD_*` di `app_config.h`).

## Struktur

```
firmware/esp32s3/
├─ CMakeLists.txt, sdkconfig.defaults, partitions.csv
└─ main/
   ├─ app_config.h      pin & konfigurasi board
   ├─ shared.h          antarmuka antar-modul (nama pendek dari leumpang-core)
   ├─ control.cpp       HAL kontrol: UART ICS (EspIcsBus), queue, task 100 Hz di core1
   ├─ imu_icm20948.cpp  driver ICM-20948 + Mahony (core0)
   ├─ web.cpp           Wi-Fi AP+STA, esp_http_server, dashboard
   ├─ lap_tcp.cpp       server protokol LAP :5760
   ├─ local_ui.cpp      ST7789 esp_lcd DMA + WS2812 RMT
   ├─ font5x7.h
   └─ main.cpp          app_main, logger, konsol, NVS tertunda
```
Logika gait/keselamatan: `../../core/`. Komponen IDF pembungkusnya: `../components/leumpang_core`.
