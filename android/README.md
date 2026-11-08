# Leumpang Mata — aplikasi Android "indera" robot

HP = mata, telinga, dan mulut robot; firmware leumpang-ajeg tetap mengurus keseimbangan dan keselamatan.
Diuji untuk dipasang di **Xiaomi 12 Pro** (Snapdragon 8 Gen 1) dan **Redmi Note 11 Pro 5G** (Snapdragon 695), Android ≥ 8.

| Kemampuan | Cara kerja | Topik bus |
|---|---|---|
| **Hindari rintangan** (prioritas 1) | Ruang bebas berbasis warna lantai + kotak objek → diproyeksikan ke lantai memakai tinggi/kemiringan kamera kepala dari telemetry robot → peta 9 sektor. Dikirim ke robot sebagai `PERCEPTION` (refleks henti firmware) dan dipakai perencana langkah. | `perception/obstacles` |
| **Ikuti orang** (2) | Detektor objek "person" → arah + jarak (dari kaki di lantai, atau tinggi badan) → pengikut menjaga jarak 1 m sambil menghindari rintangan. | `perception/person` |
| **Isyarat tangan** (3) | MediaPipe Gesture Recognizer: telapak/kepal = berhenti, jempol = ikuti, telunjuk ke atas = jelajah, V = lambai, jempol bawah = HOME. | `vision/gesture` |
| **Wajah & objek** | MediaPipe Face Detector + embedding MobileNetV3 (daftarkan nama di aplikasi; pengenalan sederhana, bukan biometrik), Object Detector EfficientDet-Lite0 (80 kelas COCO). | `vision/faces`, `vision/objects` |
| **Perintah suara** | `SpeechRecognizer` id-ID (utamakan offline). "robot berhenti", "ikuti saya", "jalan jalan", "belok kiri", "mundur", "lambaikan tangan", "apa yang kamu lihat", "siapa kamu" (+ beberapa kata Sunda). | `speech/heard` |
| **Bicara** | `TextToSpeech` Bahasa Indonesia: konfirmasi perintah, "Ada rintangan di depan", "Aku jatuh...", deskripsi pemandangan. | `speech/say` |
| **API modul** | Leumpang Bus: pub/sub topik di dalam HP + WebSocket JSON `:8765` untuk PC/skrip/board lain + `EXT` ke board robot. Lihat [docs/BUS.md](../docs/BUS.md). | semua |

## Pasang

1. Unduh APK dari GitHub **Actions** → run terbaru → artefak **LeumpangMata-apk** → pasang (izinkan "sumber tidak dikenal").
2. Pertama kali: izinkan kamera + mikrofon; aplikasi mengunduh model MediaPipe (±15 MB, perlu internet sekali).
3. Suara offline: Pengaturan Android → Google → Ucapan / *Speech services* → unduh **Bahasa Indonesia**;
   TTS: *Text-to-speech* → Google → pasang suara Bahasa Indonesia.
4. Colok board robot ke port USB-C HP lewat OTG → aplikasi menyambung otomatis (STM32/P4: CDC; S3: CP210x/CH340).
   Atau tombol **Wi-Fi** ke ESP32 (`192.168.4.1`).
5. Di robot (sekali): `gaitEnable = 1` (tombol di aplikasi atau studio), lalu **Seimbang**. Mulai uji dengan robot digantung/tali pengaman.

## Kamera kepala

* **Kamera utama HP yang dipindah ke kepala** (konektor + kabel flex pengganti): bagi Android tetap kamera **belakang**
  (id `0`). Atur di aplikasi: *Rotasi tambahan* (bila modul terpasang miring), *Kemiringan kamera ke bawah*
  (ukur sudut sumbu lensa terhadap horizontal, mis. 20°), *Tinggi lensa di atas sumbu kaki* (default 221,5 mm =
  model 3D; ukur robot Anda: tinggi lensa dari lantai saat HOME dikurangi 185 mm).
  Cek: garis biru (horizon) harus jatuh di horizon nyata; titik merah = batas rintangan.
  Kabel flex MIPI yang diperpanjang rawan derau/gangguan sinyal — bila gambar bergaris/hilang, pendekkan kabel dan beri pelindung.
* **USB-UVC**: kamera USB muncul di daftar sebagai "USB/UVC eksternal" **hanya bila HP mengekspos kamera eksternal ke
  Camera2** (dukungan HAL vendor, umumnya Android 14+). Bila tidak muncul, HP itu tidak mendukung UVC lewat API standar;
  jalurnya adalah pustaka UVC berbasis libusb (mis. AndroidUSBCamera) yang dibungkus sebagai sumber gambar baru —
  `CameraSource` dirancang agar bisa diganti. Satu port USB-C HP dibagi kamera + robot lewat hub.

## Daya & kabel (HP diisi dari baterai robot)

* HP host OTG **dan** diisi bersamaan butuh hub USB-C dengan **PD pass-through** (bukan kabel OTG biasa). Uji dulu dengan HP
  Anda: tidak semua HP mau mengisi daya sambil menjadi host.
* Catu 5 V ≥ 3 A dari baterai robot lewat buck terpisah dari jalur servo (servo menarik arus besar dan menimbulkan derau);
  beri LC filter / ferrite di masukan hub. Ground HP, hub, dan board robot jadi satu (via USB) — hindari jalur arus servo
  lewat kabel USB.
* HP ±205 g di punggung menaikkan titik berat; cek di Leumpang Studio (opsi "HP di punggung" di twin).

## Arsitektur kode

```
android/
├─ lap/          Kotlin JVM: protokol LAP (identik C++), LapClient          — tes + integrasi lawan leumpang_sim
├─ bus/          Kotlin JVM: Bus, ModuleHost, BusSession (WebSocket JSON)    — tes
├─ perception/   Kotlin JVM: geometri kamera, ruang bebas, peta rintangan,
│                perencana, pengikut, ucapan, isyarat, arbiter perilaku      — tes (gambar sintetis)
└─ app/          Android: CameraSource (CameraX/Camera2), VisionModule (MediaPipe), ObstacleModule,
                 BehaviorModule, SpeechModule, RobotLinkModule (USB-OTG/TCP), BusServer, UI Compose
```
Build: `gradle -p android :app:assembleDebug` (JDK 17, Android SDK 35). Tes: `gradle -p android :lap:test :bus:test :perception:test`
(tes integrasi aktif bila `leumpang_sim --gait` berjalan di port 5760).

Lisensi pihak ketiga: MediaPipe (Apache-2.0), usb-serial-for-android (MIT), Java-WebSocket (MIT), model MediaPipe (Apache-2.0).
