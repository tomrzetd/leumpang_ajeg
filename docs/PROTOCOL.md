# LAP — Leumpang Ajeg Protocol v1

Satu protokol untuk semua link: HP Android (USB-OTG CDC), aplikasi desktop (USB / TCP Wi-Fi),
simulator (TCP), dan alat baris-perintah. Implementasi referensi:

* C++: `core/include/leumpang/lap.h`, `core/src/lap.cpp`, endpoint robot `lap_endpoint.*`
* Python: `tools/lap.py` (stdlib saja), klien `tools/lapctl.py`
* Kotlin (Android): Fase 3

Transport: TCP :5760 (ESP32-S3/P4 via Wi-Fi, simulator) dan serial (USB-Serial-JTAG P4, UART S3,
USB CDC STM32). Di serial, protokol berbagi port dengan konsol teks: byte `0x00` pertama dari host
memindahkan port ke mode LAP; kembali ke teks 3 s setelah host diam.

## 1. Framing

```
kabel:   COBS( type | seq | body[0..240] | crc16 ) 0x00
```

| Field | Ukuran | Keterangan |
|---|---|---|
| `type` | 1 | jenis pesan (tabel §3) |
| `seq`  | 1 | nomor urut pengirim, wrap 255→0; dipakai `ACK` |
| `body` | 0–240 | little-endian, tanpa padding |
| `crc16`| 2 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) atas `type..body`, little-endian |

* COBS menjamin tidak ada byte `0x00` di dalam frame; `0x00` hanya pembatas.
* Penerima membuang frame yang gagal COBS/CRC dan terus berjalan — teks log atau byte acak
  di stream yang sama tidak merusak sinkronisasi.
* Port TCP default **5760**. USB CDC: baud diabaikan (virtual).

## 2. Aturan link

1. Host mengirim `HELLO` → robot menjawab `HELLO` + `CONFIG`, stream aktif (default `TELEMETRY` 20 Hz, `SERVO_STATE` 5 Hz).
2. Host mengatur laju dengan `STREAM_CFG`.
3. Host wajib mengirim paket apa pun (cukup `PING`) **minimal tiap 3 s**. Tanpa itu stream berhenti.
   `GAIT_INTENT` punya `ttlMs` sendiri (≤ 1000 ms): host yang berhenti mengirim membuat robot
   melambat lalu berhenti melangkah, tanpa menunggu link timeout.
4. Perintah dibalas `ACK`. `ACK OK` berarti **masuk antrean kontrol**; aturan keselamatan state
   (mis. trim hanya di READY) tetap diperiksa controller dan hasilnya terlihat di telemetry/log.
5. Robot adalah otoritas akhir: deteksi jatuh, batas sendi, dan slew limit tidak bisa dimatikan lewat protokol.

## 3. Pesan

Satuan: sudut **derajat** (f32), panjang **mm**, waktu **ms** kecuali disebut.

| Hex | Nama | Arah | Body |
|---|---|---|---|
| 01 | HELLO | ↔ | `u8 protoVer, u8 role, u8 board, u8 fwMaj, u8 fwMin, u8 fwPatch, u32 caps, char name[16]` (26 B) |
| 02 | PING | ↔ | bebas (digemakan di PONG) |
| 03 | PONG | ↔ | salinan body PING |
| 04 | ACK | robot→host | `u8 ackSeq, u8 ackType, u8 status` |
| 10 | TELEMETRY | robot→host | `u32 ms, u16 mode, u16 flags, f32 roll, f32 pitch, i16 imuAgeMs, u16 imuHz, f32 fwct, f32 autoH, u16 loopAvgUs, u16 loopMaxUs, u16 servoPhaseUs, u32 overruns, u8 fbOk, u32 cfgGen` (43 B) |
| 11 | SERVO_STATE | robot→host | 22 × (`i16 cmd, i16 fb, i16 trim, u8 valid`), urutan bus R slot 0..10 lalu bus L (154 B) |
| 12 | GAIT_STATE | robot→host | `f32 fwct, fwctEnd, dxi, dyi, dxis, dyis, autoH, fh, swx, swy, swMax, waist; i8 jikuasi; i16 kin[22]` (93 B) |
| 13 | CONFIG | robot→host | `u8 n, f32 value[n], u32 cfgGen, u32 savedGen` — urutan = ParamId |
| 14 | LOG | robot→host | teks UTF-8 (tanpa NUL) |
| 20 | CMD | host→robot | `u8 cmd`: 0 reset, 1 go, 2 stop (servo free), 3 kalibrasi IMU, 4 kirim CONFIG, 5 kirim HELLO, 6 kirim MOTION_STATUS |
| 21 | SET_PARAM | host→robot | `u8 param, f32 value` |
| 22 | SET_TRIM | host→robot | `u8 bus, u8 slot, i16 value` (±1000, hanya diterapkan di READY) |
| 23 | STREAM_CFG | host→robot | `u8 teleHz(0–50), u8 servoHz(0–20), u8 gaitHz(0–50), u8 logOn` |
| 30 | GAIT_INTENT | HP→robot | `f32 vx, f32 vy, f32 yawDeg, u16 ttlMs` (14 B) — §3.2 |
| 31 | PERCEPTION | HP→robot | daftar rintangan — **Fase 3** (sekarang `UNSUPPORTED`) |
| 40 | MOTION_BEGIN | host→robot | `u8 slot, u16 total` — mulai unggah LAM1 sebesar `total` byte (§3.3) |
| 41 | MOTION_CHUNK | host→robot | `u8 slot, u16 offset, u8 data[≤237]` — harus berurutan |
| 42 | MOTION_END | host→robot | `u8 slot, u8 save` — validasi; `save=1` juga tulis ke SD |
| 43 | MOTION_PLAY | host→robot | `u8 slot, u8 play` — 1 putar (hanya dari READY), 0 berhenti |
| 44 | MOTION_STATUS | robot→host | 4 × (`u8 state, u8 nFrames, char name[16]`) (72 B); state 0 kosong, 1 menulis, 2 siap, 3 diputar |
| 70 | SIM_IMU | host→sim | `f32 roll, f32 pitch, u8 sensorOk` |
| 71 | SIM_FAULT | host→sim | `u8 bus, u8 slot, u8 dead` |

### Nilai enum

* `role`: 0 robot, 1 phone, 2 desktop, 3 sim, 4 tool
* `board`: 0 unknown, 1 esp32s3, 2 esp32p4, 3 stm32h562, 4 sim
* `caps` (bit): 0 Wi-Fi, 1 SD, 2 gait intent, 3 motion, 4 sim, 5 LCD, 6 USB CDC
* `flags` TELEMETRY (bit): 0 IMU fresh, 1 IMU init, 2 kalibrasi selesai, 3 persist pending, 4 kaki tumpu kiri, 5 niat gait aktif
* `status` ACK: 0 OK, 1 BAD_LEN, 2 UNSUPPORTED, 3 REJECTED_STATE, 4 BUSY, 5 BAD_VALUE
* `mode`: 700 RAW, 705 FALLEN, 710 RESET, 720 READY, 730 BALANCE, 740–770 langkah, 780 POSE_HOLD, 790 ANGLE, 791 MIRROR,
  800 WALK_CONT (gait kontinu, eksperimental), 810 MOTION_PLAY
* `param`: 0 fallDeg, 1 rearmDeg, 2 imuStaleMs, 3 servoStep, 4 trigRollDeg, 5 trigPitchDeg, 6 footLift,
  7 swayBase, 8 swayGain, 9 rollSign, 10 pitchSign, 11 imuSwap,
  12 gaitEnable (0/1, default 0), 13 gaitStepMax (mm/langkah, default 10, maks 25),
  14 gaitYawMaxDeg (°/langkah, default 5, maks 15)

### 3.2 GAIT_INTENT (gait kontinu, mode 800)

Eksperimental, **mati secara default**: aktifkan dulu dengan `SET_PARAM gaitEnable 1`.

* `vx` = panjang langkah maju (+) / mundur (−) dalam **mm per langkah**, dipotong ke ±`gaitStepMax`.
* `yawDeg` = putaran **derajat per langkah**, dipotong ke ±`gaitYawMaxDeg`.
* `vy` disimpan tapi belum dipakai (langkah menyamping: fase berikut).
* `ttlMs` (0–1000) = niat kedaluwarsa setelah waktu ini. Kirim ulang tiap ±100–200 ms selama ingin berjalan.
* Hanya diterima di BALANCE (730) atau WALK_CONT (800); di mode lain dijawab `ACK REJECTED_STATE`.
  Dari BALANCE dengan `ttlMs > 0`, robot masuk 800.
* Perubahan target dihaluskan di batas langkah: panjang langkah berubah maks 3 mm/langkah,
  yaw maks 1°/langkah. Setelah niat kedaluwarsa, langkah meluruh ke 0, lalu robot menutup
  langkah (770) dan kembali ke BALANCE.
* Keselamatan tidak berubah: deteksi jatuh, batas sendi, dan slew limit tetap aktif; jatuh membatalkan niat.

### 3.3 Template gerak (LAM1, mode 810)

Format biner (little-endian), sama untuk unggahan LAP dan berkas SD:

```
u32 magic "LAM1" | u8 ver=1 | u8 nFrames (1..64) | u16 flags | char name[16]          (24 B)
nFrames × { u16 durMs | u8 interp | u8 rsv | i16 joint[2][11] }                      (48 B/frame)
u16 crc16 (CCITT-FALSE atas semua byte sebelumnya)
```

* `joint` = offset servo di ruang yang sama dengan `kin` pada GAIT_STATE (center, sign, dan trim
  ditambahkan robot), urutan bus R slot 0..10 lalu bus L.
* `interp`: 0 cosine, 1 linear, 2 step. `flags` bit 0 = kembali ke HOME setelah selesai.
* Alur unggah: `MOTION_BEGIN` → `MOTION_CHUNK` berurutan (offset harus sama dengan jumlah byte
  yang sudah diterima) → `MOTION_END`. Robot memvalidasi struktur dan CRC saat END
  (gagal → `ACK BAD_VALUE` + `LOG` alasan) lalu mengirim `MOTION_STATUS`.
* Slot yang sedang diputar tidak bisa ditimpa (`REJECTED_STATE`).
* `MOTION_PLAY` hanya dari READY. Sebelum mulai, **setiap frame dicek terhadap batas sendi**
  (termasuk trim); satu nilai di luar batas membatalkan seluruh gerakan.
* Alat: `tools/lam.py build gerak.json gerak.lam` (JSON → LAM1), `tools/lapctl.py motion-upload gerak.lam 0`,
  `motion-play 0`, `motion-stop`, `motions`. Contoh: `motions/lambai.json`.

### 3.4 Blackbox SD

Robot dengan SD merekam satu record 64 B per siklus kontrol (100 Hz) ke berkas `.lbb`
(header `LABB`): waktu, mode, roll/pitch, durasi loop, 22 perintah servo, bit feedback valid, flag,
`fwct`, `autoH`. Ubah ke CSV: `python3 tools/bbx2csv.py LBB00001.LBB > log.csv`.

## 4. Versi & kompatibilitas

* Field baru hanya **ditambahkan di akhir body**; penerima lama mengabaikan sisa byte, penerima baru
  memberi nilai default bila body lebih pendek.
* Perubahan yang tidak kompatibel menaikkan `protoVer`.
* Tipe `0x70–0x7F` khusus simulator/pengujian; robot sungguhan menjawab `UNSUPPORTED`.

## 5. Contoh

```bash
./build/leumpang_sim                       # terminal 1
python3 tools/lapctl.py --tcp 127.0.0.1 watch --servo
python3 tools/lapctl.py --tcp 127.0.0.1 cmd go
python3 tools/lapctl.py --tcp 127.0.0.1 sim-imu 0 40   # dorong sampai jatuh
python3 tools/lapctl.py --tcp 192.168.4.1 hello        # robot via SoftAP
python3 tools/lapctl.py --serial /dev/ttyACM0 hello    # robot via USB

# Fase 1
python3 tools/lam.py build motions/lambai.json /tmp/lambai.lam
python3 tools/lapctl.py --tcp 127.0.0.1 motion-upload /tmp/lambai.lam 0
python3 tools/lapctl.py --tcp 127.0.0.1 motion-play 0          # dari READY
python3 tools/lapctl.py --tcp 127.0.0.1 param gaitEnable 1
python3 tools/lapctl.py --tcp 127.0.0.1 cmd go                 # BALANCE
python3 tools/lapctl.py --tcp 127.0.0.1 gait 8 2 10             # 8 mm/langkah, 2°/langkah, selama 10 s
```
