# LAP — Leumpang Ajeg Protocol v1

Satu protokol untuk semua link: HP Android (USB-OTG CDC), aplikasi desktop (USB / TCP Wi-Fi),
simulator (TCP), dan alat baris-perintah. Implementasi referensi:

* C++: `core/include/leumpang/lap.h`, `core/src/lap.cpp`, endpoint robot `lap_endpoint.*`
* Python: `tools/lap.py` (stdlib saja), klien `tools/lapctl.py`
* Kotlin (Android): Fase 3

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
   Mulai Fase 1, putus link juga membatalkan `GAIT_INTENT` (robot kembali BALANCE/READY).
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
| 20 | CMD | host→robot | `u8 cmd`: 0 reset, 1 go, 2 stop (servo free), 3 kalibrasi IMU, 4 kirim CONFIG, 5 kirim HELLO |
| 21 | SET_PARAM | host→robot | `u8 param, f32 value` |
| 22 | SET_TRIM | host→robot | `u8 bus, u8 slot, i16 value` (±1000, hanya diterapkan di READY) |
| 23 | STREAM_CFG | host→robot | `u8 teleHz(0–50), u8 servoHz(0–20), u8 gaitHz(0–50), u8 logOn` |
| 30 | GAIT_INTENT | HP→robot | `f32 vx, f32 vy, f32 yawRate, u16 ttlMs` — **Fase 1** (v1: `UNSUPPORTED`) |
| 31 | PERCEPTION | HP→robot | daftar rintangan — **Fase 1/3** |
| 40–43 | MOTION_* | host→robot | unggah & putar template gerak dari SD — **Fase 1** |
| 70 | SIM_IMU | host→sim | `f32 roll, f32 pitch, u8 sensorOk` |
| 71 | SIM_FAULT | host→sim | `u8 bus, u8 slot, u8 dead` |

### Nilai enum

* `role`: 0 robot, 1 phone, 2 desktop, 3 sim, 4 tool
* `board`: 0 unknown, 1 esp32s3, 2 esp32p4, 3 stm32h562, 4 sim
* `caps` (bit): 0 Wi-Fi, 1 SD, 2 gait intent, 3 motion, 4 sim, 5 LCD, 6 USB CDC
* `flags` TELEMETRY (bit): 0 IMU fresh, 1 IMU init, 2 kalibrasi selesai, 3 persist pending, 4 kaki tumpu kiri
* `status` ACK: 0 OK, 1 BAD_LEN, 2 UNSUPPORTED, 3 REJECTED_STATE, 4 BUSY, 5 BAD_VALUE
* `mode`: 700 RAW, 705 FALLEN, 710 RESET, 720 READY, 730 BALANCE, 740–770 langkah, 780 POSE_HOLD, 790 ANGLE, 791 MIRROR
* `param`: 0 fallDeg, 1 rearmDeg, 2 imuStaleMs, 3 servoStep, 4 trigRollDeg, 5 trigPitchDeg, 6 footLift,
  7 swayBase, 8 swayGain, 9 rollSign, 10 pitchSign, 11 imuSwap

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
```
