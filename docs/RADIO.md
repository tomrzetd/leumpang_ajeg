# Leumpang Radio — stik PS4/PS5 untuk robot (Fase 4B)

Stik DualShock 4 butuh **Bluetooth Classic**, yang tidak ada di ESP32-S3/C6/P4. Karena itu stik
disambungkan ke **board radio** kecil, lalu board radio meneruskan status stik ke board utama lewat
UART. Board radio yang didukung:

| Board | Chip | Firmware | Catatan |
|---|---|---|---|
| ESP32 DevKit WROOM-32 / ESP32-PICO-KIT (PICO-D4) | ESP32 klasik | `firmware/radio/esp32` (ESP-IDF 5.3/5.4) | paling lama diuji di Bluepad32 |
| Raspberry Pi **Pico 2 W** | RP2350 + CYW43439 | `firmware/radio/pico2w` (Pico SDK 2.2.0+) | juga Pico W (RP2040): `-DPICO_BOARD=pico_w` |

Keduanya memakai [Bluepad32](https://github.com/ricardoquesada/bluepad32) 4.2.0 dan kode bersama
`firmware/radio/common/radio.cpp`, jadi perilakunya sama. Selain DS4, DualSense (PS5), Switch Pro,
Xbox Wireless dan stik lain yang didukung Bluepad32 juga bisa dipakai. Tombolnya dipetakan ke posisi
yang sama (A = X, B = O, dst.).

## Pengkabelan

```
 board radio                 board utama
 TX  ───────────────────────> RX radio
 RX  <─────────────────────── TX radio
 GND ──────────────────────── GND
 5V/VBUS <── 5 V dari regulator robot (opsional; atau USB)
```

UART 460800 8N1, level 3,3 V di kedua sisi (jangan pakai level 5 V).

| | TX | RX | Catatan |
|---|---|---|---|
| ESP32 radio (UART2) | GPIO26 | GPIO27 | ubah di `idf.py menuconfig` → Leumpang Radio. GPIO16/17 dipakai flash di PICO-D4 |
| Pico 2 W radio (UART0) | GP0 (pin 1) | GP1 (pin 2) | log lewat USB |
| **P4** FireBeetle 2 (UART3) | GPIO46 | GPIO47 | **cek pinout cetak board Anda**; ubah `LA_RADIO_TX/RX` di `board_esp32p4.h` |
| S3 (UART0) | GPIO48 | GPIO12 | hanya bila konsol dipindah ke USB-Serial-JTAG (S3 hanya punya 3 UART) |
| STM32H562 (USART1) | PA9 | PA10 | |

## Membangun dan mem-flash

**ESP32 klasik**
```bash
git submodule update --init --recursive firmware/radio/third_party/bluepad32
cd firmware/radio/esp32
idf.py set-target esp32
idf.py build flash monitor
```
Atau unduh artefak **leumpang-radio-esp32** dari tab Actions dan flash dengan `esptool.py` memakai `flash_args`.

**Pico 2 W**
Unduh artefak **leumpang-radio-pico2_w** dari tab Actions. Tahan tombol BOOTSEL, colokkan USB,
lalu salin `leumpang_radio_pico2_w.uf2` ke drive `RP2350`. Untuk membangun sendiri:
```bash
export PICO_SDK_PATH=/jalur/pico-sdk        # 2.2.0+, dengan submodule lib/btstack lib/cyw43-driver ...
cd firmware/radio/pico2w
cmake -B build -DPICO_BOARD=pico2_w && cmake --build build -j
```

## Menyambungkan stik

1. Nyalakan board radio. LED berkedip artinya radio sedang mencari stik.
2. DualShock 4: tahan **Share + PS** sampai lightbar berkedip cepat (mode pairing).
   DualSense: tahan **Create + PS**.
3. Setelah tersambung, LED radio menyala tetap dan stik bergetar sebentar.
   Lightbar mengikuti mode robot (lihat tabel di bawah).
4. Kunci pairing disimpan, jadi lain kali cukup tekan **PS**. Tekan PS ini tidak memicu E-STOP,
   karena PS/touchpad diabaikan selama 1 detik pertama setelah stik tersambung.

Hanya **satu** stik yang bisa mengendalikan robot; stik kedua ditolak.

## Pemetaan tombol (bawaan)

| Tombol | Aksi |
|---|---|
| **PS** atau **klik touchpad** | **E-STOP** kapan saja: servo free (robot jatuh lemas). Lightbar merah + getar |
| **L1 ditahan** | *deadman*: tanpa L1 tidak ada gerak. Lepas L1 = gait berhenti (robot tetap berdiri) |
| L1 + **Options** | reset / berdiri (dari FALLEN, robot ditegakkan) |
| L1 + **X** | GO: READY → BALANCE |
| L1 + **stik kiri** maju/mundur | jalan (GAIT_INTENT; perlu `param gaitEnable 1`) |
| L1 + **stik kanan** kiri/kanan | belok |
| L1 + **segitiga** / **kotak** | motion slot 0 / 1 |
| L1 + **d-pad** atas/kanan/bawah/kiri | motion slot 2 / 3 / 4 / 5 |
| L1 + **O** | hentikan motion |

Pengaman lain:
* Bila tidak ada laporan stik selama > 250 ms (radio mati, kabel lepas, stik di luar jangkauan),
  gait berhenti. Robot **tidak** dijatuhkan, karena E-STOP hanya terjadi kalau diminta.
* Stik terputus → sama seperti melepas L1.
* `gaitEnable` tetap harus dinyalakan dulu dari desktop/konsol; stik tidak bisa menyalakannya.

Warna lightbar: hijau = READY, biru = jalan/balance, merah = FALLEN/E-STOP, ungu = alat servo,
toska = live pose, kuning = motion, putih redup = radio hidup tetapi robot tidak menjawab.

Pemetaan ada di core (`Controller::padInput`), jadi teruji di simulator dan sama di semua board.
Skrip MicroPython (Fase 4C) nanti bisa membaca stik sendiri untuk pemetaan lain.

## Uji tanpa board radio

PC bisa berperan sebagai radio lewat `lapctl` (simulator atau robot via Wi-Fi):
```bash
./build/leumpang_sim
python3 tools/lapctl.py --tcp 127.0.0.1 pad --secs 2                  # stik virtual tersambung
python3 tools/lapctl.py --tcp 127.0.0.1 pad l1 x --secs 1             # L1+X = GO
python3 tools/lapctl.py --tcp 127.0.0.1 param gaitEnable 1
python3 tools/lapctl.py --tcp 127.0.0.1 pad l1 --ly 80 --secs 3       # jalan maju
python3 tools/lapctl.py --tcp 127.0.0.1 pad touch --press-after 1.3   # E-STOP
```

## Protokol

Board radio adalah peer LAP biasa di UART: `PAD` (0x50) 50 Hz radio → robot, `PAD_FEEDBACK` (0x51)
robot → radio. Detailnya di [PROTOCOL.md](PROTOCOL.md). Karena memakai LAP penuh, board radio kelak
bisa juga menjadi jembatan Wi-Fi untuk board utama tanpa Wi-Fi (STM32).

## Lisensi pihak ketiga

* Bluepad32: Apache-2.0.
* BTstack: gratis untuk penggunaan **non-komersial** (proyek hobi/terbuka seperti ini aman).
  Raspberry Pi memberi lisensi BTstack untuk **produk** Pico W/WH, Pico 2 W/WH dan modul RM2
  (`pico-sdk/src/rp2_common/pico_btstack/LICENSE.RP`). Lisensi itu tidak mencakup chip CYW43439
  yang dilepas lalu dipasang ke board lain. Produk komersial di luar itu (mis. berbasis ESP32)
  perlu lisensi dari BlueKitchen.
