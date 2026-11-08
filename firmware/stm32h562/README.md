# Leumpang Ajeg — firmware STM32H562 (WeAct STM32H562RGT6)

Bare-metal (STM32Cube HAL, tanpa RTOS). Memakai `leumpang-core` yang sama persis dengan ESP32-S3/P4.
Link ke PC/HP lewat **USB CDC** (konsol teks + protokol LAP di port yang sama). Tidak ada Wi-Fi.

## Arsitektur waktu nyata

| Sumber | Laju | Prioritas NVIC | Tugas |
|---|---|---|---|
| SysTick | 1 kHz | 0 | `HAL_GetTick` (timeout HAL dipakai di ISR) |
| TIM2 | 1 MHz | 1 | jam mikrodetik 64-bit `la_micros()` |
| USB DRD FS | — | 3 | TinyUSB (`tud_int_handler`) |
| TIM7 | 500 Hz | 5 | baca ICM-20948 (I2C1) + fusi Mahony |
| TIM6 | 100 Hz | 6 | `la::Controller::step()` + pertukaran servo ICS |
| loop `main` | bebas | — | `tud_task`, SerialLink/LAP, SD (config, motion, blackbox), LED |

Data antar konteks lewat ring SPSC tanpa lock (`core/include/leumpang/blackbox.h`):
tombol konsol & perintah LAP → ISR kontrol; log & blackbox → loop main. ISR kontrol tidak pernah
menyentuh USB atau SD.

## Pin (verifikasi terhadap header board dan datasheet sebelum menyambung)

| Fungsi | Pin | Catatan |
|---|---|---|
| Bus ICS kanan | USART2 TX **PA2**, RX **PA3** (AF7), EN **PA4** | 1,25 Mbps 8E1 |
| Bus ICS kiri | USART6 TX **PC6**, RX **PC7** (AF7), EN **PB0** | |
| IMU ICM-20948 | I2C1 SCL **PB6**, SDA **PB7** (AF4) | ~390 kHz, `IMU_I2C_TIMING` |
| microSD | SDMMC1 D0–D3 **PC8–PC11**, CLK **PC12**, CMD **PD2** (AF12) | 4-bit |
| USB-C | **PA11/PA12** (AF10) | VID:PID `1209:0001` |
| LED | **PB2** | kedip 1 Hz normal, 5 Hz saat FALLEN |
| Pin ukur T2 | **PB1** | HIGH selama satu siklus kontrol |

Semua pin ada di `include/board.h`. EN HIGH = TX (`ICS_EN_TX_LEVEL`), sama dengan board ESP.
Rangkaian half-duplex (74HC126 / driver Kondo) sama dengan diagram wiring ESP32-S3.

## Build & flash

```bash
# arm-none-eabi-gcc >= 12 dan CMake >= 3.20. Dependensi (HAL v1.7.0, CMSIS, FatFs r0.16,
# TinyUSB 0.21.0) diunduh otomatis oleh CMake (versi dipin).
cd firmware/stm32h562
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build -j          # -> build/leumpang_h562.{elf,bin,hex}

# Flash: ST-LINK (SWD) atau DFU USB (tahan BOOT0 saat reset)
STM32_Programmer_CLI -c port=SWD -w build/leumpang_h562.hex -v -rst
# atau: dfu-util -a 0 -s 0x08000000:leave -D build/leumpang_h562.bin
```
Ukuran saat ini: FLASH ±117 KB (11%), RAM ±71 KB (11%). CI membangunnya di setiap push.

## Konsol & LAP

Buka port serial USB (`/dev/ttyACM*`, `COMx`; baud bebas). Perintah teks sama dengan board lain
(`r g x c t y z ?`). Begitu host mengirim byte `0x00`, port beralih ke mode LAP; teks log tetap
dikirim sebagai pesan `LOG`. Klien:

```bash
python3 tools/lapctl.py --serial /dev/ttyACM0 hello
python3 tools/lapctl.py --serial /dev/ttyACM0 watch --servo
```

## SD card (FAT32)

| Berkas | Isi |
|---|---|
| `/CFG.BIN` | konfigurasi + trim (crc16); ditulis dari loop main 1 s setelah konfigurasi berhenti berubah |
| `/MOTIONS/SLOTn.LAM` | template gerak LAM1, dimuat ke slot 0–3 saat boot |
| `/BBX/LBBnnnnn.LBB` | blackbox 100 Hz (64 B/record), ubah ke CSV dengan `tools/bbx2csv.py` |

Tanpa kartu, robot tetap jalan dengan konfigurasi default (log `SD: tidak ada kartu / gagal mount`).

## Belum di Fase 1

* LCD ST7789 dan RGB WS2812 (ada di board ESP).
* Penyimpanan konfigurasi di flash internal saat tidak ada SD.

## Status verifikasi

* Dikompilasi dan di-link dengan arm-none-eabi-gcc 14.2 (lokal) dan 13.3 (CI); tabel vektor dan handler ISR dicek.
* **Belum diuji di hardware.** Yang paling perlu dicek pertama kali: clock 250 MHz dari HSE 8 MHz,
  enumerasi USB, nilai `IMU_I2C_TIMING`, polaritas EN, dan apakah board versi Anda punya slot microSD
  di pin SDMMC1 di atas.
