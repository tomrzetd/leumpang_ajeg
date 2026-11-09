# Profil robot — `config/robot.json` di kartu SD

Mulai Fase 4A, Leumpang Ajeg adalah **controller servo umum**. Firmware (P4, S3, STM32H562) membaca
`/config/robot.json` saat boot. Profil menentukan servo mana yang terpasang, ID-nya, arah, batas,
dan pose HOME. Bila berkas tidak ada, firmware menulis profil bawaan KHR-3HV supaya mudah diedit.
Bila berkas rusak, firmware memakai profil bawaan dan menulis alasannya ke log (pengaman: robot tetap
bisa dihubungi dan profil bisa diperbaiki lewat LAP).

Ubah profil dari desktop:

```bash
python3 tools/lapctl.py --tcp 192.168.4.1 profile get robot.json   # unduh
#   ... edit robot.json ...
python3 tools/lapctl.py --tcp 192.168.4.1 profile put robot.json   # cek sintaks, unggah
python3 tools/lapctl.py --tcp 192.168.4.1 cmd reboot               # berlaku setelah boot
```

## Bentuk berkas

```json
{
  "name": "leumpang-ajeg",
  "kind": "humanoid-khr",
  "servos": [
    { "name": "R ELBOW", "model": "KRS-3304", "center": 7400, "home": -500 },
    { "bus": 1, "slot": 4, "id": 4, "sign": -1, "min": 4000, "max": 11000 }
  ]
}
```

| Kunci | Arti |
|---|---|
| `name` | nama robot (≤ 23 karakter) |
| `kind` | `humanoid-khr` (gait UVC, IK, GO diizinkan) atau `generic` (lengan, kepala, robot lain: hanya pose/motion/live pose/skrip; GO ditolak) |
| `servos[]` | daftar servo; field yang tidak ditulis memakai nilai bawaan |

Per servo:

| Kunci | Arti | Bawaan |
|---|---|---|
| `bus`, `slot` | bus 0 = R (UART kanan), 1 = L; slot 0..10 | — |
| `name` | nama sendi. Pada `humanoid-khr`, nama KHR (`"R KNEE"`, `"L HIP P"`, `"WAIST"`, `"HEAD"` …) cukup untuk menunjuk bus/slot | `B<bus> S<slot>` (generic) |
| `id` | ID ICS servo (0..31), unik per bus | = slot |
| `model` | teks bebas, mis. `KRS-2552`, `KRS-3204`, `KRS-3304` | — |
| `center` | posisi ICS untuk sudut 0 sendi | 7500 |
| `sign` | +1 / −1, arah putar | +1 |
| `min`, `max` | batas posisi ICS, dalam 3500..11500 | 3500, 11500 |
| `home` | pose HOME sebagai offset dari `center` (ruang sendi, sebelum `sign`) | 0 |
| `present` | `false` = slot kosong (tidak dikirimi frame) | `true` untuk yang ditulis |

## Aturan validasi

* `id` ≤ 31 dan tidak ada ID ganda di bus yang sama.
* `min < max`, keduanya dalam 3500..11500; `center` di antara keduanya.
* `center + sign × home` harus di dalam batas.
* `humanoid-khr` butuh semua 22 sendi; field yang tidak ditulis memakai nilai KHR-3HV bawaan,
  jadi profil kalibrasi cukup memuat sendi yang berubah.
* `generic`: slot yang tidak ditulis dianggap kosong.

## Contoh: lengan 4 servo

```json
{
  "name": "lengan-meja",
  "kind": "generic",
  "servos": [
    { "bus": 0, "slot": 0, "id": 0, "name": "BASE",     "model": "KRS-3304" },
    { "bus": 0, "slot": 1, "id": 1, "name": "SHOULDER", "model": "KRS-3304", "min": 5000, "max": 10000 },
    { "bus": 0, "slot": 2, "id": 2, "name": "ELBOW",    "model": "KRS-2552", "sign": -1 },
    { "bus": 0, "slot": 3, "id": 3, "name": "GRIP",     "model": "KRS-2552", "min": 6500, "max": 8500, "home": 300 }
  ]
}
```

## Menyiapkan servo baru

ID servo diberi lewat alat servo (mode 830, [PROTOCOL.md §3.9](PROTOCOL.md)). Pasang **satu** servo
saja di bus, lalu:

```bash
python3 tools/lapctl.py --tcp 192.168.4.1 cmd stop
python3 tools/lapctl.py --tcp 192.168.4.1 servo id R          # ID sekarang
python3 tools/lapctl.py --tcp 192.168.4.1 servo setid R 3     # ID baru
python3 tools/lapctl.py --tcp 192.168.4.1 servo scan          # setelah semua terpasang
```

## Batas saat ini

* 2 bus × 11 slot (22 kanal). Lebih banyak kanal menyusul bersama skrip MicroPython (Fase 4C).
* Profil hanya dibaca saat boot (`CMD 7` reboot).
* Studio masih memakai model 3D KHR-3HV; studio yang membaca profil dari robot direncanakan di Fase 4D.
