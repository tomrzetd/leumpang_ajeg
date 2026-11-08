# Leumpang Bus — API antar-modul

Leumpang Bus adalah "jalur saraf" robot: setiap kemampuan (indera, perilaku, perangkat tambahan) adalah **modul**
yang **menerbitkan** dan **berlangganan topik**. Modul baru cukup memakai topik yang sudah ada, tanpa mengubah modul lain.

```
             ┌──────────────── HP "mata" (Leumpang Mata) ─────────────────┐
 kamera ───► │ vision ─► vision/objects, vision/gesture, vision/faces     │
             │ obstacles ─► perception/obstacles, perception/person       │ ws://IP-HP:8765
 mikrofon ─► │ speech ─► speech/heard, behavior/request ◄─ speech/say     │ ◄──────────────► studio PC,
             │ behavior ◄─ behavior/request ─► behavior/state, speech/say │     JSON          skrip Python,
             │ robot (LAP) ─► robot/telemetry, robot/log, robot/ext/+     │                   board Wi-Fi lain
             └───────────────────────────┬─────────────────────────────────┘
                              USB-OTG / Wi-Fi, LAP biner (PERCEPTION, GAIT_INTENT, EXT ...)
                             ┌───────────┴───────────┐
                             │ leumpang-ajeg (ESP32 / │  refleks rintangan, keseimbangan,
                             │ STM32): firmware core  │  batas sendi = otoritas akhir
                             └────────────────────────┘
```

Tiga lapis, satu konsep topik:

| Lapis | Transport | Format | Dipakai oleh |
|---|---|---|---|
| Di dalam HP | memori (`id.leumpang.bus.Bus`) | `JsonElement` | modul Kotlin di aplikasi |
| HP ↔ luar | WebSocket `ws://<ip-HP>:8765` | JSON teks | studio PC, Python, Node, board Wi-Fi |
| HP ↔ robot | LAP (USB-OTG / TCP) | biner; pesan `EXT 0x60` = topik 16 bit | firmware & modul HAL di board robot |

## Topik bawaan

| Topik | Arah | Isi |
|---|---|---|
| `robot/link` | terbit | `{connected, alive, label, rx, crcErrors}` |
| `robot/hello` | terbit | `{name, board, caps, fw}` |
| `robot/telemetry` | terbit ≤ 10 Hz | `{mode, modeName, roll, pitch, autoH, flags, obstacleBlock, fbOk, loopUs}` |
| `robot/log` | terbit | `{line}` |
| `robot/ext/<hhhh>` | terbit | topik EXT dari robot: `{topic, hex, text}` |
| `robot/ext/out` | **boleh dari luar** | kirim EXT ke robot: `{topic, hex}` atau `{topic, text}` |
| `vision/objects` | terbit | `[{label, score, box:[l,t,r,b]}]` (koordinat 0..1, label COCO) |
| `vision/gesture` | terbit | `{name, score, hand}` (`Open_Palm`, `Thumb_Up`, `Victory`, ...) |
| `vision/faces` | terbit | `[{name, similarity, box}]` (`name` kosong = tidak dikenal) |
| `perception/obstacles` | terbit 10 Hz | `{halfFovDeg, distMm:[...], frontMm, cameraHeightMm}` sektor kanan→kiri, 65535 = bebas |
| `perception/person` | terbit | `{visible, bearingDeg, distMm, score}` (+ = kiri) |
| `speech/heard` | terbit | `{text, alternatives}` |
| `speech/say` | **boleh dari luar** | `{text}` → diucapkan TTS |
| `behavior/request` | **boleh dari luar** | `{action, ...}` lihat bawah |
| `behavior/state` | terbit | `{activity, reason}` (`idle`, `wander`, `follow`, `manual`, `stopped`) |
| `sensor/<nama>` | **boleh dari luar** | sensor tambahan buatan Anda (bebas) |
| `app/#` | **boleh dari luar** | data aplikasi lain |
| `app/models`, `app/bus` | terbit | status unduh model AI, alamat & jumlah klien bus |

`behavior/request`: `stop`, `wander`, `follow`, `wave`, `home`, `balance`, `describe`,
`{"action":"walk","vx":6,"seconds":3}` (mm/langkah), `{"action":"turn","yaw":4,"seconds":2}` (°/langkah, + kiri),
`{"action":"say","text":"..."}`. Semua jalan lewat arbiter perilaku (rintangan tetap menahan langkah maju) dan
firmware tetap menolak gerak yang tidak aman.

## Protokol WebSocket (JSON)

Aktifkan di aplikasi: **Pengaturan → Leumpang Bus WebSocket** (opsional isi token di kode/Settings).

```jsonc
// klien -> HP
{"op":"hello","token":"rahasia","name":"studio"}           // wajib pertama bila token diset
{"op":"sub","topics":["perception/#","robot/telemetry"]}   // "+" satu segmen, "#" sisa
{"op":"unsub","topics":["perception/#"]}
{"op":"pub","topic":"sensor/jarak_kaki","data":{"mm":180}}
{"op":"get","topic":"robot/telemetry"}                      // nilai terakhir
{"op":"describe"}                                           // modul, topik aktif, izin publish
// HP -> klien
{"op":"msg","topic":"perception/obstacles","t":1731050000000,"src":"obstacles","data":{...}}
{"op":"ok","re":"sub"}   {"op":"error","re":"pub","msg":"topik 'robot/cmd' tidak boleh diterbitkan dari luar"}
```

Contoh Python (`pip install websockets`):

```python
import asyncio, json, websockets
async def main():
    async with websockets.connect("ws://192.168.1.50:8765") as ws:
        await ws.send(json.dumps({"op": "sub", "topics": ["perception/obstacles", "vision/objects"]}))
        await ws.send(json.dumps({"op": "pub", "topic": "behavior/request", "data": {"action": "wander"}}))
        async for m in ws:
            print(json.loads(m))
asyncio.run(main())
```

## Menambah modul di HP (Kotlin)

```kotlin
class JarakUltrasonik : Module {
    override val info = ModuleInfo("ultrasonik", "HC-SR04 lewat EXT robot", listOf("sensor/ultrasonik"), listOf("robot/ext/1001"))
    override fun start(bus: Bus) {
        bus.subscribe("robot/ext/1001") { m ->
            val mm = J.str(m.data, "hex")!!.chunked(2).map { it.toInt(16) }.let { it[0] or (it[1] shl 8) }
            bus.publish("sensor/ultrasonik", J.obj("mm" to mm), "ultrasonik")
        }
    }
}
// Engine: host.add(JarakUltrasonik()).start("ultrasonik")
```

## Menambah sensor di board robot (C++, EXT)

Topik EXT 16 bit: `0x0000–0x0FFF` dicadangkan core (`0x0001` = echo untuk uji), `0x1000+` bebas untuk modul Anda.

```cpp
// HAL: terima dari HP
h.extHook = [](void* ctx, uint16_t topic, const uint8_t* d, size_t n) {
    if (topic == 0x1002) { setLampu(d[0]); return true; }
    return false;                       // ACK BAD_VALUE
};
// terbitkan ke HP (thread komunikasi, bukan task kontrol)
uint8_t b[2] = {(uint8_t)mm, (uint8_t)(mm >> 8)};
ep.sendExt(0x1001, b, 2);              // muncul di HP sebagai robot/ext/1001
```
Firmware yang mendukung EXT mengiklankan `HELLO.caps` bit 9. Saat ini simulator mendukungnya; HAL ESP32/STM32
belum memasang `extHook` (tinggal ditambahkan per sensor).
