# Leumpang Studio (desktop, Qt 6)

Aplikasi desktop x86-64 (Windows / Linux) untuk robot Leumpang Ajeg:

| Tab / panel | Fungsi |
|---|---|
| **Digital twin 3D** (kiri) | Model 3D realistis (24 link, 7 material, bayangan, AO) digerakkan pose robot secara langsung. Hantu biru = umpan balik servo aktual, atau pose robot saat pratinjau. Kemiringan IMU diterapkan ke badan, kaki terendah selalu menapak lantai. |
| **Servo** | 22 sendi: slider pose kerja, nilai perintah vs aktual (oranye bila selisih > 150), status servo, **trim** langsung ke robot. *Pratinjau di twin*, *Live ke robot* (mode 820, 20 Hz), *Ambil dari robot* (teach-by-hand: lepas servo, bentuk pose dengan tangan). |
| **Gerak** | Editor keyframe (≤ 64): durasi, interpolasi cosine/linear/step, urutkan, gandakan. Pratinjau di twin **identik** dengan pemutar firmware. Validasi batas sendi + trim robot. JSON (format `tools/lam.py`), impor/ekspor `.lam`, unggah ke slot 0–3 (+SD), putar/stop. |
| **Kamera** | Kamera PC → MoveNet (ONNX Runtime) → retarget lengan + kepala → twin, **live ke robot**, atau **rekam** → keyframe otomatis (penyederhanaan RDP). Kalibrasi T-pose. Kaki tidak pernah dikendalikan kamera. |
| **Jalan** | Gait kontinu (GAIT_INTENT): maju/belok, tahan untuk jalan. Simulator: miringkan IMU. |
| **Parameter** | 15 parameter runtime (SET_PARAM). |
| **Log** | Log robot (pesan LOG) + ACK yang ditolak. |

Koneksi: **Simulator** (leumpang-core lengkap di dalam studio, tanpa hardware), **Wi-Fi** (ESP32-S3/P4, `192.168.4.1:5760`), atau **USB serial** (semua board; STM32 USB CDC, P4 USB-C, S3 UART).

## Unduh (Windows)

Setiap push ke `main` membangun paket Windows di GitHub Actions: tab **Actions** → run terbaru →
artefak **LeumpangStudio-windows-x64** (zip berisi `leumpang_studio.exe` + Qt + ONNX Runtime).
Ekstrak, jalankan `leumpang_studio.exe`.

## Model pose kamera

Studio membutuhkan model **MoveNet SinglePose** dalam format ONNX (Lightning 192 px atau Thunder 256 px;
masukan int32/uint8/float32, NHWC/NCHW dideteksi otomatis). Contoh sumber:

* Hugging Face `Xenova/movenet-singlepose-lightning` (folder `onnx/`), atau
* konversi sendiri dari TF Hub: `python -m tf2onnx.convert --saved-model movenet_singlepose_lightning_4 --output movenet.onnx`

Tab **Kamera** → *Pilih model ONNX*.

## Build dari sumber

```bash
# Qt >= 6.5 dengan modul: Quick3D, ShaderTools, SerialPort, Multimedia
cmake -S studio -B build-studio -DCMAKE_PREFIX_PATH=/path/ke/Qt/6.8.3/gcc_64
cmake --build build-studio --config Release
./build-studio/leumpang_studio
```
ONNX Runtime 1.22 diunduh otomatis dari rilis resmi Microsoft (matikan: `-DLA_STUDIO_POSE=OFF`).
Windows: buka "x64 Native Tools Command Prompt", generator Visual Studio 2022, lalu
`windeployqt --qmldir studio/qml build-studio/Release/leumpang_studio.exe`.

Uji otomatis (dipakai CI): `leumpang_studio --selftest folder` — menyambung ke simulator, mengunggah
dan memutar gerak, menguji live pose, menyimpan tangkapan layar tiap tab, gagal bila ada warning QML.

## Arsitektur

```
studio/
├─ lib/        tanpa Qt, diuji di ctest host (tests/test_studio.cpp)
│  ├─ kinematics.*      FK twin dari tabel model (dibangkitkan tools/gen_model.py)
│  ├─ retarget.*        17 keypoint 2D -> bahu pitch/roll/yaw + siku + kepala, One-Euro
│  ├─ motion_doc.*      keyframe <-> LAM1, pratinjau = Controller::motionStep, RDP
│  ├─ lap_client.*      sisi host protokol LAP
│  ├─ sim_robot.*       robot simulasi (Controller + LapEndpoint + model servo)
│  └─ pose_onnx.*       MoveNet via ONNX Runtime
├─ app/        backend Qt: RobotLink, TwinModel, StlMesh, MotionEditor, PoseCapture
└─ qml/        UI (Qt Quick Controls Material + Qt Quick 3D)
```
Mesh dan URDF ada di `model/` (dibangkitkan `python3 tools/gen_model.py`). Ubah dimensi di skrip
itu bila robot Anda berbeda; FK twin, URDF, dan mesh ikut berubah bersama.
