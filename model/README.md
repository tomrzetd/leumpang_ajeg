# Model 3D Leumpang Ajeg

Dibangkitkan oleh `python3 tools/gen_model.py` (tambahkan `--preview` untuk `docs/img/model.png`). **Jangan edit manual**:
ubah dimensi di skrip, lalu bangkitkan ulang. Satu skrip menghasilkan:

| Berkas | Isi |
|---|---|
| `meshes/<link>.<material>.stl` | 82 mesh STL biner (milimeter), 24 link × material (servo, alu, shell, accent, rubber, glass, phone) |
| `leumpang.urdf` | URDF (meter) dengan 22 sendi revolute bernama (`R_HIP_P`, `L_ELBOW`, …) dan komentar bus/slot ICS |
| `../studio/lib/robot_model_gen.h` | tabel link/mesh/material untuk FK digital twin di studio |

Konvensi: X depan, Y kiri, Z atas. Semua frame link sejajar pada pose nol; sudut sendi = nilai src (`kin`, LAM1) / 1718,9 rad.
Dimensi kaki mengikuti konstanta IK core (paha = betis = 65 mm, offset roll→pitch 32 + 32,5 mm = `LEG_OFFSET`,
HOME = `HEIGHT` 185 mm) dan diverifikasi `tests/test_studio.cpp`.

Bentuknya mengikuti gaya KHR-3HV (servo KRS-2552RHV 32,5 × 21,5 × 26 mm, bracket aluminium, cover plastik), bukan
CAD resmi Kondo. Link `phone` (opsional) adalah Xiaomi 12 Pro (163,6 × 74,6 × 8,2 mm) lanskap di punggung.

Pakai di MuJoCo: `python -c "import mujoco; mujoco.MjModel.from_xml_path('model/leumpang.urdf')"` (mesh dicari relatif terhadap URDF).
