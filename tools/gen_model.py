#!/usr/bin/env python3
"""gen_model.py - model 3D parametrik Leumpang Ajeg (gaya KHR-3HV, 22 servo ICS).

Satu sumber kebenaran untuk:
  model/meshes/<link>.<material>.stl   mesh biner (mm), dipakai studio & URDF
  model/leumpang.urdf                  URDF (meter) untuk MuJoCo / ROS / Isaac
  studio/lib/robot_model_gen.h         tabel link C++ untuk FK studio
  docs/img/model.png                   pratinjau (opsional, --preview)

Kinematika (dicek tests/test_studio.cpp terhadap IK leumpang-core):
  * Sudut sendi = nilai src (ruang 'kin', hitungan) / CHG_SVA (1718.9 hitungan/rad).
  * Frame: X maju, Y kiri, Z atas. Semua frame link sejajar pada pose nol (hanya translasi).
  * Kaki: pitch-chain 65 + 65 mm, offset roll->pitch 32 + 32.5 mm (= LEG_OFFSET 64.5),
    jarak sumbu hip-roll ke ankle-roll pada HOME = HEIGHT 185 mm.
  * Lengan: pada src 0 lengan menunjuk ke depan; HOME bahu pitch -2700 (~ -90 deg) = lengan turun.

  python3 tools/gen_model.py            # tulis semua keluaran
  python3 tools/gen_model.py --preview  # + render docs/img/model.png
"""
import math
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHG_SVA = 1718.9

# ---------------------------------------------------------------------------
# Material (nama, warna linear-ish sRGB, metalness, roughness)
# ---------------------------------------------------------------------------
MATERIALS = [
    ("servo",  (0.11, 0.12, 0.13), 0.0, 0.55),   # plastik servo ICS gelap
    ("alu",    (0.74, 0.76, 0.79), 1.0, 0.32),   # bracket aluminium
    ("shell",  (0.91, 0.90, 0.87), 0.0, 0.45),   # cover plastik putih
    ("accent", (0.91, 0.42, 0.11), 0.0, 0.40),   # aksen oranye
    ("rubber", (0.05, 0.05, 0.05), 0.0, 0.90),   # sol karet
    ("glass",  (0.02, 0.02, 0.03), 0.6, 0.08),   # visor / lensa
    ("phone",  (0.16, 0.17, 0.19), 0.3, 0.25),   # HP Android (opsional)
]
MAT = {m[0]: i for i, m in enumerate(MATERIALS)}

# ---------------------------------------------------------------------------
# Primitif mesh -> list segitiga (N,3,3)
# ---------------------------------------------------------------------------
def rounded_box(size, center=(0, 0, 0), r=2.0, n=4):
    """Kotak membulat: titik grid permukaan kubus diproyeksikan ke 'kotak dalam + bola r'."""
    sx, sy, sz = [s / 2.0 for s in size]
    r = min(r, sx * 0.95, sy * 0.95, sz * 0.95)
    inner = np.array([sx - r, sy - r, sz - r])
    tris = []
    u = np.linspace(-1, 1, n + 1)
    for axis in range(3):
        for sign in (-1, 1):
            a1, a2 = [k for k in range(3) if k != axis]
            grid = np.zeros((n + 1, n + 1, 3))
            for i, uu in enumerate(u):
                for j, vv in enumerate(u):
                    p = np.zeros(3)
                    p[axis] = sign
                    p[a1] = uu
                    p[a2] = vv
                    grid[i, j] = p
            half = np.array([sx, sy, sz])
            # sebar titik lebih rapat di tepi agar lengkung halus
            g = np.sign(grid) * np.abs(grid) ** 0.6 * half
            c = np.clip(g, -inner, inner)
            d = g - c
            ln = np.linalg.norm(d, axis=2, keepdims=True)
            ln[ln < 1e-9] = 1.0
            q = c + d / ln * r
            flat = (np.abs(d) < 1e-9).all(axis=2, keepdims=True)
            q = np.where(flat, g, q)
            for i in range(n):
                for j in range(n):
                    p00, p10, p01, p11 = q[i, j], q[i + 1, j], q[i, j + 1], q[i + 1, j + 1]
                    t1, t2 = [p00, p10, p11], [p00, p11, p01]
                    nrm = np.zeros(3); nrm[axis] = sign
                    for t in (t1, t2):
                        if np.dot(np.cross(t[1] - t[0], t[2] - t[0]), nrm) < 0:
                            t[1], t[2] = t[2], t[1]
                        tris.append(t)
    return np.array(tris) + np.array(center)


def cylinder(radius, length, center=(0, 0, 0), axis="z", seg=28, r2=None):
    r2 = radius if r2 is None else r2
    h = length / 2.0
    ang = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    tris = []
    for k in range(seg):
        a0, a1 = ang[k], ang[(k + 1) % seg]
        b0 = np.array([radius * math.cos(a0), radius * math.sin(a0), -h])
        b1 = np.array([radius * math.cos(a1), radius * math.sin(a1), -h])
        t0 = np.array([r2 * math.cos(a0), r2 * math.sin(a0), h])
        t1 = np.array([r2 * math.cos(a1), r2 * math.sin(a1), h])
        tris += [[b0, b1, t1], [b0, t1, t0]]
        tris += [[np.array([0, 0, h]), t0, t1], [np.array([0, 0, -h]), b1, b0]]
    t = np.array(tris)
    if axis == "x":
        t = t[..., [2, 0, 1]]
    elif axis == "y":
        t = t[..., [1, 2, 0]]
    return t + np.array(center)


def rot_z(tris, deg, pivot=(0, 0, 0)):
    a = math.radians(deg)
    R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    p = np.array(pivot)
    return (tris - p) @ R.T + p


# ---------------------------------------------------------------------------
# Komponen robot
# ---------------------------------------------------------------------------
SERVO = (32.5, 21.5, 26.0)   # KRS-2552RHV: panjang x lebar x tinggi (mm)


def servo(horn_at, axis, body_dir, parts):
    """Servo ICS: horn di titik sendi pada sumbu 'axis', badan memanjang ke 'body_dir'.
    axis, body_dir: 'x','y','z' dengan tanda ('+x', '-z', ...)."""
    ax = "xyz".index(axis[1]); asg = 1 if axis[0] == "+" else -1
    bd = "xyz".index(body_dir[1]); bsg = 1 if body_dir[0] == "+" else -1
    third = 3 - ax - bd
    size = [0, 0, 0]
    size[bd] = SERVO[0]; size[ax] = SERVO[2]; size[third] = SERVO[1]
    c = np.array(horn_at, dtype=float)
    c[ax] -= asg * (SERVO[2] / 2 + 1.5)          # badan di belakang horn
    c[bd] += bsg * (SERVO[0] / 2 - 10.0)         # poros servo ~10 mm dari ujung
    parts.append(("servo", rounded_box(size, c, r=2.2, n=4)))
    # garis aksen tipis di badan servo (label)
    lab = list(size); lab[ax] += 0.3; lab[bd] *= 0.55; lab[third] = 2.0
    lc = c.copy(); lc[third] += 0.0
    parts.append(("accent", rounded_box(lab, lc, r=0.6, n=2)))
    hc = np.array(horn_at, dtype=float); hc[ax] -= asg * 0.5
    parts.append(("alu", cylinder(9.0, 3.0, hc, axis[1], seg=24)))
    hc2 = np.array(horn_at, dtype=float); hc2[ax] += asg * 1.4
    parts.append(("servo", cylinder(3.2, 1.2, hc2, axis[1], seg=12)))


def plate(size, center, parts, mat="alu", r=1.0):
    parts.append((mat, rounded_box(size, center, r=r, n=2)))


# ---------------------------------------------------------------------------
# Kinematika + geometri per link
#   joint: (bus, slot, axis) atau None (root/fixed). origin = posisi sendi di frame parent.
# ---------------------------------------------------------------------------
LINKS = []   # dict(name, parent, origin, joint, axis, parts, optional)


def link(name, parent, origin, joint=None, axis=(0, 0, 0), optional=False):
    d = dict(name=name, parent=parent, origin=tuple(float(v) for v in origin), joint=joint,
             axis=tuple(float(v) for v in axis), parts=[], optional=optional)
    LINKS.append(d)
    return d["parts"]


HIP_Y = 32.0
SHOULDER_Y = 54.0


def build():
    LINKS.clear()
    # ---- pelvis (root) ----
    p = link("pelvis", None, (0, 0, 0))
    servo((0, 0, 0), "+z", "+x", p)                       # servo pinggang (yaw)
    for sy in (-1, 1):
        servo((0, sy * HIP_Y, -28), "-z", "+x", p)       # servo hip yaw
    plate((78, 92, 3), (2, 0, -13.5), p)
    plate((78, 3, 26), (2, 46, -2), p); plate((78, 3, 26), (2, -46, -2), p)
    plate((60, 84, 2), (2, 0, -1), p, "shell", r=1.0)

    # ---- dada ----
    c = link("chest", "pelvis", (0, 0, 4), (0, 0), (0, 0, 1))
    c.append(("shell", rounded_box((64, 100, 96), (-2, 0, 54), r=9, n=6)))
    c.append(("accent", rounded_box((3, 70, 50), (30.5, 0, 58), r=1.5, n=3)))
    c.append(("glass", rounded_box((2, 30, 10), (31.6, 0, 72), r=1.0, n=2)))   # panel status
    c.append(("servo", rounded_box((24, 72, 52), (-44, 0, 50), r=4, n=4)))     # baterai
    plate((3, 80, 60), (-31.5, 0, 52), c)
    for sy in (-1, 1):
        servo((0, sy * SHOULDER_Y, 82), "+y" if sy > 0 else "-y", "-z", c)   # bahu pitch
    servo((0, 0, 102), "+z", "-x", c)                                          # leher yaw
    c.append(("alu", cylinder(6, 8, (0, 0, 98), "z")))

    # ---- HP Android di punggung (opsional; Xiaomi 12 Pro 163.6 x 74.6 x 8.2 mm, lanskap) ----
    ph = link("phone", "chest", (-62, 0, 52), None, optional=True)
    ph.append(("phone", rounded_box((8.2, 163.6, 74.6), (0, 0, 0), r=3.5, n=5)))
    ph.append(("glass", rounded_box((1.0, 150, 66), (-4.4, 0, 0), r=2, n=3)))
    ph.append(("phone", rounded_box((2.5, 46, 32), (4.6, -50, 12), r=6, n=4)))   # modul kamera
    for k, (dy, dz) in enumerate([(-62, 18), (-44, 18), (-62, 4)]):
        ph.append(("glass", cylinder(5.5 if k == 0 else 4.2, 1.4, (6.3, dy, dz), "x", seg=20)))
    plate((14, 120, 3), (3.5, 0, -39), ph)   # dudukan

    # ---- kepala ----
    h = link("head", "chest", (0, 0, 104), (1, 0), (0, 0, 1))
    plate((20, 3, 18), (0, 12, 6), h); plate((20, 3, 18), (0, -12, 6), h)
    h.append(("shell", rounded_box((52, 58, 44), (3, 0, 30), r=10, n=6)))
    h.append(("glass", rounded_box((4, 46, 18), (28, 0, 32), r=4, n=3)))       # visor
    for k, dy in enumerate((-11, 0, 11)):                                       # modul kamera HP
        h.append(("alu", cylinder(4.6, 1.2, (30.4, dy, 32), "x", seg=20)))
        h.append(("glass", cylinder(3.4, 1.6, (30.8, dy, 32), "x", seg=20)))
    h.append(("accent", rounded_box((40, 3, 8), (3, 29.5, 34), r=1.2, n=2)))
    h.append(("accent", rounded_box((40, 3, 8), (3, -29.5, 34), r=1.2, n=2)))

    # ---- lengan ----
    for side, sy, bus in (("r", -1, 0), ("l", 1, 1)):
        s = link(f"{side}_shoulder", "chest", (0, sy * SHOULDER_Y, 82), (bus, 1), (0, -1, 0))
        plate((26, 3, 30), (0, sy * 3, 0), s)
        servo((0, sy * 22, 0), "-z" if sy < 0 else "+z", "+x", s)              # bahu roll
        plate((3, 22, 30), (-13, sy * 12, 0), s)

        u = link(f"{side}_upperarm", f"{side}_shoulder", (0, sy * 22, 0), (bus, 2), (0, 0, -1 if sy < 0 else 1))
        plate((18, 26, 3), (6, 0, 14.5), u); plate((18, 26, 3), (6, 0, -14.5), u)
        servo((14, 0, 0), "+x" if sy < 0 else "-x", "-z", u)                   # lengan atas yaw

        u2 = link(f"{side}_upperarm2", f"{side}_upperarm", (14, 0, 0), (bus, 3), (1 if sy < 0 else -1, 0, 0))
        u2.append(("shell", rounded_box((30, 26, 30), (30, 0, 0), r=6, n=4)))
        servo((48, sy * -14, 0), "+y", "+x", u2)                               # siku
        plate((28, 3, 24), (36, sy * 14, 0), u2)

        f = link(f"{side}_forearm", f"{side}_upperarm2", (48, 0, 0), (bus, 4), (0, 1, 0))
        plate((40, 3, 24), (16, 14, 0), f); plate((40, 3, 24), (16, -14, 0), f)
        f.append(("shell", rounded_box((36, 24, 26), (26, 0, 0), r=6, n=4)))
        f.append(("servo", rounded_box((18, 20, 18), (52, 0, 0), r=4, n=3)))   # tangan
        for dz in (-6, 0, 6):
            f.append(("servo", rounded_box((16, 6, 4.5), (66, sy * -5, dz), r=1.8, n=2)))
        f.append(("servo", rounded_box((12, 5, 5), (62, sy * 8, 0), r=1.8, n=2)))

    # ---- kaki ----
    for side, sy, bus in (("r", -1, 0), ("l", 1, 1)):
        y = link(f"{side}_hip_yaw", "pelvis", (0, sy * HIP_Y, -28), (bus, 5), (0, 0, 1))
        plate((34, 3, 24), (0, 13, -12), y); plate((34, 3, 24), (0, -13, -12), y)
        plate((34, 29, 3), (0, 0, -1.5), y)
        servo((0, 0, -30), "+x", "-z", y)                                      # hip roll

        r = link(f"{side}_hip_roll", f"{side}_hip_yaw", (0, 0, -30), (bus, 6), (1, 0, 0))
        plate((3, 30, 36), (13.5, 0, -14), r); plate((3, 30, 36), (-13.5, 0, -14), r)
        servo((0, 0, -32), "+y" if sy > 0 else "-y", "-z", r)                  # hip pitch

        t = link(f"{side}_thigh", f"{side}_hip_roll", (0, 0, -32), (bus, 7), (0, -1, 0))
        plate((26, 3, 70), (0, 14, -32), t); plate((26, 3, 70), (0, -14, -32), t)
        t.append(("shell", rounded_box((34, 24, 44), (2, 0, -30), r=7, n=4)))
        servo((0, 0, -65), "+y", "+z", t)                                      # lutut

        k = link(f"{side}_shin", f"{side}_thigh", (0, 0, -65), (bus, 8), (0, 1, 0))
        plate((26, 3, 70), (0, 14, -32), k); plate((26, 3, 70), (0, -14, -32), k)
        k.append(("shell", rounded_box((14, 26, 48), (14, 0, -30), r=5, n=4)))
        k.append(("accent", rounded_box((2, 18, 22), (21.5, 0, -28), r=0.8, n=2)))
        servo((0, 0, -65), "-y", "+z", k)                                      # ankle pitch

        a = link(f"{side}_ankle", f"{side}_shin", (0, 0, -65), (bus, 9), (0, -1, 0))
        plate((3, 30, 34), (13.5, 0, -14), a); plate((3, 30, 34), (-13.5, 0, -14), a)
        servo((0, 0, -32.5), "+x", "+z", a)                                    # ankle roll

        ft = link(f"{side}_foot", f"{side}_ankle", (0, 0, -32.5), (bus, 10), (1, 0, 0))
        oy = sy * 5.0
        plate((34, 3, 18), (0, 13, -9), ft); plate((34, 3, 18), (0, -13, -9), ft)
        ft.append(("alu", rounded_box((96, 58, 3), (9, oy, -19.5), r=6, n=3)))
        ft.append(("rubber", rounded_box((94, 56, 2.5), (9, oy, -22.25), r=6, n=3)))
        ft.append(("accent", rounded_box((24, 40, 2), (44, oy, -17.0), r=4, n=2)))


SOLE_Z = -23.5   # bawah sol relatif frame foot


# ---------------------------------------------------------------------------
# Keluaran
# ---------------------------------------------------------------------------
def write_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    with open(path, "wb") as f:
        f.write(b"leumpang-ajeg gen_model.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(n)
            n = n / ln if ln > 0 else n
            f.write(struct.pack("<12fH", *n, *t[0], *t[1], *t[2], 0))


def grouped(lk):
    out = {}
    for mat, tr in lk["parts"]:
        out.setdefault(mat, []).append(tr)
    return {m: np.concatenate(v) for m, v in out.items()}


def joint_name(lk):
    names = {(0, 0): "WAIST", (1, 0): "HEAD", 1: "SHOULDER_P", 2: "SHOULDER_R", 3: "SHOULDER_Y", 4: "ELBOW",
             5: "HIP_Y", 6: "HIP_R", 7: "HIP_P", 8: "KNEE", 9: "ANKLE_P", 10: "ANKLE_R"}
    b, s = lk["joint"]
    if s == 0:
        return names[(b, 0)]
    return ("R_" if b == 0 else "L_") + names[s]


def write_urdf(path, meshes):
    L = ['<?xml version="1.0"?>', '<!-- dibangkitkan tools/gen_model.py - jangan diedit manual -->',
         '<robot name="leumpang_ajeg">']
    for i, (n, rgb, met, rough) in enumerate(MATERIALS):
        L.append(f'  <material name="{n}"><color rgba="{rgb[0]} {rgb[1]} {rgb[2]} 1"/></material>')
    for lk in LINKS:
        L.append(f'  <link name="{lk["name"]}">')
        for mat, fn in meshes[lk["name"]]:
            L.append(f'    <visual><geometry><mesh filename="meshes/{fn}" scale="0.001 0.001 0.001"/></geometry>'
                     f'<material name="{mat}"/></visual>')
        if lk["name"].endswith("_foot"):
            L.append('    <collision><origin xyz="0.009 0 -0.0215"/><geometry><box size="0.096 0.058 0.004"/></geometry></collision>')
        L.append('  </link>')
    for lk in LINKS:
        if lk["parent"] is None:
            continue
        o = " ".join(f"{v / 1000:.4f}" for v in lk["origin"])
        if lk["joint"] is None:
            L.append(f'  <joint name="{lk["name"]}_fixed" type="fixed"><parent link="{lk["parent"]}"/>'
                     f'<child link="{lk["name"]}"/><origin xyz="{o}"/></joint>')
            continue
        ax = " ".join(f"{v:g}" for v in lk["axis"])
        L.append(f'  <!-- ICS bus {lk["joint"][0]} slot {lk["joint"][1]}: sudut = src / {CHG_SVA} rad -->')
        L.append(f'  <joint name="{joint_name(lk)}" type="revolute"><parent link="{lk["parent"]}"/>'
                 f'<child link="{lk["name"]}"/><origin xyz="{o}"/><axis xyz="{ax}"/>'
                 f'<limit lower="-2.3" upper="2.3" effort="2.5" velocity="6.0"/></joint>')
    L.append('</robot>')
    open(path, "w").write("\n".join(L) + "\n")


def write_header(path, meshes):
    idx = {lk["name"]: i for i, lk in enumerate(LINKS)}
    L = ["// Dibangkitkan tools/gen_model.py - JANGAN diedit manual.",
         "#pragma once", "#include <cstdint>", "", "namespace la::studio::gen {", "",
         f"constexpr float CHG_SVA = {CHG_SVA}f;", f"constexpr float SOLE_Z = {SOLE_Z}f;   // bawah sol di frame *_foot", "",
         "struct LinkDef { const char* name; int8_t parent; float ox, oy, oz; float ax, ay, az; int8_t bus, slot; bool optional; };",
         "struct MeshDef { int8_t link; uint8_t material; const char* file; };",
         "struct MaterialDef { const char* name; float r, g, b, metalness, roughness; };", "",
         "constexpr MaterialDef MATERIALS[] = {"]
    for n, rgb, met, rough in MATERIALS:
        L.append(f'    {{"{n}", {float(rgb[0])!r}f, {float(rgb[1])!r}f, {float(rgb[2])!r}f, {float(met)!r}f, {float(rough)!r}f}},')
    L += ["};", "", "constexpr LinkDef LINKS[] = {"]
    for lk in LINKS:
        par = idx[lk["parent"]] if lk["parent"] else -1
        b, s = lk["joint"] if lk["joint"] else (-1, -1)
        o = ", ".join(f"{float(v)!r}f" for v in lk["origin"]); a = ", ".join(f"{float(v)!r}f" for v in lk["axis"])
        L.append(f'    {{"{lk["name"]}", {par}, {o}, {a}, {b}, {s}, {"true" if lk["optional"] else "false"}}},')
    L += ["};", f"constexpr int LINK_COUNT = {len(LINKS)};", "", "constexpr MeshDef MESHES[] = {"]
    for lk in LINKS:
        for mat, fn in meshes[lk["name"]]:
            L.append(f'    {{{idx[lk["name"]]}, {MAT[mat]}, "{fn}"}},')
    L += ["};", "", "}  // namespace la::studio::gen", ""]
    open(path, "w").write("\n".join(L))


def fk_home():
    """Pose HOME (untuk pratinjau): src dari HOME_POSE core."""
    home = {(0, 1): -2700, (1, 1): -2760, (0, 7): 661, (1, 7): 661, (0, 8): 1322, (1, 8): 1322,
            (0, 9): 661, (1, 9): 661}
    T = {}
    for lk in LINKS:
        Rp, pp = (np.eye(3), np.zeros(3)) if lk["parent"] is None else T[lk["parent"]]
        p = pp + Rp @ np.array(lk["origin"])
        R = Rp
        if lk["joint"]:
            ang = home.get(tuple(lk["joint"]), 0) / CHG_SVA
            a = np.array(lk["axis"]); a = a / np.linalg.norm(a)
            K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            R = Rp @ (np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K)
        T[lk["name"]] = (R, p)
    return T


def preview(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    T = fk_home()
    fig = plt.figure(figsize=(10, 8), dpi=100)
    light = np.array([0.5, -0.4, 0.75]); light /= np.linalg.norm(light)
    for k, (el, az) in enumerate([(12, -35), (8, 210)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        for lk in LINKS:
            R, p = T[lk["name"]]
            for mat, tr in grouped(lk).items():
                w = tr @ R.T + p
                n = np.cross(w[:, 1] - w[:, 0], w[:, 2] - w[:, 0])
                n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
                base = np.array(MATERIALS[MAT[mat]][1])
                shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
                cols = np.clip(base[None, :] * shade[:, None] * 1.1 + 0.03, 0, 1)
                ax.add_collection3d(Poly3DCollection(w, facecolors=cols, edgecolors="none"))
        ax.set_xlim(-100, 100); ax.set_ylim(-100, 100); ax.set_zlim(-270, 170)
        ax.set_box_aspect((1, 1, 2.2), zoom=0.95); ax.view_init(el, az); ax.set_axis_off()
    fig.suptitle("Leumpang Ajeg - model parametrik (pose HOME)")
    fig.tight_layout()
    fig.savefig(path)


def main():
    build()
    mdir = os.path.join(ROOT, "model", "meshes")
    os.makedirs(mdir, exist_ok=True)
    for f in os.listdir(mdir):
        if f.endswith(".stl"):
            os.remove(os.path.join(mdir, f))
    meshes = {}
    ntri = 0
    for lk in LINKS:
        meshes[lk["name"]] = []
        for mat, tr in grouped(lk).items():
            fn = f'{lk["name"]}.{mat}.stl'
            write_stl(os.path.join(mdir, fn), tr)
            meshes[lk["name"]].append((mat, fn))
            ntri += len(tr)
    write_urdf(os.path.join(ROOT, "model", "leumpang.urdf"), meshes)
    os.makedirs(os.path.join(ROOT, "studio", "lib"), exist_ok=True)
    write_header(os.path.join(ROOT, "studio", "lib", "robot_model_gen.h"), meshes)
    print(f"{len(LINKS)} link, {sum(len(v) for v in meshes.values())} mesh, {ntri} segitiga")
    if "--preview" in sys.argv:
        os.makedirs(os.path.join(ROOT, "docs", "img"), exist_ok=True)
        preview(os.path.join(ROOT, "docs", "img", "model.png"))
        print("pratinjau -> docs/img/model.png")


if __name__ == "__main__":
    main()
