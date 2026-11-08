#!/usr/bin/env python3
"""lam.py - buat / periksa template gerak LAM1 dari JSON.

JSON:
{
  "name": "lambai",
  "returnHome": true,
  "frames": [
    {"ms": 600, "interp": "cosine", "joints": {"R SHOULDER R": 900, "R ELBOW": -600}},
    {"ms": 400, "joints": {"R SHOULDER R": 900, "R ELBOW": 0}}
  ]
}
Sendi yang tidak disebut memakai nilai frame sebelumnya (frame pertama: pose HOME).
Nilai = offset servo (hitungan, ruang yang sama dengan 'kin' telemetry).

  lam.py build lambai.json lambai.lam
  lam.py show lambai.lam
"""
import json
import sys

import lap

HOME = {  # HOME_POSE core (ruang src), urutan [bus][slot]
    0: [0, -2700, 0, 0, 0, 0, 0, 661, 1322, 661, 0],
    1: [0, -2760, 0, 0, 0, 0, 0, 661, 1322, 661, 0],
}
INTERP = {"cosine": lap.MI_COSINE, "linear": lap.MI_LINEAR, "step": lap.MI_STEP}


def build(src, dst):
    spec = json.load(open(src))
    names = {n: (b, i) for b in range(2) for i, n in enumerate(lap.JOINT_NAMES[b])}
    cur = [list(HOME[0]), list(HOME[1])]
    frames = []
    for k, f in enumerate(spec["frames"]):
        for n, v in f.get("joints", {}).items():
            if n not in names:
                sys.exit(f"frame {k}: sendi '{n}' tidak dikenal. Pilihan: {', '.join(names)}")
            b, i = names[n]
            cur[b][i] = int(v)
        frames.append(lap.MotionFrame(int(f["ms"]), [list(cur[0]), list(cur[1])], INTERP[f.get("interp", "cosine")]))
    blob = lap.encode_motion(spec.get("name", "motion"), frames, lap.MF_RETURN_HOME if spec.get("returnHome") else 0)
    open(dst, "wb").write(blob)
    print(f"{dst}: {len(frames)} frame, {len(blob)} B")


def show(path):
    name, frames, flags = lap.decode_motion(open(path, "rb").read())
    print(f"'{name}' {len(frames)} frame, flags=0x{flags:X}")
    for k, f in enumerate(frames):
        diff = [f"{lap.JOINT_NAMES[b][i]}={f.joints[b][i]}" for b in range(2) for i in range(11) if f.joints[b][i] != HOME[b][i]]
        print(f"  #{k} {f.dur_ms} ms {['cos', 'lin', 'step'][f.interp]}: {', '.join(diff) or 'HOME'}")


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "build":
        build(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3 and sys.argv[1] == "show":
        show(sys.argv[2])
    else:
        print(__doc__)
        sys.exit(2)
