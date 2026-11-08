#!/usr/bin/env python3
"""bbx2csv.py - ubah rekaman blackbox .lbb (SD card / simulator) menjadi CSV.
  bbx2csv.py rekaman.lbb > rekaman.csv"""
import csv
import sys

import lap

if len(sys.argv) != 2:
    sys.exit(__doc__)
w = csv.writer(sys.stdout)
names = [f"{lap.JOINT_NAMES[b][i]}" for b in range(2) for i in range(11)]
w.writerow(["ms", "mode", "roll", "pitch", "loop_us", "fwct", "auto_h", "flags", "fb_valid"] + names)
for r in lap.read_blackbox(sys.argv[1]):
    w.writerow([r["ms"], lap.MODES.get(r["mode"], r["mode"]), r["roll"], r["pitch"], r["loop_us"], r["fwct"],
                r["auto_h"], r["flags"], f"0x{r['fb_valid']:06X}"] + r["cmd"])
