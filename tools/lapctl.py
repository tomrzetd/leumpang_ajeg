#!/usr/bin/env python3
"""lapctl - klien baris-perintah protokol LAP untuk robot / simulator Leumpang Ajeg.

Contoh:
  lapctl.py --tcp 127.0.0.1 hello
  lapctl.py --tcp 192.168.4.1 watch --servo
  lapctl.py --tcp 127.0.0.1 cmd go
  lapctl.py --tcp 127.0.0.1 param fallDeg 22
  lapctl.py --tcp 127.0.0.1 trim 0 8 120
  lapctl.py --tcp 127.0.0.1 sim-imu 0 40       (simulator: miringkan pitch 40 deg)
  lapctl.py --serial /dev/ttyACM0 watch        (port USB/serial board, dibagi dengan konsol)
  lapctl.py --tcp 127.0.0.1 motion-upload wave.lam 0
  lapctl.py --tcp 127.0.0.1 motion-play 0
  lapctl.py --tcp 127.0.0.1 param gaitEnable 1 && lapctl.py --tcp 127.0.0.1 gait 8 0 5

Berkas SD & profil robot (Fase 4A):
  lapctl.py --tcp 192.168.4.1 ls scripts
  lapctl.py --tcp 192.168.4.1 put jalan.py scripts/jalan.py
  lapctl.py --tcp 192.168.4.1 get poses/salam.json
  lapctl.py --tcp 192.168.4.1 profile get robot.json      (unduh /config/robot.json)
  lapctl.py --tcp 192.168.4.1 profile put robot.json      (unggah, lalu: cmd reboot)

Alat servo ICS 3.5 (robot harus FALLEN: cmd stop dulu, robot di stand):
  lapctl.py --tcp 192.168.4.1 servo scan
  lapctl.py --tcp 192.168.4.1 servo get R 3 stretch
  lapctl.py --tcp 192.168.4.1 servo set R 3 speed 100
  lapctl.py --tcp 192.168.4.1 servo id L            (HANYA satu servo terpasang di bus L!)
  lapctl.py --tcp 192.168.4.1 servo setid L 5
"""
import argparse
import sys
import time

import lap


def wait_ack(link: lap.Link, seq: int, timeout: float = 1.5):
    end = time.time() + timeout
    while time.time() < end:
        for p in link.recv(0.1):
            m = lap.parse(p)
            if p.type == lap.LOG:
                print("LOG", m.rstrip())
            if p.type == lap.ACK and m.ack_seq == seq:
                return m
    return None


def handshake(link: lap.Link, tele=0, servo=0, gait=0, log=1):
    link.send(lap.HELLO, lap.pk_hello())
    link.send(lap.STREAM_CFG, lap.pk_stream_cfg(tele, servo, gait, log))
    hello = None
    end = time.time() + 2
    while time.time() < end and hello is None:
        for p in link.recv(0.1):
            if p.type == lap.HELLO:
                hello = lap.parse(p)
    return hello


def file_cmd(link: lap.Link, a) -> int:
    try:
        if a.cmd == "ls":
            for name, size, is_dir in sorted(link.file_list(a.path), key=lambda e: (not e[2], e[0])):
                print(f"{'<DIR>' if is_dir else size:>8}  {name}{'/' if is_dir else ''}")
        elif a.cmd == "get":
            data = link.file_get(a.remote)
            out = a.local or a.remote.rsplit("/", 1)[-1]
            open(out, "wb").write(data)
            print(f"{a.remote} -> {out} ({len(data)} B)")
        elif a.cmd == "put":
            data = open(a.local, "rb").read()
            link.file_put(a.remote, data)
            print(f"{a.local} -> {a.remote} ({len(data)} B)")
        elif a.cmd == "rm":
            link.file_simple(lap.FILE_DELETE, a.path)
        elif a.cmd == "mkdir":
            link.file_simple(lap.FILE_MKDIR, a.path)
        elif a.cmd == "mv":
            link.file_rename(a.src, a.dst)
        elif a.action == "get":
            data = link.file_get("config/robot.json")
            open(a.file, "wb").write(data)
            print(f"profil -> {a.file} ({len(data)} B)")
        else:
            data = open(a.file, "rb").read()
            import json
            json.loads(data)                                   # cek sintaks sebelum mengunggah
            try:
                link.file_simple(lap.FILE_MKDIR, "config")
            except IOError:
                pass                                           # sudah ada
            link.file_put("config/robot.json", data)
            print(f"{a.file} -> config/robot.json ({len(data)} B). Berlaku setelah: lapctl.py ... cmd reboot")
    except (IOError, TimeoutError, ValueError) as e:
        print("gagal:", e, file=sys.stderr)
        return 1
    return 0


def _bus(s: str) -> int:
    s = s.upper()
    if s in ("R", "0"):
        return 0
    if s in ("L", "1"):
        return 1
    raise ValueError(f"bus '{s}' (pakai R/L)")


def servo_cmd(link: lap.Link, a) -> int:
    x = a.args
    try:
        if a.action in ("enter", "exit"):
            st, _, _ = link.servo_tool(lap.TOOL_ENTER if a.action == "enter" else lap.TOOL_EXIT)
        elif a.action == "scan":
            link.servo_tool(lap.TOOL_ENTER)
            st, _, d = link.servo_tool(lap.TOOL_SCAN, 0xFF)
            if st == 0:
                for b, name in ((0, "R"), (1, "L")):
                    m = int.from_bytes(d[4 * b:4 * b + 4], "little")
                    ids = [i for i in range(32) if m >> i & 1]
                    print(f"bus {name}: {len(ids):2d} servo  ID {ids}")
        else:
            link.servo_tool(lap.TOOL_ENTER)
            bus = _bus(x[0])
            if a.action == "id":
                st, v, _ = link.servo_tool(lap.TOOL_READ_ID, bus)
                if st == 0:
                    print(f"ID = {v}")
            elif a.action == "setid":
                st, v, _ = link.servo_tool(lap.TOOL_WRITE_ID, bus, 0, 0, int(x[1]))
                if st == 0:
                    print(f"ID baru = {v}")
            elif a.action == "get":
                st, v, _ = link.servo_tool(lap.TOOL_READ_PARAM, bus, int(x[1]), lap.SC_NAMES[x[2]])
                if st == 0:
                    print(f"{x[2]} = {v}")
            elif a.action == "set":
                st, v, _ = link.servo_tool(lap.TOOL_WRITE_PARAM, bus, int(x[1]), lap.SC_NAMES[x[2]], int(x[3]))
                if st == 0:
                    print(f"{x[2]} = {v} (RAM servo; hilang saat daya mati)")
            elif a.action == "eeprom":
                st, _, d = link.servo_tool(lap.TOOL_READ_EEPROM, bus, int(x[1]))
                if st == 0:
                    for o in range(0, len(d), 16):
                        print(f"{o:02X}: " + " ".join(f"{b:X}" for b in d[o:o + 16]))
            else:
                st, v, _ = link.servo_tool(lap.TOOL_READ_POS, bus, int(x[1]))
                if st == 0:
                    print(f"posisi = {v}")
    except (IndexError, KeyError, ValueError) as e:
        print("argumen salah:", e, file=sys.stderr)
        return 2
    except TimeoutError as e:
        print("timeout:", e, file=sys.stderr)
        return 1
    if st:
        hint = " (cmd stop dulu: alat hanya dari FALLEN)" if st == 1 else ""
        print("TOOL", lap.TOOL_STATUS.get(st, st) + hint, file=sys.stderr)
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--tcp", help="host[:port], default port 5760")
    g.add_argument("--serial", help="port serial USB CDC")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("hello")
    w = sub.add_parser("watch"); w.add_argument("--servo", action="store_true"); w.add_argument("--gait", action="store_true")
    w.add_argument("--seconds", type=float, default=0)
    c = sub.add_parser("cmd"); c.add_argument("name", choices=sorted(lap.CMD_NAMES))
    p = sub.add_parser("param"); p.add_argument("name", choices=lap.PARAMS); p.add_argument("value", type=float)
    t = sub.add_parser("trim"); t.add_argument("bus", type=int); t.add_argument("slot", type=int); t.add_argument("value", type=int)
    s = sub.add_parser("sim-imu"); s.add_argument("roll", type=float); s.add_argument("pitch", type=float)
    f = sub.add_parser("sim-fault"); f.add_argument("bus", type=int); f.add_argument("slot", type=int); f.add_argument("dead", type=int)
    mu = sub.add_parser("motion-upload"); mu.add_argument("file"); mu.add_argument("slot", type=int); mu.add_argument("--no-save", action="store_true")
    mp = sub.add_parser("motion-play"); mp.add_argument("slot", type=int)
    ms = sub.add_parser("motion-stop"); ms.add_argument("slot", type=int, nargs="?", default=0)
    sub.add_parser("motions")
    gt = sub.add_parser("gait", help="kirim GAIT_INTENT berulang"); gt.add_argument("vx", type=float, help="mm/langkah")
    gt.add_argument("yaw", type=float, help="derajat/langkah"); gt.add_argument("seconds", type=float)
    fl = sub.add_parser("ls", help="daftar berkas SD"); fl.add_argument("path", nargs="?", default="")
    fg = sub.add_parser("get", help="unduh berkas SD"); fg.add_argument("remote"); fg.add_argument("local", nargs="?")
    fp = sub.add_parser("put", help="unggah berkas ke SD"); fp.add_argument("local"); fp.add_argument("remote")
    fr = sub.add_parser("rm", help="hapus berkas/folder kosong"); fr.add_argument("path")
    fm = sub.add_parser("mkdir"); fm.add_argument("path")
    fv = sub.add_parser("mv"); fv.add_argument("src"); fv.add_argument("dst")
    pr = sub.add_parser("profile", help="profil robot /config/robot.json")
    pr.add_argument("action", choices=["get", "put"]); pr.add_argument("file", nargs="?", default="robot.json")
    sv = sub.add_parser("servo", help="alat servo ICS 3.5 (mode 830)")
    sv.add_argument("action", choices=["enter", "exit", "scan", "id", "setid", "get", "set", "eeprom", "pos"])
    sv.add_argument("args", nargs="*", help="bus (R/L/0/1), id, parameter (stretch/speed/current/temp), nilai")
    a = ap.parse_args()

    link = lap.Link(tcp=a.tcp, serial_port=a.serial)
    watch = a.cmd == "watch"
    hello = handshake(link, tele=10 if (watch or a.cmd == "gait") else 0, servo=2 if watch and a.servo else 0, gait=10 if watch and a.gait else 0)
    if hello is None:
        print("tidak ada HELLO dari robot", file=sys.stderr)
        return 2
    if a.cmd == "hello":
        print(f"{hello.name}  board={lap.BOARDS.get(hello.board)}  fw={'.'.join(map(str, hello.fw))}  "
              f"proto={hello.proto_ver}  caps=0x{hello.caps:08X}")
        return 0
    if watch:
        end = time.time() + a.seconds if a.seconds else float("inf")
        last_ping = 0.0
        while time.time() < end:
            if time.time() - last_ping > 1:
                link.send(lap.PING, b"")
                last_ping = time.time()
            for pk in link.recv(0.1):
                m = lap.parse(pk)
                if pk.type == lap.TELEMETRY:
                    print(f"[{m.ms:>8}] {m.mode_name:<10} R={m.roll_deg:+6.2f} P={m.pitch_deg:+6.2f} "
                          f"imu={m.imu_age_ms}ms/{m.imu_hz}Hz fb={m.fb_ok}/22 loop={m.loop_avg_us}/{m.loop_max_us}us "
                          f"ics={m.servo_phase_us}us ovr={m.overruns}")
                elif pk.type == lap.SERVO_STATE:
                    for b in range(2):
                        print("  " + " ".join(f"{lap.JOINT_NAMES[b][i][-6:]:>6}:{m.cmd[b][i]:5d}/{m.fb[b][i] if m.valid[b][i] else '----':>5}"
                                              for i in range(11)))
                elif pk.type == lap.GAIT_STATE:
                    print(f"  gait fwct={m.fwct:.0f}/{m.fwct_end:.0f} dxi={m.dxi:+.1f} dyi={m.dyi:+.1f} h={m.auto_h:.1f} fh={m.fh:.1f}")
                elif pk.type == lap.LOG:
                    print("LOG", m.rstrip())
                elif pk.type == lap.CONFIG:
                    print("CONFIG", {k: round(v, 3) for k, v in m.values.items()}, f"gen={m.cfg_gen} saved={m.saved_gen}")
        return 0
    if a.cmd in ("ls", "get", "put", "rm", "mkdir", "mv", "profile"):
        return file_cmd(link, a)
    if a.cmd == "servo":
        return servo_cmd(link, a)
    if a.cmd == "motion-upload":
        blob = open(a.file, "rb").read()
        name, frames, _ = lap.decode_motion(blob)
        ack = link.upload_motion(a.slot, blob, save=not a.no_save)
        print(f"unggah '{name}' ({len(frames)} frame, {len(blob)} B) -> slot {a.slot}:",
              ack.status_name() if ack else "TIMEOUT")
        return 0 if ack and ack.status == 0 else 1
    if a.cmd == "motions":
        link.send(lap.CMD, lap.pk_cmd(lap.CMD_MOTION_LIST))
        end = time.time() + 2
        while time.time() < end:
            for pk in link.pending + link.recv(0.1):
                if pk.type == lap.MOTION_STATUS:
                    for i, (st, nf, nm) in enumerate(lap.parse(pk).slots):
                        print(f"slot {i}: {st:<8} {nf:2d} frame  {nm}")
                    return 0
            link.pending.clear()
        print("tidak ada MOTION_STATUS", file=sys.stderr)
        return 1
    if a.cmd == "gait":
        end = time.time() + a.seconds
        while time.time() < end:
            ack = link.request(lap.GAIT_INTENT, lap.pk_gait_intent(a.vx, 0, a.yaw, 500), timeout=0.5)
            if ack and ack.status:
                print("GAIT_INTENT:", ack.status_name(), "(aktifkan: param gaitEnable 1, robot di BALANCE)")
                return 1
            for pk in link.pending:
                if pk.type == lap.TELEMETRY:
                    m = lap.parse(pk)
                    print(f"{m.mode_name:<10} R={m.roll_deg:+6.2f} P={m.pitch_deg:+6.2f} fwct={m.fwct:.0f}")
                elif pk.type == lap.LOG:
                    print("LOG", lap.parse(pk).rstrip())
            link.pending.clear()
            time.sleep(0.2)
        print("niat berhenti; robot menyelesaikan langkah lalu kembali BALANCE")
        return 0
    if a.cmd == "cmd":
        seq = link.send(lap.CMD, lap.pk_cmd(lap.CMD_NAMES[a.name]))
    elif a.cmd == "param":
        seq = link.send(lap.SET_PARAM, lap.pk_set_param(a.name, a.value))
    elif a.cmd == "trim":
        seq = link.send(lap.SET_TRIM, lap.pk_set_trim(a.bus, a.slot, a.value))
    elif a.cmd == "sim-imu":
        seq = link.send(lap.SIM_IMU, lap.pk_sim_imu(a.roll, a.pitch))
    elif a.cmd == "motion-play":
        seq = link.send(lap.MOTION_PLAY, lap.pk_motion_play(a.slot, True))
    elif a.cmd == "motion-stop":
        seq = link.send(lap.MOTION_PLAY, lap.pk_motion_play(a.slot, False))
    else:
        seq = link.send(lap.SIM_FAULT, lap.pk_sim_fault(a.bus, a.slot, bool(a.dead)))
    ack = wait_ack(link, seq)
    print("ACK", ack.status_name() if ack else "TIMEOUT")
    return 0 if ack and ack.status == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
