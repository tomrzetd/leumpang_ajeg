#!/usr/bin/env python3
"""lapctl - klien baris-perintah protokol LAP untuk robot / simulator Leumpang Ajeg.

Contoh:
  lapctl.py --tcp 127.0.0.1 hello
  lapctl.py --tcp 192.168.4.1 watch --servo
  lapctl.py --tcp 127.0.0.1 cmd go
  lapctl.py --tcp 127.0.0.1 param fallDeg 22
  lapctl.py --tcp 127.0.0.1 trim 0 8 120
  lapctl.py --tcp 127.0.0.1 sim-imu 0 40       (simulator: miringkan pitch 40 deg)
  lapctl.py --serial /dev/ttyACM0 watch        (Fase 1: USB CDC)
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
    a = ap.parse_args()

    link = lap.Link(tcp=a.tcp, serial_port=a.serial)
    watch = a.cmd == "watch"
    hello = handshake(link, tele=10 if watch else 0, servo=2 if watch and a.servo else 0, gait=10 if watch and a.gait else 0)
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
    if a.cmd == "cmd":
        seq = link.send(lap.CMD, lap.pk_cmd(lap.CMD_NAMES[a.name]))
    elif a.cmd == "param":
        seq = link.send(lap.SET_PARAM, lap.pk_set_param(a.name, a.value))
    elif a.cmd == "trim":
        seq = link.send(lap.SET_TRIM, lap.pk_set_trim(a.bus, a.slot, a.value))
    elif a.cmd == "sim-imu":
        seq = link.send(lap.SIM_IMU, lap.pk_sim_imu(a.roll, a.pitch))
    else:
        seq = link.send(lap.SIM_FAULT, lap.pk_sim_fault(a.bus, a.slot, bool(a.dead)))
    ack = wait_ack(link, seq)
    print("ACK", ack.status_name() if ack else "TIMEOUT")
    return 0 if ack and ack.status == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
