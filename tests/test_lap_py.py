#!/usr/bin/env python3
"""Tes lintas-bahasa: klien Python (tools/lap.py) <-> simulator C++ (leumpang_sim) via TCP.
Pemakaian: test_lap_py.py <path leumpang_sim>"""
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import lap  # noqa: E402

fails = 0


def check(cond, msg):
    global fails
    print(("ok    " if cond else "FAIL  ") + msg)
    fails += 0 if cond else 1


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    port = free_port()
    sim = subprocess.Popen([sys.argv[1], "--port", str(port), "--no-stdin", "--quiet", "--seconds", "30", "--dead", "1:3"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                link = lap.Link(tcp=f"127.0.0.1:{port}")
                break
            except OSError:
                time.sleep(0.1)
        else:
            check(False, "terhubung ke simulator")
            return
        link.send(lap.HELLO, lap.pk_hello(role=lap.ROLE_DESKTOP, name="pytest"))
        link.send(lap.STREAM_CFG, lap.pk_stream_cfg(20, 5, 10, 1))
        state = {"tele": None, "hello": None, "config": None, "servo": None, "gait": None, "acks": {}, "logs": []}

        def pump(sec, until=None):
            end = time.time() + sec
            last_ping = 0
            while time.time() < end:
                if time.time() - last_ping > 0.5:
                    link.send(lap.PING)
                    last_ping = time.time()
                for p in link.recv(0.05):
                    m = lap.parse(p)
                    key = {lap.TELEMETRY: "tele", lap.HELLO: "hello", lap.CONFIG: "config", lap.SERVO_STATE: "servo",
                           lap.GAIT_STATE: "gait"}.get(p.type)
                    if key:
                        state[key] = m
                    elif p.type == lap.ACK:
                        state["acks"][m.ack_seq] = m
                    elif p.type == lap.LOG:
                        state["logs"].append(m)
                if until and until():
                    return True
            return False

        pump(0.5)
        h = state["hello"]
        check(h is not None and h.board == 4 and h.name == "leumpang-sim" and h.proto_ver == lap.VERSION,
              "HELLO dari simulator (board=sim, proto v1)")
        check(state["config"] is not None and abs(state["config"].values["fallDeg"] - 20.05) < 0.1,
              "CONFIG awal: fallDeg 20.05 (0.35 rad)")
        ready = pump(6, lambda: state["tele"] and state["tele"].mode == 720 and state["tele"].flags & lap.TF_CAL_DONE)
        check(ready, "TELEMETRY: READY + kalibrasi selesai")
        s = state["servo"]
        check(s is not None and s.valid[1][3] == 0 and s.valid[0][3] == 1, "SERVO_STATE: servo L slot 3 mati terlihat")
        check(state["gait"] is not None and 140 <= state["gait"].auto_h <= 185.1, "GAIT_STATE terdekode (autoH wajar)")

        seq = link.send(lap.SET_PARAM, lap.pk_set_param("fallDeg", 25))
        pump(1, lambda: state["config"] and abs(state["config"].values["fallDeg"] - 25) < 0.01)
        check(state["acks"].get(seq) and state["acks"][seq].status == 0, "SET_PARAM -> ACK OK")
        check(abs(state["config"].values["fallDeg"] - 25) < 0.01, "CONFIG broadcast setelah param berubah")

        seq = link.send(lap.SET_TRIM, lap.pk_set_trim(0, 8, -90))
        pump(1, lambda: state["servo"] and state["servo"].trim[0][8] == -90)
        check(state["servo"].trim[0][8] == -90, "SET_TRIM di READY -> trim terlihat di SERVO_STATE")

        seq = link.send(lap.CMD, lap.pk_cmd(lap.CMD_GO))
        ok = pump(1.5, lambda: state["tele"].mode == 730)
        check(ok, "CMD go -> BALANCE (730)")

        link.send(lap.SIM_IMU, lap.pk_sim_imu(0, 40))
        ok = pump(1.5, lambda: state["tele"].mode == 705)
        check(ok, "SIM_IMU pitch 40 deg -> FALLEN (705)")
        check(any("FALL DETECTED" in l for l in state["logs"]), "LOG diteruskan ke host")

        link.send(lap.SIM_IMU, lap.pk_sim_imu(0, 0))
        pump(0.3)
        link.send(lap.CMD, lap.pk_cmd(lap.CMD_RESET))
        ok = pump(3, lambda: state["tele"].mode == 720)
        check(ok, "re-arm: CMD reset saat tegak -> READY")

        seq = link.send(lap.GAIT_INTENT, lap.pk_gait_intent(10, 0, 0, 500))
        pump(0.5, lambda: seq in state["acks"])
        check(state["acks"].get(seq) and state["acks"][seq].status_name() == "UNSUPPORTED", "GAIT_INTENT -> UNSUPPORTED (Fase 1)")

        # sampah di stream tidak memutus link
        link.sock.sendall(b"teks acak\x00\x01\x02\x03\x00" + bytes(range(1, 200)) + b"\x00")
        before = state["tele"].ms
        pump(0.5)
        check(state["tele"].ms > before, "link tetap hidup setelah byte sampah")
        link.close()
    finally:
        sim.terminate()
        try:
            sim.wait(3)
        except subprocess.TimeoutExpired:
            sim.kill()


if __name__ == "__main__":
    main()
    print(f"== test_lap_py: {'LULUS' if fails == 0 else f'{fails} GAGAL'}")
    sys.exit(1 if fails else 0)
