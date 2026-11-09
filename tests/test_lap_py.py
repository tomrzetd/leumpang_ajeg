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
    sim = subprocess.Popen([sys.argv[1], "--port", str(port), "--no-stdin", "--quiet", "--seconds", "40", "--dead", "1:3"],
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
        check(state["acks"].get(seq) and state["acks"][seq].status_name() == "REJECTED_STATE",
              "GAIT_INTENT saat READY / gaitEnable=0 -> REJECTED_STATE")

        # ---- Fase 1: motion upload + play ----
        blob = open(os.path.join(os.path.dirname(__file__), "..", "motions", "lambai.lam"), "rb").read()
        link.pending = []
        ack = link.upload_motion(1, blob)
        check(ack is not None and ack.status == 0, f"unggah lambai.lam ({len(blob)} B) via MOTION_BEGIN/CHUNK/END")
        link.send(lap.MOTION_PLAY, lap.pk_motion_play(1))
        played = pump(2, lambda: state["tele"].mode == 810)
        check(played, "MOTION_PLAY -> mode 810")
        back = pump(6, lambda: state["tele"].mode == 720)
        check(back, "motion selesai (returnHome) -> RESET -> READY")

        # ---- Fase 1: gait kontinu ----
        link.send(lap.SET_PARAM, lap.pk_set_param("gaitEnable", 1))
        link.send(lap.CMD, lap.pk_cmd(lap.CMD_GO))
        pump(1, lambda: state["tele"].mode == 730)
        swaps, last_j, t_end = 0, None, time.time() + 2.0
        while time.time() < t_end:
            link.send(lap.GAIT_INTENT, lap.pk_gait_intent(6, 0, 1, 400))
            pump(0.2)
            j = state["tele"].flags & lap.TF_JIKU_LEFT
            swaps += 1 if (last_j is not None and j != last_j) else 0
            last_j = j
        check(state["tele"].mode == 800 and state["tele"].flags & lap.TF_INTENT_ACTIVE, "GAIT_INTENT berulang -> WALK_CONT (800)")
        check(swaps >= 4, f"kaki tumpu berganti saat melangkah ({swaps}x terlihat di telemetry 20 Hz)")
        stopped = pump(4, lambda: state["tele"].mode == 730)
        check(stopped, "niat berhenti -> kembali BALANCE")

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


def test_lapctl(sim_path):
    """Fase 4A: lapctl (berkas SD, profil, alat servo) melawan leumpang_sim --sd."""
    import json
    import shutil
    import tempfile
    sd = tempfile.mkdtemp(prefix="la_sd_")
    work = tempfile.mkdtemp(prefix="la_work_")
    port = free_port()
    sim = subprocess.Popen([sim_path, "--port", str(port), "--no-stdin", "--quiet", "--seconds", "30", "--sd", sd],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    lapctl = os.path.join(os.path.dirname(__file__), "..", "tools", "lapctl.py")

    def run(*args):
        r = subprocess.run([sys.executable, lapctl, "--tcp", f"127.0.0.1:{port}", *args], cwd=work,
                           capture_output=True, text=True, timeout=20)
        return r.returncode, r.stdout + r.stderr

    try:
        for _ in range(50):
            try:
                socket.create_connection(("127.0.0.1", port), timeout=1).close()
                break
            except OSError:
                time.sleep(0.1)
        rc, out = run("hello")
        check(rc == 0 and "caps=" in out and int(out.split("caps=0x")[1][:8], 16) & lap.CAP_FILES, "lapctl hello: CAP_FILES")
        script = b"import leumpang\n" + bytes(range(32, 127)) * 5 + b"\n"
        open(os.path.join(work, "jalan.py"), "wb").write(script)
        rc, _ = run("mkdir", "scripts")
        rc2, _ = run("put", "jalan.py", "scripts/jalan.py")
        check(rc == 0 and rc2 == 0 and open(os.path.join(sd, "scripts", "jalan.py"), "rb").read() == script,
              f"lapctl put ({len(script)} B, multi-chunk) tersimpan di SD")
        rc, out = run("ls", "scripts")
        check(rc == 0 and "jalan.py" in out and str(len(script)) in out, "lapctl ls")
        os.remove(os.path.join(work, "jalan.py"))
        rc, _ = run("get", "scripts/jalan.py")
        check(rc == 0 and open(os.path.join(work, "jalan.py"), "rb").read() == script, "lapctl get identik")
        rc, _ = run("mv", "scripts/jalan.py", "scripts/b.py")
        rc2, _ = run("rm", "scripts/b.py")
        check(rc == 0 and rc2 == 0 and not os.listdir(os.path.join(sd, "scripts")), "lapctl mv + rm")
        rc, out = run("get", "../etc/passwd")
        check(rc != 0 and "BAD_PATH" in out, "jalur di luar SD ditolak")

        prof = {"name": "lengan-uji", "kind": "generic",
                "servos": [{"bus": 0, "slot": 0, "id": 0, "name": "BASE", "model": "KRS-3304"}]}
        open(os.path.join(work, "p.json"), "w").write(json.dumps(prof))
        rc, _ = run("profile", "put", "p.json")
        check(rc == 0 and json.load(open(os.path.join(sd, "config", "robot.json")))["name"] == "lengan-uji",
              "lapctl profile put -> config/robot.json")
        open(os.path.join(work, "rusak.json"), "w").write("{ bukan json")
        rc, _ = run("profile", "put", "rusak.json")
        check(rc != 0, "profil dengan sintaks rusak tidak diunggah")

        time.sleep(1.5)                                        # tunggu READY
        rc, out = run("servo", "scan")
        check(rc != 0 and "BAD_MODE" in out, "servo scan dari READY ditolak")
        run("cmd", "stop")
        rc, out = run("servo", "scan")
        check(rc == 0 and out.count("11 servo") == 2, "servo scan: 11 + 11 servo")
        rc, out = run("servo", "set", "R", "3", "speed", "90")
        rc2, out2 = run("servo", "get", "R", "3", "speed")
        check(rc == 0 and rc2 == 0 and "speed = 90" in out2, "servo set/get speed")
        rc, out = run("servo", "id", "L")
        check(rc != 0 and "NO_REPLY" in out, "baca ID dengan banyak servo = NO_REPLY")
        rc, out = run("servo", "eeprom", "L", "2")
        check(rc == 0 and out.count("\n") >= 4, "servo eeprom (64 nibble)")
        rc, _ = run("servo", "exit")
        check(rc == 0, "servo exit -> FALLEN")

        # Fase 4B: stik virtual lewat lapctl pad
        rc, out = run("pad", "l1", "options", "--secs", "2.5")
        check(rc == 0 and "stik aktif" in out, "lapctl pad: stik aktif")
        check("rgb=" in out, f"lapctl pad: PAD_FEEDBACK diterima ({out.strip().splitlines()[-1] if out.strip() else ''})")
        rc, out = run("pad", "l1", "x", "--secs", "1.5")
        check(rc == 0 and "mode=730" in out, "lapctl pad l1 x = GO (BALANCE)")
        check("rgb=(0,0,80)" in out, "lightbar biru saat BALANCE")
        time.sleep(0.6)                                         # > 250 ms: stik dianggap hilang
        rc, out = run("pad", "touch", "--secs", "1")
        check("stik aktif" in out and "E-STOP" not in out, "touchpad dalam 1 s pertama setelah sambung diabaikan")
        time.sleep(0.6)
        rc, out = run("pad", "touch", "--press-after", "1.3", "--secs", "2")
        check("E-STOP dari stik" in out and "rgb=(80,0,0)" in out, "touchpad setelah 1 s = E-STOP, lightbar merah")
    finally:
        sim.terminate()
        try:
            sim.wait(3)
        except subprocess.TimeoutExpired:
            sim.kill()
        shutil.rmtree(sd, ignore_errors=True)
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    main()
    test_lapctl(sys.argv[1])
    print(f"== test_lap_py: {'LULUS' if fails == 0 else f'{fails} GAGAL'}")
    sys.exit(1 if fails else 0)
