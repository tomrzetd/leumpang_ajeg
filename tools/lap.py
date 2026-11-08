#!/usr/bin/env python3
"""LAP (Leumpang Ajeg Protocol) v1 - implementasi Python (stdlib saja).

Cermin dari core/include/leumpang/lap.h. Dipakai oleh lapctl.py, tes lintas-bahasa,
dan prototipe aplikasi desktop. Spesifikasi: docs/PROTOCOL.md
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Iterator, List, Optional

VERSION = 1
MAX_BODY = 240
TCP_PORT = 5760

# ---- tipe pesan ----
HELLO, PING, PONG, ACK = 0x01, 0x02, 0x03, 0x04
TELEMETRY, SERVO_STATE, GAIT_STATE, CONFIG, LOG = 0x10, 0x11, 0x12, 0x13, 0x14
CMD, SET_PARAM, SET_TRIM, STREAM_CFG = 0x20, 0x21, 0x22, 0x23
GAIT_INTENT, PERCEPTION = 0x30, 0x31
MOTION_BEGIN, MOTION_CHUNK, MOTION_END, MOTION_PLAY = 0x40, 0x41, 0x42, 0x43
SIM_IMU, SIM_FAULT = 0x70, 0x71

TYPE_NAMES = {v: k for k, v in dict(globals()).items() if k.isupper() and isinstance(v, int) and 0 < v < 0x80
              and k not in ("VERSION", "MAX_BODY", "TCP_PORT")}

ACK_STATUS = {0: "OK", 1: "BAD_LEN", 2: "UNSUPPORTED", 3: "REJECTED_STATE", 4: "BUSY", 5: "BAD_VALUE"}
ROLE_ROBOT, ROLE_PHONE, ROLE_DESKTOP, ROLE_SIM, ROLE_TOOL = range(5)
BOARDS = {0: "unknown", 1: "esp32s3", 2: "esp32p4", 3: "stm32h562", 4: "sim"}
CMD_RESET, CMD_GO, CMD_STOP, CMD_CAL, CMD_GET_CONFIG, CMD_HELLO = range(6)
CMD_NAMES = {"reset": CMD_RESET, "go": CMD_GO, "stop": CMD_STOP, "cal": CMD_CAL, "config": CMD_GET_CONFIG}

# urutan = enum la::ParamId
PARAMS = ["fallDeg", "rearmDeg", "imuStaleMs", "servoStep", "trigRollDeg", "trigPitchDeg",
          "footLift", "swayBase", "swayGain", "rollSign", "pitchSign", "imuSwap"]

TF_IMU_FRESH, TF_IMU_INIT, TF_CAL_DONE, TF_PERSIST_PENDING, TF_JIKU_LEFT = 1, 2, 4, 8, 16

MODES = {700: "RAW_ANGLES", 705: "FALLEN", 710: "RESET", 720: "READY", 730: "BALANCE", 740: "WALK_INIT",
         750: "WALK_HOLD", 760: "WALK_STEP", 770: "WALK_END", 780: "POSE_HOLD", 790: "ANGLE_DISP",
         791: "MIRROR_DBG"}

JOINT_NAMES = [
    ["WAIST", "R SHOULDER P", "R SHOULDER R", "R SHOULDER Y", "R ELBOW", "R HIP Y", "R HIP R",
     "R HIP P", "R KNEE", "R ANKLE P", "R ANKLE R"],
    ["HEAD", "L SHOULDER P", "L SHOULDER R", "L SHOULDER Y", "L ELBOW", "L HIP Y", "L HIP R",
     "L HIP P", "L KNEE", "L ANKLE P", "L ANKLE R"],
]


# ---------------------------------------------------------------------------
#  primitif
# ---------------------------------------------------------------------------
def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> Optional[bytes]:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        if code == 0 or i + code > n:
            return None
        i += 1
        for _ in range(code - 1):
            if data[i] == 0:
                return None
            out.append(data[i])
            i += 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


@dataclass
class Packet:
    type: int
    seq: int = 0
    body: bytes = b""

    def name(self) -> str:
        return TYPE_NAMES.get(self.type, f"0x{self.type:02X}")


def encode_frame(p: Packet) -> bytes:
    if len(p.body) > MAX_BODY:
        raise ValueError("body terlalu panjang")
    raw = bytes([p.type, p.seq & 0xFF]) + p.body
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


class Decoder:
    """Decoder stream; sampah/teks di antara frame dibuang."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.frames = self.crc_errors = self.cobs_errors = self.overflows = 0
        self._overflow = False

    def feed(self, data: bytes) -> Iterator[Packet]:
        for b in data:
            if b != 0:
                if len(self.buf) < 300:
                    self.buf.append(b)
                else:
                    self._overflow = True
                continue
            buf, ovf = bytes(self.buf), self._overflow
            self.buf.clear()
            self._overflow = False
            if not buf:
                continue
            if ovf:
                self.overflows += 1
                continue
            raw = cobs_decode(buf)
            if raw is None or len(raw) < 4 or len(raw) > MAX_BODY + 4:
                self.cobs_errors += 1
                continue
            if crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
                self.crc_errors += 1
                continue
            self.frames += 1
            yield Packet(raw[0], raw[1], raw[2:-2])


# ---------------------------------------------------------------------------
#  pesan
# ---------------------------------------------------------------------------
@dataclass
class Hello:
    proto_ver: int = VERSION
    role: int = ROLE_TOOL
    board: int = 0
    fw: tuple = (0, 0, 0)
    caps: int = 0
    name: str = "lapctl"
    FMT = "<BBBBBBI16s"

    def pack(self) -> bytes:
        return struct.pack(self.FMT, self.proto_ver, self.role, self.board, *self.fw, self.caps,
                           self.name.encode()[:16])

    @classmethod
    def unpack(cls, b: bytes) -> "Hello":
        v = struct.unpack_from(cls.FMT, b)
        return cls(v[0], v[1], v[2], (v[3], v[4], v[5]), v[6], v[7].rstrip(b"\0").decode(errors="replace"))


@dataclass
class Ack:
    ack_seq: int
    ack_type: int
    status: int

    @classmethod
    def unpack(cls, b: bytes) -> "Ack":
        return cls(*struct.unpack_from("<BBB", b))

    def status_name(self) -> str:
        return ACK_STATUS.get(self.status, str(self.status))


@dataclass
class Telemetry:
    ms: int
    mode: int
    flags: int
    roll_deg: float
    pitch_deg: float
    imu_age_ms: int
    imu_hz: int
    fwct: float
    auto_h: float
    loop_avg_us: int
    loop_max_us: int
    servo_phase_us: int
    overruns: int
    fb_ok: int
    cfg_gen: int
    FMT = "<IHHffhHffHHHIBI"

    @classmethod
    def unpack(cls, b: bytes) -> "Telemetry":
        return cls(*struct.unpack_from(cls.FMT, b))

    @property
    def mode_name(self) -> str:
        return MODES.get(self.mode, str(self.mode))


@dataclass
class ServoState:
    cmd: List[List[int]]
    fb: List[List[int]]
    trim: List[List[int]]
    valid: List[List[int]]

    @classmethod
    def unpack(cls, b: bytes) -> "ServoState":
        c, f, t, v = ([[0] * 11, [0] * 11] for _ in range(4))
        off = 0
        for bus in range(2):
            for s in range(11):
                c[bus][s], f[bus][s], t[bus][s], v[bus][s] = struct.unpack_from("<hhhB", b, off)
                off += 7
        return cls(c, f, t, v)


@dataclass
class GaitState:
    fwct: float
    fwct_end: float
    dxi: float
    dyi: float
    dxis: float
    dyis: float
    auto_h: float
    fh: float
    swx: float
    swy: float
    sw_max: float
    waist: float
    jikuasi: int
    kin: List[int] = field(default_factory=list)

    @classmethod
    def unpack(cls, b: bytes) -> "GaitState":
        v = struct.unpack_from("<12fb22h", b)
        return cls(*v[:13], list(v[13:]))


@dataclass
class Config:
    values: dict
    cfg_gen: int
    saved_gen: int

    @classmethod
    def unpack(cls, b: bytes) -> "Config":
        n = b[0]
        vals = struct.unpack_from(f"<{n}f", b, 1)
        cg, sg = struct.unpack_from("<II", b, 1 + 4 * n)
        return cls({PARAMS[i] if i < len(PARAMS) else f"p{i}": vals[i] for i in range(n)}, cg, sg)


# ---- pembuat paket host -> robot ----
def pk_hello(**kw) -> bytes:
    return Hello(**kw).pack()


def pk_cmd(cmd: int) -> bytes:
    return struct.pack("<B", cmd)


def pk_set_param(name_or_id, value: float) -> bytes:
    pid = PARAMS.index(name_or_id) if isinstance(name_or_id, str) else int(name_or_id)
    return struct.pack("<Bf", pid, float(value))


def pk_set_trim(bus: int, slot: int, value: int) -> bytes:
    return struct.pack("<BBh", bus, slot, value)


def pk_stream_cfg(tele_hz=20, servo_hz=5, gait_hz=0, log_on=1) -> bytes:
    return struct.pack("<BBBB", tele_hz, servo_hz, gait_hz, log_on)


def pk_gait_intent(vx: float, vy: float, yaw_rate: float, ttl_ms: int) -> bytes:
    return struct.pack("<fffH", vx, vy, yaw_rate, ttl_ms)


def pk_sim_imu(roll_deg: float, pitch_deg: float, ok: bool = True) -> bytes:
    return struct.pack("<ffB", roll_deg, pitch_deg, 1 if ok else 0)


def pk_sim_fault(bus: int, slot: int, dead: bool) -> bytes:
    return struct.pack("<BBB", bus, slot, 1 if dead else 0)


def parse(p: Packet):
    """Ubah Packet menjadi objek pesan (atau bytes/str untuk tipe sederhana)."""
    try:
        if p.type == HELLO:
            return Hello.unpack(p.body)
        if p.type == ACK:
            return Ack.unpack(p.body)
        if p.type == TELEMETRY:
            return Telemetry.unpack(p.body)
        if p.type == SERVO_STATE:
            return ServoState.unpack(p.body)
        if p.type == GAIT_STATE:
            return GaitState.unpack(p.body)
        if p.type == CONFIG:
            return Config.unpack(p.body)
        if p.type == LOG:
            return p.body.decode(errors="replace")
    except struct.error:
        return None
    return p.body


# ---------------------------------------------------------------------------
#  link (TCP / serial)
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, tcp: Optional[str] = None, serial_port: Optional[str] = None, baud: int = 115200):
        self.dec = Decoder()
        self.seq = 0
        if tcp:
            import socket
            host, _, port = tcp.partition(":")
            self.sock = socket.create_connection((host, int(port or TCP_PORT)), timeout=5)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.ser = None
        elif serial_port:
            import serial  # pip install pyserial
            self.ser = serial.Serial(serial_port, baud, timeout=0.05)
            self.sock = None
        else:
            raise ValueError("pilih tcp atau serial")

    def send(self, type_: int, body: bytes = b"") -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        f = encode_frame(Packet(type_, seq, body))
        (self.sock.sendall if self.sock else self.ser.write)(f)
        return seq

    def recv(self, timeout: float = 0.1) -> List[Packet]:
        if self.sock:
            self.sock.settimeout(timeout)
            try:
                data = self.sock.recv(4096)
                if not data:
                    raise ConnectionError("link ditutup")
            except TimeoutError:
                return []
            except OSError as e:
                if "timed out" in str(e):
                    return []
                raise
        else:
            self.ser.timeout = timeout
            data = self.ser.read(4096)
        return list(self.dec.feed(data))

    def close(self) -> None:
        (self.sock or self.ser).close()
