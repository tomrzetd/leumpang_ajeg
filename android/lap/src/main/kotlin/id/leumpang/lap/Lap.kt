// Protokol LAP v1 (Leumpang Ajeg Protocol) untuk JVM/Android - port dari core/src/lap.cpp.
// Frame: COBS( type | seq | body | crc16-CCITT-FALSE LE ) + 0x00. Little-endian.
package id.leumpang.lap

object Lap {
    const val VERSION = 1
    const val MAX_BODY = 240
    const val TCP_PORT = 5760

    // tipe pesan
    const val HELLO = 0x01; const val PING = 0x02; const val PONG = 0x03; const val ACK = 0x04
    const val TELEMETRY = 0x10; const val SERVO_STATE = 0x11; const val GAIT_STATE = 0x12
    const val CONFIG = 0x13; const val LOG = 0x14
    const val CMD = 0x20; const val SET_PARAM = 0x21; const val SET_TRIM = 0x22; const val STREAM_CFG = 0x23
    const val GAIT_INTENT = 0x30; const val PERCEPTION = 0x31
    const val MOTION_BEGIN = 0x40; const val MOTION_CHUNK = 0x41; const val MOTION_END = 0x42
    const val MOTION_PLAY = 0x43; const val MOTION_STATUS = 0x44; const val POSE_TARGET = 0x45
    const val EXT = 0x60; const val SIM_IMU = 0x70; const val SIM_FAULT = 0x71

    // CMD
    const val CMD_RESET = 0; const val CMD_GO = 1; const val CMD_STOP = 2; const val CMD_CAL = 3
    const val CMD_GET_CONFIG = 4; const val CMD_HELLO = 5; const val CMD_MOTION_LIST = 6

    // ACK
    const val ACK_OK = 0; const val ACK_BAD_LEN = 1; const val ACK_UNSUPPORTED = 2
    const val ACK_REJECTED_STATE = 3; const val ACK_BUSY = 4; const val ACK_BAD_VALUE = 5

    // peran & board
    const val ROLE_ROBOT = 0; const val ROLE_PHONE = 1; const val ROLE_DESKTOP = 2; const val ROLE_SIM = 3
    const val BOARD_ESP32S3 = 1; const val BOARD_ESP32P4 = 2; const val BOARD_STM32H562 = 3; const val BOARD_SIM = 4

    // caps
    const val CAP_WIFI = 1 shl 0; const val CAP_SD = 1 shl 1; const val CAP_GAIT_INTENT = 1 shl 2
    const val CAP_MOTION = 1 shl 3; const val CAP_SIM = 1 shl 4; const val CAP_LCD = 1 shl 5
    const val CAP_USB_CDC = 1 shl 6; const val CAP_LIVE_POSE = 1 shl 7; const val CAP_PERCEPTION = 1 shl 8
    const val CAP_EXT = 1 shl 9

    // flag TELEMETRY
    const val TF_IMU_FRESH = 1; const val TF_IMU_INIT = 2; const val TF_CAL_DONE = 4
    const val TF_PERSIST_PENDING = 8; const val TF_JIKU_LEFT = 16; const val TF_INTENT_ACTIVE = 32
    const val TF_OBSTACLE_BLOCK = 64

    // ParamId
    const val P_GAIT_ENABLE = 12; const val P_GAIT_STEP_MAX = 13; const val P_GAIT_YAW_MAX = 14
    const val P_OBSTACLE_STOP_MM = 15

    val MODE_NAMES = mapOf(
        700 to "RAW", 705 to "FALLEN", 710 to "RESET", 720 to "READY", 730 to "BALANCE",
        740 to "WALK_INIT", 750 to "WALK_HOLD", 760 to "WALK_STEP", 770 to "WALK_END",
        780 to "POSE_HOLD", 790 to "ANGLE", 791 to "MIRROR", 800 to "WALK_CONT",
        810 to "MOTION", 820 to "LIVE_POSE",
    )
    const val MODE_FALLEN = 705; const val MODE_READY = 720; const val MODE_BALANCE = 730
    const val MODE_WALK_CONT = 800

    fun crc16(d: ByteArray, off: Int = 0, len: Int = d.size, init: Int = 0xFFFF): Int {
        var crc = init
        for (i in off until off + len) {
            crc = crc xor ((d[i].toInt() and 0xFF) shl 8)
            repeat(8) { crc = if (crc and 0x8000 != 0) ((crc shl 1) xor 0x1021) and 0xFFFF else (crc shl 1) and 0xFFFF }
        }
        return crc
    }

    fun cobsEncode(input: ByteArray, n: Int = input.size): ByteArray {
        val out = ByteArray(n + n / 254 + 2)
        var codeIdx = 0; var w = 1; var code = 1
        for (r in 0 until n) {
            val b = input[r]
            if (b.toInt() == 0) {
                out[codeIdx] = code.toByte(); codeIdx = w++; code = 1
            } else {
                out[w++] = b; code++
                if (code == 0xFF) { out[codeIdx] = code.toByte(); codeIdx = w++; code = 1 }
            }
        }
        out[codeIdx] = code.toByte()
        return out.copyOf(w)
    }

    /** null bila rusak */
    fun cobsDecode(input: ByteArray, n: Int = input.size): ByteArray? {
        val out = ByteArray(n)
        var r = 0; var w = 0
        while (r < n) {
            val code = input[r].toInt() and 0xFF
            if (code == 0 || r + code > n) return null
            r++
            for (i in 1 until code) {
                if (input[r].toInt() == 0) return null
                out[w++] = input[r++]
            }
            if (code != 0xFF && r < n) out[w++] = 0
        }
        return out.copyOf(w)
    }

    fun encodeFrame(type: Int, seq: Int, body: ByteArray): ByteArray {
        require(body.size <= MAX_BODY) { "body ${body.size} > $MAX_BODY" }
        val raw = ByteArray(body.size + 4)
        raw[0] = type.toByte(); raw[1] = seq.toByte()
        System.arraycopy(body, 0, raw, 2, body.size)
        val c = crc16(raw, 0, body.size + 2)
        raw[body.size + 2] = c.toByte(); raw[body.size + 3] = (c shr 8).toByte()
        val enc = cobsEncode(raw)
        return enc + byteArrayOf(0)
    }
}

class Packet(val type: Int, val seq: Int, val body: ByteArray)

/** Decoder aliran byte; sampah/teks di antara frame dibuang. */
class Decoder {
    private val buf = ByteArray(Lap.MAX_BODY + 4 + (Lap.MAX_BODY + 4) / 254 + 2)
    private var n = 0
    private var overflow = false
    var frames = 0L; private set
    var crcErrors = 0L; private set
    var cobsErrors = 0L; private set

    fun feed(b: Byte): Packet? {
        if (b.toInt() != 0) {
            if (n < buf.size) buf[n++] = b else overflow = true
            return null
        }
        val len = n; val ovf = overflow
        n = 0; overflow = false
        if (len == 0 || ovf) return null
        val raw = Lap.cobsDecode(buf, len)
        if (raw == null || raw.size < 4 || raw.size > Lap.MAX_BODY + 4) { cobsErrors++; return null }
        val got = (raw[raw.size - 2].toInt() and 0xFF) or ((raw[raw.size - 1].toInt() and 0xFF) shl 8)
        if (Lap.crc16(raw, 0, raw.size - 2) != got) { crcErrors++; return null }
        frames++
        return Packet(raw[0].toInt() and 0xFF, raw[1].toInt() and 0xFF, raw.copyOfRange(2, raw.size - 2))
    }
}

class Writer {
    private val b = java.io.ByteArrayOutputStream()
    fun u8(v: Int) = apply { b.write(v and 0xFF) }
    fun u16(v: Int) = apply { u8(v); u8(v shr 8) }
    fun i16(v: Int) = u16(v)
    fun u32(v: Long) = apply { u16((v and 0xFFFF).toInt()); u16(((v shr 16) and 0xFFFF).toInt()) }
    fun f32(v: Float) = u32(java.lang.Float.floatToRawIntBits(v).toLong() and 0xFFFFFFFFL)
    fun fixstr(s: String, n: Int) = apply { val a = s.toByteArray(Charsets.ISO_8859_1); for (i in 0 until n) u8(if (i < a.size) a[i].toInt() else 0) }
    fun bytes(a: ByteArray) = apply { b.write(a) }
    fun toByteArray(): ByteArray = b.toByteArray()
}

class Reader(private val d: ByteArray) {
    private var i = 0
    var ok = true; private set
    val remaining get() = if (ok) d.size - i else 0
    fun u8(): Int { if (i >= d.size) { ok = false; return 0 }; return d[i++].toInt() and 0xFF }
    fun i8(): Int = u8().toByte().toInt()
    fun u16(): Int = u8() or (u8() shl 8)
    fun i16(): Int = u16().toShort().toInt()
    fun u32(): Long = (u16().toLong()) or (u16().toLong() shl 16)
    fun f32(): Float = java.lang.Float.intBitsToFloat(u32().toInt())
    fun fixstr(n: Int): String { val a = ByteArray(n) { u8().toByte() }; val z = a.indexOf(0); return String(a, 0, if (z < 0) n else z, Charsets.ISO_8859_1) }
    fun bytes(n: Int): ByteArray = ByteArray(n) { u8().toByte() }
}
