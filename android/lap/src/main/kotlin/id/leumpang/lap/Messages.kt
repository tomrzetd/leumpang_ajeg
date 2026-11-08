// Pesan LAP (layout identik dengan core/include/leumpang/lap.h).
package id.leumpang.lap

const val SLOTS = 11

data class Hello(
    val protoVer: Int = Lap.VERSION, val role: Int = Lap.ROLE_PHONE, val board: Int = 0,
    val fw: Triple<Int, Int, Int> = Triple(0, 8, 0), val caps: Long = 0, val name: String = "leumpang-mata",
) {
    fun enc(): ByteArray = Writer().u8(protoVer).u8(role).u8(board).u8(fw.first).u8(fw.second).u8(fw.third)
        .u32(caps).fixstr(name, 16).toByteArray()
    fun has(cap: Int) = (caps and cap.toLong()) != 0L
    companion object {
        fun dec(b: ByteArray): Hello? = Reader(b).run {
            val h = Hello(u8(), u8(), u8(), Triple(u8(), u8(), u8()), u32(), fixstr(16)); if (ok) h else null
        }
    }
}

data class Telemetry(
    val ms: Long, val mode: Int, val flags: Int, val rollDeg: Float, val pitchDeg: Float, val imuAgeMs: Int,
    val imuHz: Int, val fwct: Float, val autoH: Float, val loopAvgUs: Int, val loopMaxUs: Int,
    val servoPhaseUs: Int, val overruns: Long, val fbOk: Int, val cfgGen: Long,
) {
    val modeName get() = Lap.MODE_NAMES[mode] ?: mode.toString()
    val obstacleBlock get() = flags and Lap.TF_OBSTACLE_BLOCK != 0
    companion object {
        fun dec(b: ByteArray): Telemetry? = Reader(b).run {
            val t = Telemetry(u32(), u16(), u16(), f32(), f32(), i16(), u16(), f32(), f32(), u16(), u16(), u16(), u32(), u8(), u32())
            if (ok) t else null
        }
    }
}

/** Posisi servo mentah per [bus][slot]. */
class ServoState(val cmd: Array<IntArray>, val fb: Array<IntArray>, val trim: Array<IntArray>, val valid: Array<BooleanArray>) {
    companion object {
        fun dec(b: ByteArray): ServoState? {
            val r = Reader(b)
            val cmd = Array(2) { IntArray(SLOTS) }; val fb = Array(2) { IntArray(SLOTS) }
            val trim = Array(2) { IntArray(SLOTS) }; val valid = Array(2) { BooleanArray(SLOTS) }
            for (bus in 0..1) for (s in 0 until SLOTS) { cmd[bus][s] = r.i16(); fb[bus][s] = r.i16(); trim[bus][s] = r.i16(); valid[bus][s] = r.u8() != 0 }
            return if (r.ok) ServoState(cmd, fb, trim, valid) else null
        }
    }
}

/** Nilai sendi ruang src [bus][slot] dari GAIT_STATE (urutan kin = ServoPositions). */
class GaitState(val autoH: Float, val fh: Float, val jikuasi: Int, val src: Array<IntArray>) {
    val headYawSrc get() = src[1][0]
    companion object {
        fun dec(b: ByteArray): GaitState? {
            val r = Reader(b)
            val f = FloatArray(12) { r.f32() }
            val jiku = r.i8()
            val k = IntArray(22) { r.i16() }
            if (!r.ok) return null
            val o = Array(2) { IntArray(SLOTS) }
            for (bus in 0..1) {
                o[bus][7] = k[0 + bus]; o[bus][6] = k[2 + bus]; o[bus][5] = k[4 + bus]; o[bus][8] = k[6 + bus]
                o[bus][9] = k[8 + bus]; o[bus][10] = k[10 + bus]; o[bus][1] = k[12 + bus]; o[bus][2] = k[14 + bus]
                o[bus][3] = k[16 + bus]; o[bus][4] = k[18 + bus]
            }
            o[1][0] = k[20]; o[0][0] = k[21]
            return GaitState(f[6], f[7], jiku, o)
        }
    }
}

class Config(val values: FloatArray, val cfgGen: Long, val savedGen: Long) {
    companion object {
        fun dec(b: ByteArray): Config? {
            val r = Reader(b)
            val n = r.u8()
            val v = FloatArray(n) { r.f32() }
            val c = Config(v, r.u32(), r.u32())
            return if (r.ok) c else null
        }
    }
}

data class MotionSlot(val state: Int, val frames: Int, val name: String)

fun decodeMotionStatus(b: ByteArray): List<MotionSlot>? {
    val r = Reader(b)
    val l = List(4) { MotionSlot(r.u8(), r.u8(), r.fixstr(16)) }
    return if (r.ok) l else null
}

data class Ack(val seq: Int, val type: Int, val status: Int) {
    companion object { fun dec(b: ByteArray) = Reader(b).run { val a = Ack(u8(), u8(), u8()); if (ok) a else null } }
}

/**
 * Laporan persepsi HP -> robot. Sektor merata dari -halfFov (kanan) ke +halfFov (kiri), jarak mm,
 * 0xFFFF = bebas / tidak diketahui. Robot memakai rintangan terdekat dalam +-30 deg sebagai refleks henti.
 */
data class Perception(
    val ttlMs: Int, val halfFovDeg: Float, val distMm: IntArray,
    val personBearingDeg: Float? = null, val personDistMm: Int? = null, val flags: Int = 0,
) {
    init { require(distMm.size in 1..16) }
    fun enc(): ByteArray {
        val w = Writer().u16(ttlMs).u8(flags).u8(distMm.size).i16(Math.round(halfFovDeg * 10))
        distMm.forEach { w.u16(it.coerceIn(0, 0xFFFF)) }
        w.i16(Math.round((personBearingDeg ?: 0f) * 10)).u16(personDistMm?.coerceIn(0, 0xFFFE) ?: 0xFFFF)
        return w.toByteArray()
    }
    fun sectorCenterDeg(i: Int): Float = -halfFovDeg + (i + 0.5f) * (2 * halfFovDeg / distMm.size)
    fun frontMin(coneDeg: Float = 30f): Int =
        distMm.indices.filter { kotlin.math.abs(sectorCenterDeg(it)) <= coneDeg }.minOfOrNull { distMm[it] } ?: 0xFFFF
}

object Msg {
    fun cmd(id: Int) = Writer().u8(id).toByteArray()
    fun setParam(id: Int, v: Float) = Writer().u8(id).f32(v).toByteArray()
    fun setTrim(bus: Int, slot: Int, v: Int) = Writer().u8(bus).u8(slot).i16(v).toByteArray()
    fun streamCfg(teleHz: Int, servoHz: Int, gaitHz: Int, logOn: Boolean) =
        Writer().u8(teleHz).u8(servoHz).u8(gaitHz).u8(if (logOn) 1 else 0).toByteArray()
    /** vx mm/langkah, yaw derajat/langkah, ttl <= 1000 ms */
    fun gaitIntent(vx: Float, vy: Float, yawDeg: Float, ttlMs: Int) =
        Writer().f32(vx).f32(vy).f32(yawDeg).u16(ttlMs.coerceIn(0, 1000)).toByteArray()
    fun motionPlay(slot: Int, play: Boolean) = Writer().u8(slot).u8(if (play) 1 else 0).toByteArray()
    fun poseTarget(src: Array<IntArray>, mask: Long, ttlMs: Int): ByteArray {
        val w = Writer().u32(mask).u16(ttlMs.coerceIn(0, 1000))
        for (b in 0..1) for (s in 0 until SLOTS) w.i16(src[b][s])
        return w.toByteArray()
    }
    fun ext(topic: Int, data: ByteArray): ByteArray {
        require(data.size <= Lap.MAX_BODY - 2)
        return Writer().u16(topic).bytes(data).toByteArray()
    }
    fun simImu(rollDeg: Float, pitchDeg: Float) = Writer().f32(rollDeg).f32(pitchDeg).u8(1).toByteArray()
}
