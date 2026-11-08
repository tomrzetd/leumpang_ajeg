// Perencana langkah (VFH sederhana) + pengikut orang. Keluaran dalam satuan GAIT_INTENT:
// vx mm/langkah (maks gaitStepMax firmware, default 10), yaw derajat/langkah (maks 5).
package id.leumpang.perception

import kotlin.math.abs
import kotlin.math.min
import kotlin.math.sign

data class Step(val vxMm: Float, val yawDeg: Float, val reason: String)

class Planner(
    var stopMm: Int = 350,          // lebih dekat -> tidak maju (firmware sendiri berhenti di 250)
    var slowMm: Int = 900,          // mulai melambat
    var clearMm: Int = 600,         // sektor dianggap lapang bila lebih jauh
    var maxVx: Float = 8f,
    var maxYaw: Float = 4f,
) {
    /** goalBearingDeg: arah tujuan (+ kiri); 0 = lurus (jelajah). */
    fun plan(map: ObstacleMap, goalBearingDeg: Float = 0f, wantVx: Float = maxVx): Step {
        val front = map.frontMin(25f)
        // pilih sektor lapang dengan skor terbaik: dekat tujuan, jauh dari rintangan
        var best = -1; var bestScore = Float.NEGATIVE_INFINITY
        for (i in 0 until map.n) {
            val d = min(map.distMm[i], 3000)
            if (d < clearMm) continue
            val neigh = listOf(i - 1, i + 1).filter { it in 0 until map.n }.minOfOrNull { min(map.distMm[it], 3000) } ?: d
            val score = d / 1000f + 0.5f * neigh / 1000f - abs(map.centerDeg(i) - goalBearingDeg) / 15f
            if (score > bestScore) { bestScore = score; best = i }
        }
        if (best < 0) return Step(0f, maxYaw * (if (goalBearingDeg >= 0) 1f else -1f), "terkepung: berputar mencari jalan")
        val heading = map.centerDeg(best)
        val yaw = (heading * 0.25f).coerceIn(-maxYaw, maxYaw)
        val vx = when {
            front < stopMm -> 0f
            front < slowMm -> wantVx * (front - stopMm).toFloat() / (slowMm - stopMm)
            else -> wantVx
        }.let { if (abs(heading) > 25f) min(it, wantVx * 0.4f) else it }
        val why = when {
            front < stopMm -> "rintangan ${front} mm: belok ke ${"%.0f".format(heading)} deg"
            front < slowMm -> "melambat (rintangan ${front} mm)"
            abs(heading - goalBearingDeg) > 8f -> "menghindar ke ${"%.0f".format(heading)} deg"
            else -> "jalan"
        }
        return Step(vx.coerceIn(0f, maxVx), yaw, why)
    }
}

/** Target orang terpilih. */
data class PersonTarget(val bearingDeg: Float, val distMm: Int, val score: Float, val box: Detection)

class PersonTracker(var personHeightM: Float = 1.65f, var lostAfterMs: Long = 1500) {
    private var last: PersonTarget? = null
    private var lastSeenMs = 0L

    /** Pilih orang: dekat dengan target sebelumnya, kalau tidak yang terbesar. Jarak dari kaki di lantai atau tinggi kotak. */
    fun update(dets: List<Detection>, cam: CameraModel, pose: CameraPose, nowMs: Long): PersonTarget? {
        val people = dets.filter { it.label == "person" && it.score >= 0.4f }
        val cand = people.map { d ->
            val bearing = cam.bearingDeg(d.cx * cam.width) + pose.yawDeg
            val byHeight = (personHeightM * cam.fy / (d.h * cam.height).coerceAtLeast(1f) * 1000).toInt()
            val feetVisible = d.bottom < 0.97f
            val byFeet = if (feetVisible) Ground.project(cam, pose, d.cx * cam.width, d.bottom * cam.height)?.let { (it.distM * 1000).toInt() } else null
            PersonTarget(bearing, byFeet ?: byHeight, d.score, d)
        }
        val prev = last
        val pick = if (prev != null && nowMs - lastSeenMs < lostAfterMs)
            cand.minByOrNull { abs(it.bearingDeg - prev.bearingDeg) + abs(it.distMm - prev.distMm) / 200f }
        else cand.maxByOrNull { it.box.h }
        if (pick != null) {
            last = if (prev == null || nowMs - lastSeenMs >= lostAfterMs) pick else pick.copy(
                bearingDeg = prev.bearingDeg * 0.4f + pick.bearingDeg * 0.6f,
                distMm = (prev.distMm * 0.5f + pick.distMm * 0.5f).toInt())
            lastSeenMs = nowMs
        }
        return if (nowMs - lastSeenMs < lostAfterMs) last else null
    }
    fun reset() { last = null; lastSeenMs = 0 }
}

class Follower(var keepMm: Int = 1000, var deadbandMm: Int = 150, val planner: Planner = Planner()) {
    fun step(target: PersonTarget?, map: ObstacleMap): Step {
        if (target == null) return Step(0f, 0f, "orang hilang")
        val err = target.distMm - keepMm
        val want = if (err > deadbandMm) min(planner.maxVx, err / 100f) else 0f
        // orang sendiri terlihat sebagai "rintangan" di sektornya: abaikan sektor itu bila jaraknya ~ jarak orang
        val d = map.distMm.copyOf()
        map.sectorOf(target.bearingDeg)?.let { s -> if (abs(d[s] - target.distMm) < 400) d[s] = ObstacleMap.FREE }
        val s = planner.plan(ObstacleMap(map.halfFovDeg, d), target.bearingDeg, want)
        val yaw = if (want == 0f) (target.bearingDeg * 0.2f).coerceIn(-planner.maxYaw, planner.maxYaw)
                  .let { if (abs(target.bearingDeg) < 6f) 0f else it } else s.yawDeg
        return Step(s.vxMm, yaw, if (want == 0f) "menjaga jarak ${target.distMm} mm" else "mengikuti (${target.distMm} mm, ${"%.0f".format(target.bearingDeg)} deg)")
    }
}

internal fun Float.sgn() = sign(this)
