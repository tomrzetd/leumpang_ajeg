// Geometri kamera kepala -> lantai. Kamera pinhole; frame kamera x kanan, y bawah, z depan.
// Frame robot: X depan, Y kiri, Z atas. Sudut belok (bearing) + = ke kiri, sama dengan yaw robot.
package id.leumpang.perception

import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.tan

data class CameraModel(val width: Int, val height: Int, val hFovDeg: Float, val vFovDeg: Float) {
    val fx = (width / 2f) / tan(Math.toRadians(hFovDeg / 2.0)).toFloat()
    val fy = (height / 2f) / tan(Math.toRadians(vFovDeg / 2.0)).toFloat()
    val cx = (width - 1) / 2f
    val cy = (height - 1) / 2f
    /** bearing (derajat, + kiri) untuk kolom piksel */
    fun bearingDeg(col: Float): Float = Math.toDegrees(atan(((cx - col) / fx).toDouble())).toFloat()
    fun scaled(w: Int, h: Int) = CameraModel(w, h, hFovDeg, vFovDeg)
}

/**
 * Pose kamera relatif lantai.
 * heightM: tinggi lensa dari lantai. pitchDownDeg: kemiringan sumbu optik ke bawah (+).
 * yawDeg: yaw kepala relatif badan (+ kiri).
 */
data class CameraPose(val heightM: Float, val pitchDownDeg: Float, val yawDeg: Float = 0f) {
    companion object {
        /**
         * Dari telemetry robot: tinggi kamera = autoH + offset (model gen_model: pelvis 81,5 mm di atas
         * sumbu kaki + kepala 140 mm -> 221,5 mm), pitch = sudut pasang + pitch badan (IMU), yaw = servo kepala.
         */
        fun fromRobot(autoHmm: Float, bodyPitchDeg: Float, headYawSrc: Int, mountPitchDeg: Float,
                      offsetMm: Float = 221.5f, pitchSign: Float = 1f) = CameraPose(
            (autoHmm + offsetMm) / 1000f, mountPitchDeg + pitchSign * bodyPitchDeg,
            Math.toDegrees(headYawSrc / 1718.9).toFloat())
    }
}

data class GroundPoint(val xM: Float, val yM: Float) {
    val distM get() = hypot(xM, yM)
    val bearingDeg get() = Math.toDegrees(atan2(yM, xM).toDouble()).toFloat()
}

object Ground {
    /** Titik lantai untuk piksel (u, v); null bila sinar tidak mengenai lantai (di atas horizon). */
    fun project(cam: CameraModel, pose: CameraPose, u: Float, v: Float): GroundPoint? {
        val x = (u - cam.cx) / cam.fx; val y = (v - cam.cy) / cam.fy
        val p = Math.toRadians(pose.pitchDownDeg.toDouble())
        val fwd = cos(p) - y * sin(p)
        val down = sin(p) + y * cos(p)
        if (down <= 1e-4) return null
        val t = pose.heightM / down
        val gx = (t * fwd).toFloat(); val gy = (-t * x).toFloat()
        if (gx <= 0f) return null
        val yaw = Math.toRadians(pose.yawDeg.toDouble())
        return GroundPoint((gx * cos(yaw) - gy * sin(yaw)).toFloat(), (gx * sin(yaw) + gy * cos(yaw)).toFloat())
    }

    /** Baris horizon (piksel) - di atasnya tidak ada lantai. */
    fun horizonRow(cam: CameraModel, pose: CameraPose): Float =
        cam.cy - cam.fy * tan(Math.toRadians(pose.pitchDownDeg.toDouble())).toFloat()
}
