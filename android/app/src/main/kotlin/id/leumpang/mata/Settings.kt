package id.leumpang.mata

import android.content.Context
import android.content.SharedPreferences

/** Pengaturan tersimpan (SharedPreferences). */
class Settings(ctx: Context) {
    private val p: SharedPreferences = ctx.getSharedPreferences("mata", Context.MODE_PRIVATE)
    private fun f(k: String, d: Float) = p.getFloat(k, d)
    private fun i(k: String, d: Int) = p.getInt(k, d)
    private fun b(k: String, d: Boolean) = p.getBoolean(k, d)
    private fun s(k: String, d: String) = p.getString(k, d) ?: d
    private fun put(block: SharedPreferences.Editor.() -> Unit) = p.edit().apply(block).apply()

    /** id kamera Camera2 ("" = kamera belakang utama). Kamera utama HP yang dipindah ke kepala tetap kamera belakang. */
    var cameraId: String get() = s("cameraId", ""); set(v) = put { putString("cameraId", v) }
    /** rotasi tambahan (0/90/180/270) bila modul kamera dipasang miring di kepala */
    var extraRotation: Int get() = i("rot", 0); set(v) = put { putInt("rot", v) }
    /** kemiringan kamera ke bawah relatif badan (derajat) */
    var mountPitchDeg: Float get() = f("pitch", 20f); set(v) = put { putFloat("pitch", v) }
    /** tinggi lensa di atas sumbu kaki robot (mm); model gen_model: 221,5 */
    var camOffsetMm: Float get() = f("camOfs", 221.5f); set(v) = put { putFloat("camOfs", v) }
    /** FOV horizontal (0 = dari Camera2) */
    var hFovOverride: Float get() = f("hfov", 0f); set(v) = put { putFloat("hfov", v) }
    var tcpHost: String get() = s("tcpHost", "192.168.4.1"); set(v) = put { putString("tcpHost", v) }

    var gestures: Boolean get() = b("gest", true); set(v) = put { putBoolean("gest", v) }
    var faces: Boolean get() = b("faces", true); set(v) = put { putBoolean("faces", v) }
    var speech: Boolean get() = b("speech", false); set(v) = put { putBoolean("speech", v) }
    var tts: Boolean get() = b("tts", true); set(v) = put { putBoolean("tts", v) }
    var requireWake: Boolean get() = b("wake", true); set(v) = put { putBoolean("wake", v) }
    var obstacleStopMm: Int get() = i("stopMm", 350); set(v) = put { putInt("stopMm", v) }
    var waveSlot: Int get() = i("wave", 0); set(v) = put { putInt("wave", v) }

    var busServer: Boolean get() = b("bus", false); set(v) = put { putBoolean("bus", v) }
    var busToken: String get() = s("busToken", ""); set(v) = put { putString("busToken", v) }

    var faceDb: String get() = s("faceDb", "{}"); set(v) = put { putString("faceDb", v) }
}
