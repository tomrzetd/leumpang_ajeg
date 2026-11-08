// Model AI (MediaPipe) diunduh sekali dari penyimpanan resmi Google ke memori internal aplikasi.
package id.leumpang.mata

import java.io.File
import java.io.FileInputStream
import java.net.HttpURLConnection
import java.net.URL
import java.nio.ByteBuffer
import java.nio.channels.FileChannel

object Models {
    private const val BASE = "https://storage.googleapis.com/mediapipe-models"
    val OBJECTS = "objects.tflite" to "$BASE/object_detector/efficientdet_lite0/int8/1/efficientdet_lite0.tflite"
    val GESTURE = "gesture.task" to "$BASE/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task"
    val FACE = "face.tflite" to "$BASE/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite"
    val EMBED = "embedder.tflite" to "$BASE/image_embedder/mobilenet_v3_small/float32/1/mobilenet_v3_small.tflite"
    val ALL = listOf(OBJECTS, GESTURE, FACE, EMBED)

    fun file(dir: File, m: Pair<String, String>) = File(dir, "models/${m.first}")

    /** Unduh yang belum ada. Mengembalikan daftar galat (kosong = semua siap). */
    fun ensure(dir: File, progress: (String) -> Unit): List<String> {
        val errs = mutableListOf<String>()
        File(dir, "models").mkdirs()
        for (m in ALL) {
            val f = file(dir, m)
            if (f.exists() && f.length() > 1000) continue
            progress("mengunduh ${m.first} ...")
            try {
                val c = URL(m.second).openConnection() as HttpURLConnection
                c.connectTimeout = 15000; c.readTimeout = 30000
                val tmp = File(f.path + ".part")
                c.inputStream.use { i -> tmp.outputStream().use { o -> i.copyTo(o) } }
                if (!tmp.renameTo(f)) errs += "${m.first}: gagal menyimpan"
            } catch (e: Exception) { errs += "${m.first}: ${e.message}" }
        }
        progress(if (errs.isEmpty()) "model siap" else "model gagal: ${errs.joinToString()}")
        return errs
    }

    fun buffer(f: File): ByteBuffer = FileInputStream(f).channel.use { it.map(FileChannel.MapMode.READ_ONLY, 0, it.size()) }
}
