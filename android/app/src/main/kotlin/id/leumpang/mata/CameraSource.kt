// Sumber gambar: CameraX dengan id Camera2 pilihan.
//  * Kamera utama HP yang dipindah ke kepala (kabel flex + konektor) = kamera BELAKANG id "0" - tetap kamera internal.
//  * Kamera USB-UVC di kepala: muncul sebagai LENS_FACING_EXTERNAL bila HAL HP mendukung kamera eksternal
//    (Android 14+ di sebagian HP). Bila tidak muncul di daftar, HP tidak mengekspos UVC ke Camera2 - lihat android/README.md.
package id.leumpang.mata

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.util.Size
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import java.util.concurrent.Executors
import kotlin.math.atan

data class CameraEntry(val id: String, val facing: String, val hFovDeg: Float, val vFovDeg: Float) {
    val label get() = "$id · $facing · ${"%.0f".format(hFovDeg)}°"
}

class CameraSource(private val ctx: Context, private val log: (String) -> Unit) {
    private val exec = Executors.newSingleThreadExecutor()
    private var provider: ProcessCameraProvider? = null
    @Volatile var active: CameraEntry? = null; private set
    @Volatile var fps = 0f; private set
    private var lastNs = 0L

    fun list(): List<CameraEntry> {
        val cm = ctx.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        return cm.cameraIdList.mapNotNull { id ->
            try {
                val c = cm.getCameraCharacteristics(id)
                val facing = when (c.get(CameraCharacteristics.LENS_FACING)) {
                    CameraCharacteristics.LENS_FACING_BACK -> "belakang (utama)"
                    CameraCharacteristics.LENS_FACING_FRONT -> "depan"
                    CameraCharacteristics.LENS_FACING_EXTERNAL -> "USB/UVC eksternal"
                    else -> "?"
                }
                val size = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
                val focal = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull()
                val h = if (size != null && focal != null && focal > 0) Math.toDegrees(2 * atan(size.width / (2.0 * focal))).toFloat() else 66f
                val v = if (size != null && focal != null && focal > 0) Math.toDegrees(2 * atan(size.height / (2.0 * focal))).toFloat() else 51f
                CameraEntry(id, facing, h, v)
            } catch (e: Exception) { null }
        }
    }

    /** Mulai analisis gambar. onFrame dipanggil di thread analisis dengan bitmap tegak (sudah diputar). */
    @SuppressLint("UnsafeOptInUsageError")
    @androidx.annotation.OptIn(ExperimentalCamera2Interop::class)
    fun start(owner: LifecycleOwner, cameraId: String, extraRotation: Int, onFrame: (Bitmap, Long) -> Unit) {
        val all = list()
        val entry = all.find { it.id == cameraId } ?: all.find { it.facing.startsWith("belakang") } ?: all.firstOrNull()
        if (entry == null) { log("kamera: tidak ada"); return }
        ProcessCameraProvider.getInstance(ctx).addListener({
            val p = ProcessCameraProvider.getInstance(ctx).get(); provider = p
            val analysis = ImageAnalysis.Builder()
                .setResolutionSelector(ResolutionSelector.Builder().setResolutionStrategy(
                    ResolutionStrategy(Size(640, 480), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER)).build())
                .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                .build()
            analysis.setAnalyzer(exec) { img ->
                try {
                    val rot = (img.imageInfo.rotationDegrees + extraRotation) % 360
                    var bmp = img.toBitmap()
                    if (rot != 0) bmp = Bitmap.createBitmap(bmp, 0, 0, bmp.width, bmp.height, Matrix().apply { postRotate(rot.toFloat()) }, true)
                    val now = System.nanoTime()
                    if (lastNs != 0L) fps = fps * 0.9f + 0.1f * (1e9f / (now - lastNs))
                    lastNs = now
                    onFrame(bmp, img.imageInfo.timestamp / 1_000_000)
                } catch (e: Exception) { log("analisis: ${e.message}") }
                finally { img.close() }
            }
            val selector = CameraSelector.Builder().addCameraFilter { infos ->
                infos.filter { Camera2CameraInfo.from(it).cameraId == entry.id }
            }.build()
            try {
                p.unbindAll()
                p.bindToLifecycle(owner, selector, analysis)
                active = entry
                log("kamera: ${entry.label}")
            } catch (e: Exception) {
                active = null
                log("kamera ${entry.id} tidak bisa dibuka lewat CameraX: ${e.message}")
            }
        }, ContextCompat.getMainExecutor(ctx))
    }

    fun stop() { provider?.unbindAll(); active = null }
}
