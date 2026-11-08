// Penglihatan MediaPipe (GPU/CPU HP): objek (COCO, termasuk orang), isyarat tangan, wajah + pengenalan wajah
// sederhana (embedding umum MobileNetV3, dicocokkan dengan wajah yang didaftarkan).
package id.leumpang.mata

import android.graphics.Bitmap
import android.graphics.RectF
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.facedetector.FaceDetector
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizer
import com.google.mediapipe.tasks.vision.imageembedder.ImageEmbedder
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector
import id.leumpang.bus.Bus
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleInfo
import id.leumpang.perception.Detection
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.floatOrNull
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.io.File

data class FaceHit(val box: RectF, val name: String, val similarity: Float)

class VisionModule(private val ctx: android.content.Context, private val dir: File, private val settings: Settings, private val log: (String) -> Unit) : Module {
    override val info = ModuleInfo(
        "vision", "MediaPipe: objek, isyarat tangan, wajah",
        listOf("vision/objects", "vision/gesture", "vision/faces", "app/models"), emptyList(),
    )
    private var bus: Bus? = null
    private var objects: ObjectDetector? = null
    private var gestures: GestureRecognizer? = null
    private var faces: FaceDetector? = null
    private var embedder: ImageEmbedder? = null
    @Volatile var status = "model belum dimuat"; private set
    @Volatile var lastObjects: List<Detection> = emptyList(); private set
    @Volatile var lastGesture: Pair<String, Float>? = null; private set
    @Volatile var lastFaces: List<FaceHit> = emptyList(); private set
    private var frameNo = 0L
    private var lastTs = -1L
    private val known = mutableMapOf<String, FloatArray>()   // nama -> embedding
    @Volatile private var lastFaceBitmap: Pair<Bitmap, RectF>? = null

    override fun start(bus: Bus) {
        this.bus = bus
        loadFaceDb()
        Thread({
            val errs = Models.ensure(dir) { status = it; bus.publish("app/models", J.obj("status" to it), "vision") }
            fun base(m: Pair<String, String>) = BaseOptions.builder().setModelAssetBuffer(Models.buffer(Models.file(dir, m))).build()
            try {
                if (Models.file(dir, Models.OBJECTS).exists()) objects = ObjectDetector.createFromOptions(ctx,
                    ObjectDetector.ObjectDetectorOptions.builder().setBaseOptions(base(Models.OBJECTS)).setRunningMode(RunningMode.VIDEO)
                        .setMaxResults(8).setScoreThreshold(0.35f).build())
                if (Models.file(dir, Models.GESTURE).exists()) gestures = GestureRecognizer.createFromOptions(ctx,
                    GestureRecognizer.GestureRecognizerOptions.builder().setBaseOptions(base(Models.GESTURE)).setRunningMode(RunningMode.VIDEO)
                        .setNumHands(1).build())
                if (Models.file(dir, Models.FACE).exists()) faces = FaceDetector.createFromOptions(ctx,
                    FaceDetector.FaceDetectorOptions.builder().setBaseOptions(base(Models.FACE)).setRunningMode(RunningMode.VIDEO)
                        .setMinDetectionConfidence(0.6f).build())
                if (Models.file(dir, Models.EMBED).exists()) embedder = ImageEmbedder.createFromOptions(ctx,
                    ImageEmbedder.ImageEmbedderOptions.builder().setBaseOptions(base(Models.EMBED)).setRunningMode(RunningMode.IMAGE)
                        .setL2Normalize(true).build())
                status = if (errs.isEmpty()) "siap" else "sebagian model gagal: ${errs.size}"
            } catch (e: Exception) { status = "MediaPipe: ${e.message}"; log(status) }
        }, "vision-init").start()
    }

    /** Dipanggil dari thread kamera. Mengembalikan deteksi objek (ternormalisasi) untuk modul rintangan. */
    fun process(bmp: Bitmap, tsMs: Long): List<Detection> {
        val ts = if (tsMs <= lastTs) lastTs + 1 else tsMs
        lastTs = ts
        frameNo++
        val img = BitmapImageBuilder(bmp).build()
        val w = bmp.width.toFloat(); val h = bmp.height.toFloat()
        objects?.let { od ->
            val r = od.detectForVideo(img, ts)
            lastObjects = r.detections().map { d ->
                val c = d.categories().first(); val b = d.boundingBox()
                Detection(c.categoryName(), c.score(), b.left / w, b.top / h, b.right / w, b.bottom / h)
            }
            bus?.publish("vision/objects", J.of(lastObjects.map { mapOf("label" to it.label, "score" to it.score,
                "box" to listOf(it.left, it.top, it.right, it.bottom)) }), "vision")
        }
        if (settings.gestures && frameNo % 2 == 0L) gestures?.let { gr ->
            val r = gr.recognizeForVideo(img, ts)
            val g = r.gestures().firstOrNull()?.firstOrNull()
            lastGesture = g?.let { it.categoryName() to it.score() }
            if (g != null) bus?.publish("vision/gesture", J.obj("name" to g.categoryName(), "score" to g.score(),
                "hand" to (r.handedness().firstOrNull()?.firstOrNull()?.categoryName() ?: "")), "vision", retain = false)
        }
        if (settings.faces && frameNo % 3 == 0L) faces?.let { fd ->
            val r = fd.detectForVideo(img, ts)
            val hits = r.detections().map { d ->
                val b = d.boundingBox()
                val nb = RectF(b.left / w, b.top / h, b.right / w, b.bottom / h)
                val (name, sim) = identify(bmp, b)
                FaceHit(nb, name, sim)
            }
            lastFaces = hits
            r.detections().maxByOrNull { it.boundingBox().width() }?.let { lastFaceBitmap = bmp to RectF(it.boundingBox()) }
            if (hits.isNotEmpty()) bus?.publish("vision/faces", J.of(hits.map { mapOf("name" to it.name, "similarity" to it.similarity,
                "box" to listOf(it.box.left, it.box.top, it.box.right, it.box.bottom)) }), "vision")
        }
        return lastObjects
    }

    private fun embed(bmp: Bitmap, box: RectF): FloatArray? {
        val e = embedder ?: return null
        val pad = box.width() * 0.15f
        val l = (box.left - pad).toInt().coerceIn(0, bmp.width - 1); val t = (box.top - pad).toInt().coerceIn(0, bmp.height - 1)
        val r = (box.right + pad).toInt().coerceIn(l + 1, bmp.width); val b = (box.bottom + pad).toInt().coerceIn(t + 1, bmp.height)
        val crop = Bitmap.createBitmap(bmp, l, t, r - l, b - t)
        return e.embed(BitmapImageBuilder(crop).build()).embeddingResult().embeddings().firstOrNull()?.floatEmbedding()
    }

    private fun identify(bmp: Bitmap, box: RectF): Pair<String, Float> {
        if (known.isEmpty()) return "" to 0f
        val v = embed(bmp, box) ?: return "" to 0f
        var best = ""; var bs = 0f
        for ((n, k) in known) {
            val s = cosine(v, k)
            if (s > bs) { bs = s; best = n }
        }
        return (if (bs >= 0.82f) best else "") to bs
    }

    private fun cosine(a: FloatArray, b: FloatArray): Float {
        if (a.size != b.size) return 0f
        var d = 0f; var na = 0f; var nb = 0f
        for (i in a.indices) { d += a[i] * b[i]; na += a[i] * a[i]; nb += b[i] * b[i] }
        return if (na > 0 && nb > 0) d / kotlin.math.sqrt(na * nb) else 0f
    }

    /** Daftarkan wajah terbesar yang terakhir terlihat dengan nama ini. */
    fun enrollFace(name: String): String {
        val (bmp, box) = lastFaceBitmap ?: return "tidak ada wajah terlihat"
        val v = embed(bmp, box) ?: return "model embedding belum siap"
        known[name] = known[name]?.let { old -> FloatArray(v.size) { (old[it] + v[it]) / 2 } } ?: v
        saveFaceDb()
        return "wajah '$name' didaftarkan (${known.size} orang)"
    }
    fun forgetFaces() { known.clear(); saveFaceDb() }
    fun knownNames() = known.keys.sorted()

    private fun saveFaceDb() {
        settings.faceDb = JsonObject(known.mapValues { (_, v) -> JsonArray(v.map { JsonPrimitive(it) }) }).toString()
    }
    private fun loadFaceDb() {
        try {
            Json.parseToJsonElement(settings.faceDb).jsonObject.forEach { (k, v) ->
                known[k] = v.jsonArray.map { it.jsonPrimitive.floatOrNull ?: 0f }.toFloatArray()
            }
        } catch (_: Exception) {}
    }

    override fun stop() { objects?.close(); gestures?.close(); faces?.close(); embedder?.close() }
}
