// Rintangan & orang dari kamera kepala -> Leumpang Bus + PERCEPTION ke robot (refleks henti firmware).
package id.leumpang.mata

import android.graphics.Bitmap
import id.leumpang.bus.Bus
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleInfo
import id.leumpang.lap.Perception
import id.leumpang.perception.CameraModel
import id.leumpang.perception.CameraPose
import id.leumpang.perception.Detection
import id.leumpang.perception.FreeSpace
import id.leumpang.perception.Ground
import id.leumpang.perception.ObstacleMap
import id.leumpang.perception.PersonTarget
import id.leumpang.perception.PersonTracker
import id.leumpang.perception.RgbImage

class ObstacleModule(private val robot: RobotLinkModule, private val settings: Settings) : Module {
    override val info = ModuleInfo(
        "obstacles", "Ruang bebas lantai + deteksi objek -> peta rintangan polar; pelacak orang",
        listOf("perception/obstacles", "perception/person"), listOf("robot/telemetry"),
    )
    private var bus: Bus? = null
    private val free = FreeSpace()
    private val tracker = PersonTracker()
    @Volatile var lastMap: ObstacleMap? = null; private set
    @Volatile var lastBoundary: FloatArray? = null; private set      // baris batas ternormalisasi per kolom (-1 bebas)
    @Volatile var lastPerson: PersonTarget? = null; private set
    @Volatile var horizon = 0f; private set
    private var lastSend = 0L

    override fun start(bus: Bus) { this.bus = bus }

    fun pose(): CameraPose {
        val t = robot.client.telemetry; val g = robot.client.gait
        return CameraPose.fromRobot(t?.autoH?.takeIf { it > 100f } ?: 185f, t?.pitchDeg ?: 0f, g?.headYawSrc ?: 0,
            settings.mountPitchDeg, settings.camOffsetMm)
    }

    fun process(bmp: Bitmap, dets: List<Detection>, hFov: Float, vFov: Float) {
        val small = Bitmap.createScaledBitmap(bmp, 160, 120, true)
        val px = IntArray(160 * 120); small.getPixels(px, 0, 160, 0, 0, 160, 120)
        val cam = CameraModel(160, 120, if (settings.hFovOverride > 0) settings.hFovOverride else hFov, vFov)
        val pose = pose()
        val img = RgbImage(160, 120, px)
        horizon = Ground.horizonRow(cam, pose)
        free.learnFloor(img)
        val b = free.boundaries(img, horizon.toInt().coerceAtLeast(0))
        lastBoundary = FloatArray(b.size) { if (b[it] < 0) -1f else b[it] / 120f }
        val map = ObstacleMap.build(cam, pose, b, dets.filter { it.score >= 0.45f })
        lastMap = map
        val person = tracker.update(dets, cam, pose, System.currentTimeMillis())
        lastPerson = person
        val now = System.currentTimeMillis()
        if (now - lastSend >= 100) {
            lastSend = now
            robot.perception(Perception(500, map.halfFovDeg, map.distMm, person?.bearingDeg, person?.distMm))
            bus?.publish("perception/obstacles", J.obj("halfFovDeg" to map.halfFovDeg, "distMm" to map.distMm,
                "frontMm" to map.frontMin(), "cameraHeightMm" to (pose.heightM * 1000).toInt()), "obstacles")
            bus?.publish("perception/person", if (person == null) J.obj("visible" to false) else
                J.obj("visible" to true, "bearingDeg" to person.bearingDeg, "distMm" to person.distMm, "score" to person.score), "obstacles")
        }
    }
}
