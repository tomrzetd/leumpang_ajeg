// Mesin "mata": merakit Leumpang Bus + modul. Hidup selama aplikasi (Application), bukan per layar.
package id.leumpang.mata

import android.content.Context
import android.graphics.Bitmap
import androidx.lifecycle.LifecycleOwner
import id.leumpang.bus.Bus
import id.leumpang.bus.ModuleHost
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class Engine(ctx: Context) {
    val settings = Settings(ctx)
    val bus = Bus()
    val host = ModuleHost(bus)
    private val logLines = ArrayDeque<String>()
    private val fmt = SimpleDateFormat("HH:mm:ss", Locale.US)

    fun log(s: String) = synchronized(logLines) {
        logLines.addLast("${fmt.format(Date())}  $s")
        while (logLines.size > 200) logLines.removeFirst()
    }
    fun logTail(n: Int = 60): List<String> = synchronized(logLines) { logLines.takeLast(n) }

    val robot = RobotLinkModule(ctx, ::log)
    val vision = VisionModule(ctx, ctx.filesDir, settings, ::log)
    val obstacles = ObstacleModule(robot, settings)
    val behavior = BehaviorModule(robot, obstacles, vision, settings, ::log)
    val speech = SpeechModule(ctx, settings, ::log)
    val busServer = BusServer(host, settings, ::log)
    val camera = CameraSource(ctx, ::log)

    @Volatile var lastFrame: Bitmap? = null; private set
    @Volatile var frameMs = 0f; private set

    init {
        host.add(robot).add(vision).add(obstacles).add(behavior).add(speech).add(busServer)
        for (id in listOf("robot", "vision", "obstacles", "behavior", "speech")) host.start(id)
        if (settings.busServer) host.start("bus-ws")
    }

    fun startCamera(owner: LifecycleOwner) {
        camera.start(owner, settings.cameraId, settings.extraRotation) { bmp, ts ->
            val t0 = System.nanoTime()
            val dets = vision.process(bmp, ts)
            val cam = camera.active
            obstacles.process(bmp, dets, cam?.hFovDeg ?: 66f, cam?.vFovDeg ?: 51f)
            lastFrame = bmp
            frameMs = frameMs * 0.9f + 0.1f * ((System.nanoTime() - t0) / 1e6f)
        }
    }

    fun setBusServer(on: Boolean) { settings.busServer = on; if (on) host.start("bus-ws") else host.stop("bus-ws") }
}
