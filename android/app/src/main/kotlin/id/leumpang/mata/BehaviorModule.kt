// Perilaku: satu-satunya modul yang menggerakkan robot. Menerima permintaan dari suara, isyarat tangan,
// UI, atau klien luar (behavior/request), lalu menjalankan arbiter perception.Behavior pada 5 Hz.
package id.leumpang.mata

import id.leumpang.bus.Bus
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleInfo
import id.leumpang.bus.Subscription
import id.leumpang.perception.Action
import id.leumpang.perception.Activity
import id.leumpang.perception.Behavior
import id.leumpang.perception.Describe
import id.leumpang.perception.Gestures
import id.leumpang.perception.Request
import id.leumpang.perception.RobotView
import kotlinx.serialization.json.JsonElement
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

object RequestJson {
    fun encode(r: Request): JsonElement = when (r) {
        Request.Stop -> J.obj("action" to "stop")
        Request.Wander -> J.obj("action" to "wander")
        Request.Follow -> J.obj("action" to "follow")
        Request.Wave -> J.obj("action" to "wave")
        Request.Home -> J.obj("action" to "home")
        Request.Balance -> J.obj("action" to "balance")
        Request.Describe -> J.obj("action" to "describe")
        is Request.Walk -> J.obj("action" to "walk", "vx" to r.vxMm, "seconds" to r.seconds)
        is Request.Turn -> J.obj("action" to "turn", "yaw" to r.yawDeg, "seconds" to r.seconds)
        is Request.Say -> J.obj("action" to "say", "text" to r.text)
    }
    fun decode(e: JsonElement): Request? = when (J.str(e, "action")) {
        "stop" -> Request.Stop; "wander" -> Request.Wander; "follow" -> Request.Follow; "wave" -> Request.Wave
        "home" -> Request.Home; "balance" -> Request.Balance; "describe" -> Request.Describe
        "walk" -> Request.Walk((J.num(e, "vx") ?: 6.0).toFloat().coerceIn(-10f, 10f), (J.num(e, "seconds") ?: 3.0).toFloat().coerceIn(0.2f, 30f))
        "turn" -> Request.Turn((J.num(e, "yaw") ?: 4.0).toFloat().coerceIn(-5f, 5f), (J.num(e, "seconds") ?: 2.0).toFloat().coerceIn(0.2f, 30f))
        "say" -> J.str(e, "text")?.take(200)?.let { Request.Say(it) }
        else -> null
    }
}

class BehaviorModule(
    private val robot: RobotLinkModule, private val obstacles: ObstacleModule, private val vision: VisionModule,
    private val settings: Settings, private val log: (String) -> Unit,
) : Module {
    override val info = ModuleInfo(
        "behavior", "Arbiter perilaku: hindari rintangan > perintah > ikuti orang > jelajah",
        listOf("behavior/state", "speech/say"), listOf("behavior/request", "vision/gesture"),
    )
    val behavior = Behavior()
    private val gestures = Gestures()
    private var bus: Bus? = null
    private val subs = mutableListOf<Subscription>()
    private val exec = Executors.newSingleThreadScheduledExecutor()
    @Volatile var lastReason = ""; private set

    override fun start(bus: Bus) {
        this.bus = bus
        subs += bus.subscribe("behavior/request", "behavior") { m ->
            val r = RequestJson.decode(m.data) ?: return@subscribe
            log("permintaan dari ${m.source.ifEmpty { "?" }}: ${J.str(m.data, "action")}")
            exec.execute { run(behavior.request(r, System.currentTimeMillis(), view())) }
        }
        subs += bus.subscribe("vision/gesture", "behavior") { m ->
            if (!settings.gestures) return@subscribe
            val r = gestures.update(J.str(m.data, "name"), (J.num(m.data, "score") ?: 0.0).toFloat(), System.currentTimeMillis())
            if (r != null) bus.publish("behavior/request", RequestJson.encode(r), "gesture", retain = false)
        }
        exec.scheduleAtFixedRate({
            try {
                behavior.planner.stopMm = settings.obstacleStopMm
                behavior.waveSlot = settings.waveSlot
                run(behavior.tick(System.currentTimeMillis(), view(), obstacles.lastMap,
                    if (behavior.activity == Activity.FOLLOW) obstacles.lastPerson else null))
            } catch (e: Exception) { log("perilaku: ${e.message}") }
        }, 200, 200, TimeUnit.MILLISECONDS)
    }

    fun submit(r: Request) { bus?.publish("behavior/request", RequestJson.encode(r), "ui", retain = false) }

    private fun view(): RobotView {
        val t = robot.client.telemetry
        return RobotView(robot.alive, t?.mode ?: 0, t?.obstacleBlock ?: false)
    }

    private fun run(actions: List<Action>) {
        for (a in actions) when (a) {
            is Action.Gait -> robot.gait(a.vxMm, a.yawDeg, a.ttlMs)
            is Action.Cmd -> robot.cmd(a.id)
            is Action.Motion -> robot.motion(a.slot)
            is Action.Say -> bus?.publish("speech/say", J.obj("text" to a.text), "behavior", retain = false)
            is Action.State -> {
                lastReason = a.reason
                bus?.publish("behavior/state", J.obj("activity" to a.activity.name.lowercase(), "reason" to a.reason), "behavior")
            }
            Action.DescribeScene -> bus?.publish("speech/say", J.obj("text" to
                Describe.sentence(vision.lastObjects.map { it.label }, vision.lastFaces.map { it.name })), "behavior", retain = false)
        }
    }

    override fun stop() { subs.forEach { it.cancel() }; exec.shutdownNow() }
}
