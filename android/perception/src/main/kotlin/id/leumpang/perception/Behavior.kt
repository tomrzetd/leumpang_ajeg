// Perintah (suara / isyarat tangan / modul luar) dan arbiter perilaku robot.
// Prioritas: keselamatan (jatuh, rintangan) > perintah eksplisit > mengikuti orang > jelajah > diam.
// Firmware tetap otoritas akhir: refleks rintangan, batas sendi, deteksi jatuh ada di robot.
package id.leumpang.perception

sealed interface Request {
    data object Stop : Request
    data object Wander : Request
    data object Follow : Request
    data object Wave : Request
    data object Home : Request
    data object Balance : Request
    data class Walk(val vxMm: Float, val seconds: Float = 3f) : Request
    data class Turn(val yawDeg: Float, val seconds: Float = 2f) : Request
    data class Say(val text: String) : Request
    data object Describe : Request
}

sealed interface Action {
    data class Gait(val vxMm: Float, val yawDeg: Float, val ttlMs: Int) : Action
    data class Cmd(val id: Int) : Action
    data class Motion(val slot: Int) : Action
    data class Say(val text: String) : Action
    data class State(val activity: Activity, val reason: String) : Action
    data object DescribeScene : Action
}

enum class Activity { IDLE, WANDER, FOLLOW, MANUAL, STOPPED }

data class RobotView(val alive: Boolean, val mode: Int, val obstacleBlock: Boolean = false)

object Speech {
    private val wake = listOf("leumpang", "robot", "hai robot", "halo robot")
    private fun norm(s: String) = s.lowercase().replace(Regex("[^a-z0-9 ]"), " ").replace(Regex("\\s+"), " ").trim()

    /** Teks ucapan (Indonesia, sebagian Sunda & Inggris) -> permintaan. requireWake: harus diawali kata bangun. */
    fun parse(text: String, requireWake: Boolean = false): Request? {
        var t = norm(text)
        val w = wake.firstOrNull { t.startsWith(it) }
        if (requireWake && w == null) return null
        if (w != null) t = t.removePrefix(w).trim()
        fun has(vararg k: String) = k.any { Regex("\\b${Regex.escape(it)}\\b").containsMatchIn(t) }
        return when {
            has("berhenti", "stop", "diam", "eureun", "cicing", "jangan bergerak") -> Request.Stop
            has("ikuti saya", "ikuti aku", "ikut aku", "ikut saya", "ikuti", "tuturkeun", "follow me") -> Request.Follow
            has("lambaikan", "lambai", "dadah", "da dah", "wave") -> Request.Wave
            has("siapa kamu", "siapa namamu", "nama kamu", "saha anjeun") -> Request.Say("Aku Leumpang Ajeg, robot yang berjalan dengan mantap.")
            has("apa yang kamu lihat", "lihat apa", "kamu lihat apa", "naon nu katingali") -> Request.Describe
            has("belok kiri", "ke kiri", "kiri") -> Request.Turn(4f)
            has("belok kanan", "ke kanan", "kanan") -> Request.Turn(-4f)
            has("mundur", "ka tukang") -> Request.Walk(-5f)
            has("jelajah", "jalan jalan", "keliling", "explore") -> Request.Wander
            has("jalan", "maju", "leumpang", "walk") -> Request.Walk(7f)
            has("seimbang", "siap jalan", "balance") -> Request.Balance
            has("berdiri", "pulang", "posisi awal", "home", "nangtung") -> Request.Home
            else -> null
        }
    }
}

/** Nama kategori MediaPipe GestureRecognizer -> permintaan, dengan debounce & jeda. */
class Gestures(var holdFrames: Int = 4, var cooldownMs: Long = 2500) {
    private var cur: String? = null; private var count = 0
    private val lastFired = mutableMapOf<String, Long>()
    fun map(name: String): Request? = when (name) {
        "Open_Palm" -> Request.Stop
        "Closed_Fist" -> Request.Stop
        "Thumb_Up" -> Request.Follow
        "Pointing_Up" -> Request.Wander
        "Victory" -> Request.Wave
        "Thumb_Down" -> Request.Home
        "ILoveYou" -> Request.Say("Aku juga sayang kamu!")
        else -> null
    }
    fun update(name: String?, score: Float, nowMs: Long): Request? {
        if (name == null || name == "None" || score < 0.6f) { cur = null; count = 0; return null }
        if (name == cur) count++ else { cur = name; count = 1 }
        if (count != holdFrames) return null
        if (nowMs - (lastFired[name] ?: Long.MIN_VALUE / 2) < cooldownMs) return null
        lastFired[name] = nowMs
        return map(name)
    }
}

class Behavior(
    val planner: Planner = Planner(),
    val follower: Follower = Follower(planner = planner),
    var waveSlot: Int = 0,
    var intentTtlMs: Int = 600,
) {
    var activity = Activity.IDLE; private set
    var reason = ""; private set
    private var manualVx = 0f; private var manualYaw = 0f; private var manualUntil = 0L
    private var lastGoMs = -10_000L; private var lastObstacleSay = -100_000L; private var lostSaid = false
    private var wasFallen = false

    private fun set(a: Activity, why: String, out: MutableList<Action>) {
        if (a != activity || why != reason) out += Action.State(a, why)
        activity = a; reason = why
    }

    fun request(r: Request, nowMs: Long, robot: RobotView): List<Action> {
        val out = mutableListOf<Action>()
        when (r) {
            Request.Stop -> { set(Activity.STOPPED, "perintah berhenti", out); out += Action.Gait(0f, 0f, 0); out += Action.Say("Berhenti.") }
            Request.Wander -> { set(Activity.WANDER, "jelajah sambil menghindari rintangan", out); out += Action.Say("Aku jalan-jalan, ya.") }
            Request.Follow -> { set(Activity.FOLLOW, "mengikuti orang", out); lostSaid = false; out += Action.Say("Siap, aku ikuti kamu.") }
            Request.Wave -> if (robot.mode == 720) { out += Action.Motion(waveSlot); out += Action.Say("Halo!") }
                            else out += Action.Say("Aku harus berdiri diam dulu untuk melambai.")
            Request.Home -> { set(Activity.IDLE, "kembali ke posisi awal", out); out += Action.Gait(0f, 0f, 0); out += Action.Cmd(0) }
            Request.Balance -> { if (robot.mode == 720) out += Action.Cmd(1); out += Action.Say("Mode seimbang.") }
            is Request.Walk -> { manualVx = r.vxMm; manualYaw = 0f; manualUntil = nowMs + (r.seconds * 1000).toLong(); set(Activity.MANUAL, if (r.vxMm >= 0) "maju" else "mundur", out) }
            is Request.Turn -> { manualVx = 0f; manualYaw = r.yawDeg; manualUntil = nowMs + (r.seconds * 1000).toLong(); set(Activity.MANUAL, if (r.yawDeg > 0) "belok kiri" else "belok kanan", out) }
            is Request.Say -> out += Action.Say(r.text)
            Request.Describe -> out += Action.DescribeScene
        }
        return out
    }

    /** Dipanggil ~5 Hz. */
    fun tick(nowMs: Long, robot: RobotView, map: ObstacleMap?, person: PersonTarget?): List<Action> {
        val out = mutableListOf<Action>()
        val fallen = robot.mode == 705
        if (fallen && !wasFallen) { set(Activity.IDLE, "robot jatuh", out); out += Action.Say("Aku jatuh. Tolong bantu aku berdiri.") }
        wasFallen = fallen
        if (!robot.alive || fallen) return out
        if (activity == Activity.MANUAL && nowMs >= manualUntil) { set(Activity.IDLE, "selesai", out); out += Action.Gait(0f, 0f, 0); return out }
        if (activity !in setOf(Activity.WANDER, Activity.FOLLOW, Activity.MANUAL)) return out
        // perlu mode seimbang / jalan
        if (robot.mode != 730 && robot.mode != 800) {
            if (robot.mode == 720 && nowMs - lastGoMs > 2000) { lastGoMs = nowMs; out += Action.Cmd(1) }
            return out
        }
        val m = map ?: ObstacleMap(30f, IntArray(9) { ObstacleMap.FREE })
        val step = when (activity) {
            Activity.WANDER -> planner.plan(m)
            Activity.FOLLOW -> {
                if (person == null && !lostSaid) { lostSaid = true; out += Action.Say("Kamu di mana?") }
                if (person != null) lostSaid = false
                follower.step(person, m)
            }
            else -> if (manualVx > 0) planner.plan(m, 0f, manualVx).let { it.copy(yawDeg = if (it.vxMm == 0f) 0f else it.yawDeg) }
                    else Step(manualVx, manualYaw, reason)
        }
        if (m.frontMin(25f) < planner.stopMm && (activity == Activity.WANDER || manualVx > 0) && nowMs - lastObstacleSay > 8000) {
            lastObstacleSay = nowMs; out += Action.Say("Ada rintangan di depan.")
        }
        out += Action.Gait(step.vxMm, step.yawDeg, intentTtlMs)
        if (activity != Activity.MANUAL) set(activity, step.reason, out)
        return out
    }
}

/** Kalimat ringkas tentang apa yang terlihat (untuk "apa yang kamu lihat"). */
object Describe {
    private val id = mapOf("person" to "orang", "chair" to "kursi", "cup" to "cangkir", "bottle" to "botol", "cat" to "kucing",
        "dog" to "anjing", "laptop" to "laptop", "cell phone" to "HP", "book" to "buku", "tv" to "televisi",
        "couch" to "sofa", "dining table" to "meja", "potted plant" to "tanaman", "car" to "mobil", "bicycle" to "sepeda",
        "backpack" to "tas", "ball" to "bola", "sports ball" to "bola", "teddy bear" to "boneka", "keyboard" to "keyboard",
        "mouse" to "mouse", "clock" to "jam", "bed" to "kasur", "toilet" to "toilet", "bird" to "burung")
    fun sentence(labels: List<String>, faces: List<String> = emptyList()): String {
        val names = faces.filter { it.isNotBlank() }
        val counts = labels.groupingBy { id[it] ?: it }.eachCount().entries.sortedByDescending { it.value }.take(4)
        if (counts.isEmpty() && names.isEmpty()) return "Aku tidak melihat apa-apa yang kukenal."
        val parts = counts.map { (k, v) -> if (v == 1) "satu $k" else "$v $k" }
        val who = if (names.isNotEmpty()) " Aku kenal ${names.joinToString(" dan ")}." else ""
        return "Aku melihat ${parts.joinToString(", ")}.$who"
    }
}
