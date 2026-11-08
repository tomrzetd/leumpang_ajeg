// Leumpang Bus: publish/subscribe topik antar-modul (di dalam HP) dan, lewat BusSession, ke luar
// (WebSocket JSON: studio PC, skrip Python, board sensor Wi-Fi). Topik bergaya MQTT: "perception/obstacles",
// pola langganan dengan "+" (satu segmen) dan "#" (sisa segmen).
package id.leumpang.bus

import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonNull
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.CopyOnWriteArrayList

data class BusMessage(val topic: String, val data: JsonElement, val source: String, val tMs: Long)

fun interface Subscription { fun cancel() }

class Bus(private val clock: () -> Long = System::currentTimeMillis) {
    private class Sub(val pattern: String, val owner: String, val fn: (BusMessage) -> Unit)
    private val subs = CopyOnWriteArrayList<Sub>()
    private val retained = ConcurrentHashMap<String, BusMessage>()
    @Volatile var published = 0L; private set

    /** Terbitkan. retain = simpan nilai terakhir (dikirim ke pelanggan baru & tersedia lewat last()). */
    fun publish(topic: String, data: JsonElement = JsonNull, source: String = "", retain: Boolean = true) {
        require(validTopic(topic)) { "topik tidak valid: $topic" }
        val m = BusMessage(topic, data, source, clock())
        if (retain) retained[topic] = m
        published++
        for (s in subs) if (matches(s.pattern, topic)) {
            try { s.fn(m) } catch (e: Exception) { System.err.println("bus: pelanggan '${s.owner}' gagal pada $topic: $e") }
        }
    }

    fun subscribe(pattern: String, owner: String = "", replayRetained: Boolean = false, fn: (BusMessage) -> Unit): Subscription {
        require(validPattern(pattern)) { "pola tidak valid: $pattern" }
        val s = Sub(pattern, owner, fn)
        subs += s
        if (replayRetained) retained.values.filter { matches(pattern, it.topic) }.sortedBy { it.tMs }.forEach(fn)
        return Subscription { subs.remove(s) }
    }

    fun last(topic: String): BusMessage? = retained[topic]
    fun topics(): List<String> = retained.keys.sorted()
    fun subscriberCount() = subs.size

    companion object {
        private val seg = Regex("[A-Za-z0-9_\\-.]+")
        fun validTopic(t: String) = t.isNotEmpty() && t.length <= 128 && t.split('/').all { seg.matches(it) }
        fun validPattern(p: String): Boolean {
            if (p.isEmpty() || p.length > 128) return false
            val parts = p.split('/')
            return parts.withIndex().all { (i, s) -> s == "+" || (s == "#" && i == parts.size - 1) || seg.matches(s) }
        }
        fun matches(pattern: String, topic: String): Boolean {
            val p = pattern.split('/'); val t = topic.split('/')
            for (i in p.indices) {
                if (p[i] == "#") return true
                if (i >= t.size) return false
                if (p[i] != "+" && p[i] != t[i]) return false
            }
            return p.size == t.size
        }
    }
}

/** Deskripsi modul untuk penemuan (describe) oleh klien luar. */
data class ModuleInfo(val id: String, val description: String, val publishes: List<String>, val subscribes: List<String>)

/** Modul indera / perilaku. start() dipanggil sekali dengan bus; stop() melepas sumber daya. */
interface Module {
    val info: ModuleInfo
    fun start(bus: Bus)
    fun stop() {}
}

class ModuleHost(val bus: Bus) {
    private val modules = CopyOnWriteArrayList<Module>()
    private val running = ConcurrentHashMap.newKeySet<String>()
    fun add(m: Module) = apply { require(modules.none { it.info.id == m.info.id }) { "modul ganda: ${m.info.id}" }; modules += m }
    fun start(id: String) { modules.find { it.info.id == id }?.let { if (running.add(id)) it.start(bus) } }
    fun stop(id: String) { modules.find { it.info.id == id }?.let { if (running.remove(id)) it.stop() } }
    fun startAll() = modules.forEach { start(it.info.id) }
    fun stopAll() = modules.reversed().forEach { stop(it.info.id) }
    fun isRunning(id: String) = id in running
    fun infos(): List<ModuleInfo> = modules.map { it.info }
}
