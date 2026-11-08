// Sesi klien luar Leumpang Bus (JSON teks, satu pesan per frame WebSocket). Tanpa ketergantungan transport.
//
// Klien -> HP:
//   {"op":"hello","token":"...","name":"studio"}           wajib pertama bila token diset
//   {"op":"sub","topics":["perception/#","robot/telemetry"]} {"op":"unsub","topics":[...]}
//   {"op":"pub","topic":"sensor/jarak","data":{...}}        hanya topik yang diizinkan (lihat allowPublish)
//   {"op":"get","topic":"robot/telemetry"}                   nilai terakhir
//   {"op":"describe"}                                         modul + topik
// HP -> klien:
//   {"op":"msg","topic":"...","t":1234,"src":"vision","data":{...}}
//   {"op":"ok","re":"sub"} / {"op":"error","re":"pub","msg":"..."} / {"op":"describe",...}
package id.leumpang.bus

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.put

class BusSession(
    private val bus: Bus,
    private val host: ModuleHost?,
    private val send: (String) -> Unit,
    private val token: String? = null,
    /** pola topik yang boleh diterbitkan klien luar; default: sensor & permintaan perilaku & EXT robot & ucapan */
    private val allowPublish: List<String> = listOf("sensor/#", "behavior/request", "robot/ext/out", "speech/say", "app/#"),
    private val maxSubs: Int = 32,
) {
    private val subs = mutableMapOf<String, Subscription>()
    var clientName = "?"; private set
    var authed = token.isNullOrEmpty(); private set
    private val lock = Any()

    fun onText(text: String) {
        val o = try { Json.parseToJsonElement(text).jsonObject } catch (e: Exception) { reply("error", "?", "JSON tidak valid"); return }
        val op = o["op"]?.jsonPrimitive?.content ?: run { reply("error", "?", "tanpa 'op'"); return }
        if (!authed && op != "hello") { reply("error", op, "kirim hello+token dulu"); return }
        when (op) {
            "hello" -> {
                clientName = o["name"]?.jsonPrimitive?.content?.take(32) ?: "?"
                if (!token.isNullOrEmpty() && o["token"]?.jsonPrimitive?.content != token) { reply("error", op, "token salah"); return }
                authed = true
                reply("ok", op)
            }
            "sub" -> {
                val ts = o["topics"]?.jsonArray?.map { it.jsonPrimitive.content } ?: emptyList()
                synchronized(lock) {
                    for (p in ts) {
                        if (!Bus.validPattern(p)) { reply("error", op, "pola tidak valid: $p"); return }
                        if (subs.size >= maxSubs) { reply("error", op, "terlalu banyak langganan"); return }
                        if (p !in subs) subs[p] = bus.subscribe(p, "ws:$clientName", replayRetained = true) { m -> deliver(m) }
                    }
                }
                reply("ok", op)
            }
            "unsub" -> {
                val ts = o["topics"]?.jsonArray?.map { it.jsonPrimitive.content } ?: emptyList()
                synchronized(lock) { ts.forEach { subs.remove(it)?.cancel() } }
                reply("ok", op)
            }
            "pub" -> {
                val topic = o["topic"]?.jsonPrimitive?.content ?: ""
                if (!Bus.validTopic(topic)) { reply("error", op, "topik tidak valid"); return }
                if (allowPublish.none { Bus.matches(it, topic) }) { reply("error", op, "topik '$topic' tidak boleh diterbitkan dari luar"); return }
                bus.publish(topic, o["data"] ?: JsonNull, source = "ws:$clientName")
                reply("ok", op)
            }
            "get" -> {
                val topic = o["topic"]?.jsonPrimitive?.content ?: ""
                val m = bus.last(topic)
                if (m == null) reply("error", op, "belum ada nilai untuk '$topic'") else deliver(m)
            }
            "describe" -> send(describe().toString())
            else -> reply("error", op, "op tidak dikenal")
        }
    }

    fun describe(): JsonObject = buildJsonObject {
        put("op", "describe")
        put("modules", JsonArray(host?.infos()?.map { i ->
            buildJsonObject {
                put("id", i.id); put("description", i.description); put("running", host.isRunning(i.id))
                put("publishes", JsonArray(i.publishes.map { JsonPrimitive(it) }))
                put("subscribes", JsonArray(i.subscribes.map { JsonPrimitive(it) }))
            }
        } ?: emptyList()))
        put("topics", JsonArray(bus.topics().map { JsonPrimitive(it) }))
        put("allowPublish", JsonArray(allowPublish.map { JsonPrimitive(it) }))
    }

    private fun deliver(m: BusMessage) {
        send(buildJsonObject {
            put("op", "msg"); put("topic", m.topic); put("t", m.tMs); put("src", m.source); put("data", m.data)
        }.toString())
    }

    private fun reply(kind: String, re: String, msg: String? = null) {
        send(buildJsonObject { put("op", kind); put("re", re); if (msg != null) put("msg", msg) }.toString())
    }

    fun close() = synchronized(lock) { subs.values.forEach { it.cancel() }; subs.clear() }
}

/** Pembantu membangun data JSON ringkas. */
object J {
    fun obj(vararg kv: Pair<String, Any?>): JsonObject = JsonObject(kv.associate { (k, v) -> k to of(v) })
    fun of(v: Any?): JsonElement = when (v) {
        null -> JsonNull
        is JsonElement -> v
        is Number -> JsonPrimitive(v)
        is Boolean -> JsonPrimitive(v)
        is String -> JsonPrimitive(v)
        is IntArray -> JsonArray(v.map { JsonPrimitive(it) })
        is FloatArray -> JsonArray(v.map { JsonPrimitive(it) })
        is Iterable<*> -> JsonArray(v.map { of(it) })
        is Map<*, *> -> JsonObject(v.entries.associate { (k, x) -> k.toString() to of(x) })
        else -> JsonPrimitive(v.toString())
    }
    fun str(e: JsonElement?, k: String): String? = (e as? JsonObject)?.get(k)?.jsonPrimitive?.content
    fun num(e: JsonElement?, k: String): Double? = (e as? JsonObject)?.get(k)?.jsonPrimitive?.content?.toDoubleOrNull()
}
