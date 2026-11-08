package id.leumpang.bus

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class BusTest {
    @Test fun polaTopik() {
        assertTrue(Bus.matches("perception/#", "perception/obstacles"))
        assertTrue(Bus.matches("perception/#", "perception"))
        assertTrue(Bus.matches("vision/+/score", "vision/face/score"))
        assertFalse(Bus.matches("vision/+", "vision/face/score"))
        assertFalse(Bus.matches("robot/telemetry", "robot/log"))
        assertFalse(Bus.validPattern("a/#/b"))
        assertFalse(Bus.validTopic("a//b"))
        assertFalse(Bus.validTopic("a/+"))
    }

    @Test fun terbitLanggananRetain() {
        val bus = Bus()
        val got = mutableListOf<String>()
        bus.publish("robot/telemetry", J.obj("mode" to 720), "lap")
        val s = bus.subscribe("robot/#", replayRetained = true) { got += it.topic }
        bus.publish("robot/log", J.obj("line" to "halo"))
        bus.publish("vision/objects", J.obj())
        assertEquals(listOf("robot/telemetry", "robot/log"), got)
        s.cancel()
        bus.publish("robot/log", J.obj("line" to "lagi"))
        assertEquals(2, got.size)
        assertEquals(720.0, J.num(bus.last("robot/telemetry")?.data, "mode")!!, 0.0)
    }

    @Test fun pelangganGagalTidakMematikanBus() {
        val bus = Bus(); var ok = 0
        bus.subscribe("a") { throw IllegalStateException("x") }
        bus.subscribe("a") { ok++ }
        bus.publish("a")
        assertEquals(1, ok)
    }

    @Test fun moduleHost() {
        val bus = Bus(); val host = ModuleHost(bus); var started = 0
        host.add(object : Module {
            override val info = ModuleInfo("dummy", "uji", listOf("sensor/dummy"), emptyList())
            override fun start(bus: Bus) { started++; bus.publish("sensor/dummy", J.obj("v" to 1)) }
        })
        host.startAll(); host.startAll()
        assertEquals(1, started)
        assertTrue(host.isRunning("dummy"))
    }

    @Test fun sesiJsonTokenIzinDescribe() {
        val bus = Bus(); val host = ModuleHost(bus)
        host.add(object : Module {
            override val info = ModuleInfo("vision", "kamera", listOf("vision/objects"), emptyList())
            override fun start(bus: Bus) {}
        })
        val out = mutableListOf<String>()
        val s = BusSession(bus, host, { out += it }, token = "rahasia")
        fun last() = Json.parseToJsonElement(out.last()).jsonObject
        s.onText("""{"op":"sub","topics":["vision/#"]}""")
        assertEquals("error", last()["op"]!!.jsonPrimitive.content)
        s.onText("""{"op":"hello","token":"salah"}""")
        assertEquals("error", last()["op"]!!.jsonPrimitive.content)
        s.onText("""{"op":"hello","token":"rahasia","name":"studio"}""")
        assertEquals("ok", last()["op"]!!.jsonPrimitive.content)
        s.onText("""{"op":"sub","topics":["vision/#"]}""")
        bus.publish("vision/objects", J.obj("n" to 2), "vision")
        assertEquals("vision/objects", last()["topic"]!!.jsonPrimitive.content)
        s.onText("""{"op":"pub","topic":"robot/cmd","data":{}}""")
        assertTrue(out.last().contains("tidak boleh"))
        s.onText("""{"op":"pub","topic":"sensor/jarak","data":{"mm":420}}""")
        assertEquals(420.0, J.num(bus.last("sensor/jarak")?.data, "mm")!!, 0.0)
        s.onText("""{"op":"describe"}""")
        assertTrue(out.last().contains("\"vision\"") && out.last().contains("sensor/jarak"))
        s.onText("bukan json")
        assertTrue(out.last().contains("JSON tidak valid"))
        s.close()
        val n = out.size
        bus.publish("vision/objects", J.obj())
        assertEquals(n, out.size)
    }
}
