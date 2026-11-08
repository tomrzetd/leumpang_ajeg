package id.leumpang.lap

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Assume.assumeTrue
import org.junit.Test
import java.net.InetSocketAddress
import java.net.Socket
import kotlin.random.Random

class LapTest {
    @Test fun crcVektorStandar() {
        assertEquals(0x29B1, Lap.crc16("123456789".toByteArray()))
    }

    @Test fun cobsVektor() {
        fun h(vararg v: Int) = ByteArray(v.size) { v[it].toByte() }
        assertArrayEquals(h(1, 1), Lap.cobsEncode(h(0)))
        assertArrayEquals(h(3, 0x11, 0x22, 2, 0x33), Lap.cobsEncode(h(0x11, 0x22, 0, 0x33)))
        val big = ByteArray(300) { (it % 255 + 1).toByte() }
        assertArrayEquals(big, Lap.cobsDecode(Lap.cobsEncode(big)))
        assertNull(Lap.cobsDecode(h(5, 1, 2)))
    }

    @Test fun frameAcakRoundtripDanResync() {
        val rnd = Random(7)
        val d = Decoder()
        repeat(3000) {
            val body = ByteArray(rnd.nextInt(0, Lap.MAX_BODY + 1)) { rnd.nextInt(256).toByte() }
            val type = rnd.nextInt(1, 255); val seq = rnd.nextInt(256)
            val junk = ByteArray(rnd.nextInt(0, 6)) { (rnd.nextInt(1, 255)).toByte() } + byteArrayOf(0)
            var got: Packet? = null
            for (b in junk + Lap.encodeFrame(type, seq, body)) d.feed(b)?.let { got = it }
            assertNotNull(got)
            assertEquals(type, got!!.type); assertEquals(seq, got!!.seq); assertArrayEquals(body, got!!.body)
        }
        val f = Lap.encodeFrame(Lap.PING, 1, byteArrayOf(1, 2, 3)); f[2] = (f[2] + 1).toByte()
        var bad: Packet? = null; for (b in f) d.feed(b)?.let { bad = it }
        assertNull(bad); assertEquals(1, d.crcErrors)
    }

    @Test fun ukuranPesanSamaDenganCpp() {
        assertEquals(26, Hello().enc().size)
        assertEquals(14, Msg.gaitIntent(1f, 0f, 2f, 300).size)
        assertEquals(50, Msg.poseTarget(Array(2) { IntArray(SLOTS) }, 0, 300).size)
        val p = Perception(400, 30f, IntArray(9) { 2000 }.also { it[4] = 150 }, 5f, 1200)
        assertEquals(2 + 1 + 1 + 2 + 18 + 4, p.enc().size)
        assertEquals(150, p.frontMin())
        assertEquals(0f, p.sectorCenterDeg(4), 1e-4f)
    }

    /** Integrasi: klien Kotlin <-> leumpang_sim C++ lewat TCP (CI menyalakan sim; lokal: LEUMPANG_SIM=1). */
    @Test fun integrasiDenganSimulator() {
        val port = (System.getenv("LEUMPANG_SIM_PORT") ?: "5760").toInt()
        val sock = try { Socket().apply { connect(InetSocketAddress("127.0.0.1", port), 500) } } catch (e: Exception) { null }
        assumeTrue("simulator tidak berjalan di :$port", sock != null)
        sock!!.soTimeout = 100
        val out = sock.getOutputStream(); val inp = sock.getInputStream()
        val cli = LapClient { synchronized(out) { out.write(it); out.flush() } }
        val logs = mutableListOf<String>(); val exts = mutableListOf<Pair<Int, ByteArray>>(); val acks = mutableListOf<Ack>()
        cli.addListener(object : LapClient.Listener {
            override fun onLog(line: String) { logs += line }
            override fun onExt(topic: Int, data: ByteArray) { exts += topic to data }
            override fun onAck(ack: Ack) { acks += ack }
        })
        val buf = ByteArray(4096)
        fun pump(ms: Long, until: () -> Boolean = { false }) {
            val end = System.currentTimeMillis() + ms
            while (System.currentTimeMillis() < end && !until()) {
                try { val n = inp.read(buf); if (n > 0) cli.feed(buf, n) } catch (_: java.net.SocketTimeoutException) {}
            }
        }
        cli.begin()
        pump(4000) { cli.telemetry?.mode == Lap.MODE_READY }
        assertTrue("HELLO dari sim", cli.hello?.board == Lap.BOARD_SIM)
        assertTrue("sim mengiklankan PERCEPTION", cli.hello!!.has(Lap.CAP_PERCEPTION))
        assertEquals(Lap.MODE_READY, cli.telemetry?.mode)
        cli.ext(0x0001, "halo".toByteArray())
        pump(1000) { exts.isNotEmpty() }
        assertTrue("EXT echo", exts.any { it.first == 1 && String(it.second) == "halo" })
        cli.setParam(Lap.P_GAIT_ENABLE, 1f)
        pump(5000) { (cli.telemetry?.flags ?: 0) and Lap.TF_CAL_DONE != 0 }   // GO ditolak sebelum kalibrasi IMU
        cli.cmd(Lap.CMD_GO)
        pump(1500) { cli.telemetry?.mode == Lap.MODE_BALANCE }
        assertEquals(Lap.MODE_BALANCE, cli.telemetry?.mode)
        // jalan maju dengan rintangan dekat -> firmware memblokir langkah maju
        var blocked = false
        repeat(15) {
            cli.gaitIntent(9f, 0f, 500)
            cli.perception(Perception(400, 30f, IntArray(9) { 0xFFFF }.also { it[4] = 120 }))
            pump(200) { false }
            blocked = blocked || (cli.telemetry?.obstacleBlock == true)
        }
        assertEquals(Lap.MODE_WALK_CONT, cli.telemetry?.mode)
        assertTrue("TF_OBSTACLE_BLOCK terlihat", blocked)
        assertTrue("log refleks", logs.any { it.contains("rintangan") })
        cli.cmd(Lap.CMD_RESET)
        sock.close()
    }
}
