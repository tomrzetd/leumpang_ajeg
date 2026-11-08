package id.leumpang.perception

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.random.Random

class PerceptionTest {
    private val cam = CameraModel(160, 120, 66f, 51f)
    private val pose = CameraPose(0.40f, 20f)

    /** Render adegan sintetis: lantai bertekstur + kotak merah (depan x = boxX, lebar y0..y1, tinggi h). */
    private fun scene(boxX: Float, y0: Float, y1: Float, h: Float = 0.3f, seed: Int = 1): RgbImage {
        val rnd = Random(seed)
        val px = IntArray(cam.width * cam.height)
        val p = Math.toRadians(pose.pitchDownDeg.toDouble())
        for (v in 0 until cam.height) for (u in 0 until cam.width) {
            val x = (u - cam.cx) / cam.fx; val y = (v - cam.cy) / cam.fy
            val fwd = cos(p) - y * sin(p); val down = sin(p) + y * cos(p); val left = -x.toDouble()
            var c = 0xB4B4BE                                                     // dinding
            val tBox = if (fwd > 1e-6) boxX / fwd else Double.MAX_VALUE
            val yb = tBox * left; val zb = pose.heightM - tBox * down
            val tFloor = if (down > 1e-6) pose.heightM / down else Double.MAX_VALUE
            if (yb in y0.toDouble()..y1.toDouble() && zb in 0.0..h.toDouble() && tBox < tFloor) c = 0xC83228
            else if (down > 1e-6) {                                              // lantai keramik krem + noise + nat
                val gx = tFloor * fwd; val gy = tFloor * left
                val grout = (((gx * 3.3) % 1.0) < 0.03) || (((gy * 3.3 + 5) % 1.0) < 0.03)
                val n = rnd.nextInt(-10, 11)
                val base = if (grout) 0xA09A8C else 0xC8BEA8
                c = (((base shr 16 and 0xFF) + n).coerceIn(0, 255) shl 16) or (((base shr 8 and 0xFF) + n).coerceIn(0, 255) shl 8) or ((base and 0xFF) + n).coerceIn(0, 255)
            }
            px[v * cam.width + u] = c
        }
        return RgbImage(cam.width, cam.height, px)
    }

    @Test fun geometriLantai() {
        val g = Ground.project(cam, pose, cam.cx, cam.cy)
        assertNotNull(g)
        assertEquals(0.40 / Math.tan(Math.toRadians(20.0)), g!!.xM.toDouble(), 1e-3)
        assertEquals(0f, g.bearingDeg, 0.01f)
        assertNull(Ground.project(cam, pose, cam.cx, Ground.horizonRow(cam, pose) - 2))
        val left = Ground.project(cam, pose, 10f, 110f)!!
        assertTrue("kolom kiri gambar = bearing positif (kiri robot)", left.bearingDeg > 0)
        val yawed = Ground.project(cam, pose.copy(yawDeg = 30f), cam.cx, cam.cy)!!
        assertEquals(30f, yawed.bearingDeg, 0.1f)
        val fr = CameraPose.fromRobot(185f, 0f, 0, 20f)
        assertEquals(0.4065f, fr.heightM, 1e-4f)
    }

    @Test fun rintanganDariGambarSintetis() {
        for ((boxX, y0, y1) in listOf(Triple(0.8f, -0.10f, 0.20f), Triple(1.4f, 0.15f, 0.45f), Triple(0.6f, -0.35f, -0.12f))) {
            val img = scene(boxX, y0, y1)
            val fs = FreeSpace()
            val b = fs.boundaries(img, Ground.horizonRow(cam, pose).toInt())
            val map = ObstacleMap.build(cam, pose, b, emptyList())
            val bearing = Math.toDegrees(atan2(((y0 + y1) / 2).toDouble(), boxX.toDouble())).toFloat()
            val s = map.sectorOf(bearing)!!
            val expect = boxX * 1000
            println("      kotak x=${boxX} m bearing ${"%.1f".format(bearing)}: sektor $s = ${map.distMm[s]} mm, peta ${map.distMm.joinToString()}")
            assertTrue("jarak sektor kotak ~ ${expect} mm (dapat ${map.distMm[s]})", abs(map.distMm[s] - expect) < expect * 0.15f)
            val far = map.distMm.indices.filter { abs(map.centerDeg(it) - bearing) > 20f }
            assertTrue("sektor jauh dari kotak bebas (lantai tidak dianggap rintangan)", far.all { map.distMm[it] >= 1500 })
        }
    }

    @Test fun deteksiObjekJadiRintangan() {
        // kotak deteksi: kaki kursi menyentuh lantai di baris 100 -> jarak dari proyeksi
        val det = Detection("chair", 0.8f, 0.45f, 0.4f, 0.55f, 100f / 120f)
        val map = ObstacleMap.build(cam, pose, null, listOf(det))
        val g = Ground.project(cam, pose, 0.5f * cam.width, 100f)!!
        assertEquals((g.distM * 1000).toInt(), map.frontMin())
    }

    @Test fun perencana() {
        val pl = Planner()
        val free = ObstacleMap(33f, IntArray(9) { ObstacleMap.FREE })
        val s0 = pl.plan(free)
        assertEquals(pl.maxVx, s0.vxMm, 1e-3f); assertEquals(0f, s0.yawDeg, 1e-3f)
        val blockLeft = ObstacleMap(33f, intArrayOf(9999, 9999, 9999, 9999, 300, 300, 300, 9999, 9999))
        val s1 = pl.plan(blockLeft)
        assertEquals("rintangan depan < stop: tidak maju", 0f, s1.vxMm, 1e-3f)
        assertTrue("menghindar ke kanan (yaw -)", s1.yawDeg < 0)
        val near = ObstacleMap(33f, IntArray(9) { 700 })
        assertTrue(pl.plan(near).vxMm in 0.1f..pl.maxVx - 0.1f)
        val trapped = ObstacleMap(33f, IntArray(9) { 250 })
        val s3 = pl.plan(trapped)
        assertEquals(0f, s3.vxMm, 1e-3f); assertTrue(s3.yawDeg != 0f)
    }

    @Test fun pengikutOrang() {
        val tr = PersonTracker()
        // kaki di luar bingkai: jarak dari tinggi kotak (1,65 m terlihat 72 px dari 120)
        val box = Detection("person", 0.9f, 0.55f, 0.4f, 0.65f, 1.0f)
        val t = tr.update(listOf(box, Detection("chair", 0.9f, 0f, 0f, 0.1f, 0.1f)), cam, pose, 1000)!!
        assertEquals(1.65f * cam.fy / 72f * 1000, t.distMm.toFloat(), 5f)
        // kaki terlihat: jarak dari titik kontak di lantai (lebih akurat)
        val feet = Detection("person", 0.9f, 0.45f, 0.1f, 0.55f, 0.9f)
        val tf = PersonTracker().update(listOf(feet), cam, pose, 0)!!
        assertEquals(Ground.project(cam, pose, 0.5f * cam.width, 0.9f * cam.height)!!.distM * 1000, tf.distMm.toFloat(), 2f)
        assertTrue("orang di kanan gambar -> bearing negatif", t.bearingDeg < 0)
        val f = Follower()
        val free = ObstacleMap(33f, IntArray(9) { ObstacleMap.FREE })
        val far = f.step(PersonTarget(10f, 2500, 0.9f, box), free)
        assertTrue(far.vxMm > 0 && far.yawDeg > 0)
        val close = f.step(PersonTarget(2f, 1000, 0.9f, box), free)
        assertEquals(0f, close.vxMm, 1e-3f)
        assertNull("hilang setelah lostAfterMs", tr.update(emptyList(), cam, pose, 1000 + 2000))
    }

    @Test fun ucapan() {
        assertEquals(Request.Stop, Speech.parse("Robot, berhenti!"))
        assertEquals(Request.Follow, Speech.parse("ikuti saya"))
        assertEquals(Request.Follow, Speech.parse("Leumpang tuturkeun"))
        assertEquals(Request.Wave, Speech.parse("lambaikan tangan dong"))
        assertEquals(Request.Wander, Speech.parse("jalan jalan yuk"))
        assertTrue(Speech.parse("jalan") is Request.Walk)
        assertEquals(Request.Turn(4f), Speech.parse("belok kiri"))
        assertEquals(Request.Turn(-4f), Speech.parse("belok kanan"))
        assertTrue((Speech.parse("mundur") as Request.Walk).vxMm < 0)
        assertEquals(Request.Describe, Speech.parse("apa yang kamu lihat?"))
        assertTrue(Speech.parse("siapa kamu") is Request.Say)
        assertNull(Speech.parse("cuaca hari ini cerah"))
        assertNull("wajib kata bangun", Speech.parse("berhenti", requireWake = true))
        assertEquals(Request.Stop, Speech.parse("hai robot stop", requireWake = true))
    }

    @Test fun isyaratDebounce() {
        val g = Gestures(holdFrames = 3, cooldownMs = 2000)
        assertNull(g.update("Open_Palm", 0.9f, 0)); assertNull(g.update("Open_Palm", 0.9f, 100))
        assertEquals(Request.Stop, g.update("Open_Palm", 0.9f, 200))
        assertNull("tidak berulang tiap frame", g.update("Open_Palm", 0.9f, 300))
        repeat(3) { g.update("Thumb_Up", 0.4f, 400L + it) }
        assertNull("skor rendah diabaikan", g.update("Thumb_Up", 0.4f, 500))
        assertNull(g.update("Thumb_Up", 0.9f, 600)); assertNull(g.update("Thumb_Up", 0.9f, 700))
        assertEquals(Request.Follow, g.update("Thumb_Up", 0.9f, 800))
    }

    @Test fun arbiterPerilaku() {
        val b = Behavior()
        val ready = RobotView(true, 720)
        val a0 = b.request(Request.Wander, 0, ready)
        assertTrue(a0.any { it is Action.Say })
        val a1 = b.tick(100, ready, null, null)
        assertTrue("READY -> kirim GO dulu", a1.contains(Action.Cmd(1)))
        assertTrue("belum melangkah sebelum BALANCE", a1.none { it is Action.Gait })
        val bal = RobotView(true, 730)
        val wall = ObstacleMap(33f, IntArray(9) { 200 })
        val a2 = b.tick(300, bal, wall, null)
        val g2 = a2.filterIsInstance<Action.Gait>().single()
        assertEquals(0f, g2.vxMm, 1e-3f)
        assertTrue(a2.any { it is Action.Say && it.text.contains("rintangan") })
        val a3 = b.tick(500, bal, ObstacleMap(33f, IntArray(9) { ObstacleMap.FREE }), null)
        assertTrue(a3.filterIsInstance<Action.Gait>().single().vxMm > 0)
        val a4 = b.request(Request.Stop, 600, bal)
        assertTrue(a4.contains(Action.Gait(0f, 0f, 0)))
        assertTrue("diam: tidak ada niat jalan", b.tick(800, bal, null, null).none { it is Action.Gait })
        val a5 = b.tick(1000, RobotView(true, 705), null, null)
        assertTrue(a5.any { it is Action.Say && it.text.contains("jatuh") })
        assertTrue(b.request(Request.Wave, 1200, ready).contains(Action.Motion(0)))
        assertEquals("Aku melihat 2 orang, satu kursi. Aku kenal Ujang.", Describe.sentence(listOf("person", "chair", "person"), listOf("Ujang")))
    }
}
