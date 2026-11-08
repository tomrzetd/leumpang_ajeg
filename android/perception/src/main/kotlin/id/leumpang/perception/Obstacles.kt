// Deteksi rintangan tanpa sensor jarak (Xiaomi 12 Pro / Redmi Note 11 Pro 5G tidak punya ToF/UWB):
//  1. Ruang bebas berbasis tampilan lantai (Ulrich & Nourbakhsh 2000): model warna lantai dari petak tepat di
//     depan kaki, tiap kolom dipindai dari bawah ke atas sampai piksel bukan-lantai -> batas rintangan.
//  2. Batas itu + tepi bawah kotak deteksi objek diproyeksikan ke lantai (geometri kamera kepala).
//  3. Hasil: histogram polar (sektor) jarak terdekat -> pesan PERCEPTION + perencana langkah.
package id.leumpang.perception

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** Gambar RGB kecil (mis. 160x120) dalam IntArray 0xRRGGBB, baris demi baris. */
class RgbImage(val width: Int, val height: Int, val px: IntArray) {
    init { require(px.size >= width * height) }
    operator fun get(x: Int, y: Int) = px[y * width + x]
}

class FreeSpace(
    var refRows: Int = 6,              // tinggi petak referensi lantai (baris paling bawah)
    var refWidthFrac: Float = 0.4f,    // lebar petak referensi (pusat)
    var threshold: Float = 3.2f,       // ambang jarak ternormalisasi
    var runLength: Int = 3,            // piksel bukan-lantai berturut-turut = rintangan
) {
    // model lantai: rata-rata & simpangan (kroma r,g ternormalisasi + luma) - tahan perubahan terang
    private var mean = FloatArray(3); private var sd = FloatArray(3) { 1f }
    var floorValid = false; private set

    private fun feat(c: Int, out: FloatArray) {
        val r = (c shr 16 and 0xFF).toFloat(); val g = (c shr 8 and 0xFF).toFloat(); val b = (c and 0xFF).toFloat()
        val s = r + g + b + 1f
        out[0] = r / s * 255f; out[1] = g / s * 255f; out[2] = (r + g + b) / 3f
    }

    fun learnFloor(img: RgbImage) {
        val x0 = ((1 - refWidthFrac) / 2 * img.width).toInt(); val x1 = img.width - x0
        val f = FloatArray(3); val sum = FloatArray(3); val sq = FloatArray(3); var n = 0
        for (y in img.height - refRows until img.height) for (x in x0 until x1) {
            feat(img[x, y], f); for (k in 0..2) { sum[k] += f[k]; sq[k] += f[k] * f[k] }; n++
        }
        if (n == 0) return
        val m = FloatArray(3) { sum[it] / n }
        val s = FloatArray(3) { sqrt(max(0f, sq[it] / n - m[it] * m[it])) }
        // simpangan minimum: kroma 4, luma 12 (lantai mulus pun punya noise & bayangan halus)
        s[0] = max(s[0], 4f); s[1] = max(s[1], 4f); s[2] = max(s[2], 12f)
        if (!floorValid) { mean = m; sd = s } else for (k in 0..2) { mean[k] = 0.8f * mean[k] + 0.2f * m[k]; sd[k] = 0.8f * sd[k] + 0.2f * s[k] }
        floorValid = true
    }

    fun isFloor(c: Int, f: FloatArray = FloatArray(3)): Boolean {
        feat(c, f)
        // luma diberi bobot lebih kecil agar bayangan tidak langsung dianggap rintangan
        val d0 = (f[0] - mean[0]) / sd[0]; val d1 = (f[1] - mean[1]) / sd[1]; val d2 = (f[2] - mean[2]) / sd[2] * 0.6f
        return d0 * d0 + d1 * d1 + d2 * d2 < threshold * threshold
    }

    /** Baris batas rintangan per kolom (baris piksel bukan-lantai pertama dari bawah); -1 = bebas sampai horizon. */
    fun boundaries(img: RgbImage, horizonRow: Int): IntArray {
        if (!floorValid) learnFloor(img)
        val out = IntArray(img.width) { -1 }
        val f = FloatArray(3)
        val top = horizonRow.coerceIn(0, img.height - 1)
        for (x in 0 until img.width) {
            var run = 0
            for (y in img.height - 1 downTo top) {
                if (isFloor(img[x, y], f)) run = 0
                else if (++run >= runLength) { out[x] = y + runLength - 1; break }
            }
        }
        // median 5 kolom: buang bintik
        return IntArray(out.size) { i -> (max(0, i - 2)..min(out.size - 1, i + 2)).map { out[it] }.sorted()[(min(out.size - 1, i + 2) - max(0, i - 2)) / 2] }
    }
}

/** Kotak deteksi ternormalisasi (0..1) di gambar. */
data class Detection(val label: String, val score: Float, val left: Float, val top: Float, val right: Float, val bottom: Float) {
    val cx get() = (left + right) / 2
    val h get() = bottom - top
}

data class ObstacleMap(val halfFovDeg: Float, val distMm: IntArray) {
    val n get() = distMm.size
    fun centerDeg(i: Int) = -halfFovDeg + (i + 0.5f) * (2 * halfFovDeg / n)
    fun sectorOf(bearingDeg: Float): Int? {
        if (abs(bearingDeg) > halfFovDeg) return null
        return ((bearingDeg + halfFovDeg) / (2 * halfFovDeg) * n).toInt().coerceIn(0, n - 1)
    }
    fun frontMin(coneDeg: Float = 30f) = distMm.indices.filter { abs(centerDeg(it)) <= coneDeg }.minOfOrNull { distMm[it] } ?: FREE

    companion object {
        const val FREE = 0xFFFF

        fun build(
            cam: CameraModel, pose: CameraPose, boundary: IntArray?, detections: List<Detection>,
            sectors: Int = 9, maxRangeMm: Int = 3000, imgW: Int = cam.width, imgH: Int = cam.height,
            obstacleLabels: (String) -> Boolean = { true },
        ): ObstacleMap {
            val half = cam.hFovDeg / 2
            val d = IntArray(sectors) { FREE }
            val m = ObstacleMap(half, d)
            fun put(gp: GroundPoint?) {
                if (gp == null) return
                val mm = (gp.distM * 1000).toInt()
                if (mm > maxRangeMm) return
                val s = m.sectorOf(gp.bearingDeg) ?: return
                if (mm < d[s]) d[s] = mm
            }
            if (boundary != null) {
                val sx = cam.width.toFloat() / imgW; val sy = cam.height.toFloat() / imgH
                for (x in boundary.indices) if (boundary[x] >= 0) put(Ground.project(cam, pose, (x + 0.5f) * sx, (boundary[x] + 0.5f) * sy))
            }
            for (det in detections) if (obstacleLabels(det.label)) {
                // titik kontak dengan lantai = tengah tepi bawah kotak
                put(Ground.project(cam, pose, det.cx * cam.width, min(det.bottom, 0.999f) * cam.height))
            }
            return m
        }
    }
}
