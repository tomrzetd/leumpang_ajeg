package id.leumpang.mata

import android.Manifest
import android.app.Application
import android.content.pm.PackageManager
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import id.leumpang.lap.Lap
import id.leumpang.perception.ObstacleMap
import id.leumpang.perception.Request
import kotlinx.coroutines.delay

class MataApp : Application() {
    lateinit var engine: Engine
    override fun onCreate() { super.onCreate(); engine = Engine(this) }
}

private val Orange = Color(0xFFE86A1C)
private val Panel = Color(0xFF262B31)

class MainActivity : ComponentActivity() {
    private val engine get() = (application as MataApp).engine

    private val perms = registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { res ->
        if (res[Manifest.permission.CAMERA] == true) engine.startCamera(this)
        if (res[Manifest.permission.RECORD_AUDIO] == true && engine.settings.speech) engine.speech.enableListening(true)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val need = listOf(Manifest.permission.CAMERA, Manifest.permission.RECORD_AUDIO)
            .filter { ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (need.isEmpty()) engine.startCamera(this) else perms.launch(need.toTypedArray())
        setContent { MaterialTheme(colorScheme = darkColorScheme(primary = Orange, surface = Panel)) { Screen(engine) { engine.startCamera(this) } } }
    }

    override fun onResume() {
        super.onResume()
        // board robot dicolok saat aplikasi terbuka (intent USB_DEVICE_ATTACHED) -> sambung otomatis
        if (!engine.robot.connected && engine.robot.usbDevices().isNotEmpty()) engine.robot.connectUsb()
    }
}

@Composable
private fun Screen(e: Engine, restartCamera: () -> Unit) {
    var tick by remember { mutableIntStateOf(0) }
    LaunchedEffect(Unit) { while (true) { delay(150); tick++ } }
    Row(Modifier.fillMaxSize().background(Color(0xFF1D2126))) {
        Box(Modifier.weight(1.25f).fillMaxHeight().padding(6.dp)) { CameraView(e, tick) }
        Column(Modifier.weight(1f).fillMaxHeight().verticalScroll(rememberScrollState()).padding(6.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)) {
            // tick ikut sebagai parameter: Compose (strong skipping) tidak melewatkan kartu yang membaca state Engine
            RobotCard(e, tick); BehaviorCard(e, tick); SensesCard(e, tick); SettingsCard(e, tick, restartCamera); LogCard(e, tick)
        }
    }
}

@Composable
private fun CameraView(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int) {
    val bmp = e.lastFrame
    Box(Modifier.fillMaxSize().background(Color.Black)) {
        if (bmp != null) Image(bmp.asImageBitmap(), null, Modifier.fillMaxSize(), contentScale = ContentScale.Fit)
        else Text("menunggu kamera...", Modifier.align(Alignment.Center), color = Color.Gray)
        if (bmp != null) Canvas(Modifier.fillMaxSize()) {
            // area gambar dalam kotak (ContentScale.Fit)
            val s = minOf(size.width / bmp.width, size.height / bmp.height)
            val w = bmp.width * s; val h = bmp.height * s
            val ox = (size.width - w) / 2; val oy = (size.height - h) / 2
            // garis batas rintangan
            e.obstacles.lastBoundary?.let { b ->
                for (i in b.indices) if (b[i] >= 0) drawCircle(Color(0xFFFF5252), 2.5f, Offset(ox + (i + 0.5f) / b.size * w, oy + b[i] * h))
            }
            drawLine(Color(0x664FC3F7), Offset(ox, oy + e.obstacles.horizon / 120f * h), Offset(ox + w, oy + e.obstacles.horizon / 120f * h), 2f)
            for (d in e.vision.lastObjects) {
                val c = if (d.label == "person") Color(0xFF4FC3F7) else Color(0xFFFFC107)
                drawRect(c, Offset(ox + d.left * w, oy + d.top * h), Size((d.right - d.left) * w, (d.bottom - d.top) * h), style = Stroke(3f))
            }
            e.obstacles.lastPerson?.let { p ->
                val d = p.box
                drawRect(Orange, Offset(ox + d.left * w, oy + d.top * h), Size((d.right - d.left) * w, (d.bottom - d.top) * h), style = Stroke(7f))
            }
            for (f in e.vision.lastFaces) drawRect(Color(0xFF9CCC65), Offset(ox + f.box.left * w, oy + f.box.top * h),
                Size(f.box.width() * w, f.box.height() * h), style = Stroke(3f))
            // peta sektor di bawah: hijau jauh, merah dekat
            e.obstacles.lastMap?.let { m ->
                val bw = w / m.n
                for (i in 0 until m.n) {
                    val mm = m.distMm[m.n - 1 - i]                  // sektor 0 = kanan -> gambar kanan
                    val col = when { mm < 350 -> Color(0xDDD9534F); mm < 900 -> Color(0xDDF0AD4E); mm < ObstacleMap.FREE -> Color(0xDD8BC34A); else -> Color(0x553FA34D) }
                    drawRect(col, Offset(ox + i * bw + 2, oy + h - 18), Size(bw - 4f, 14f))
                }
            }
        }
        Column(Modifier.align(Alignment.TopStart).padding(8.dp)) {
            val labels = e.vision.lastObjects.joinToString { "${it.label} ${(it.score * 100).toInt()}%" }
            Text(labels, color = Color.White, fontSize = 13.sp)
            e.vision.lastGesture?.let { Text("isyarat: ${it.first} ${(it.second * 100).toInt()}%", color = Color(0xFF4FC3F7), fontSize = 13.sp) }
            e.vision.lastFaces.filter { it.name.isNotEmpty() }.forEach { Text("wajah: ${it.name}", color = Color(0xFF9CCC65), fontSize = 13.sp) }
            Text("${e.camera.active?.label ?: "-"}  ·  ${"%.0f".format(e.camera.fps)} fps  ·  ${"%.0f".format(e.frameMs)} ms/frame",
                color = Color.LightGray, fontSize = 11.sp)
        }
    }
}

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(10.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text(title, fontWeight = FontWeight.Bold, color = Orange)
            content()
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun RobotCard(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int) = Section("Robot") {
    val t = e.robot.client.telemetry; val h = e.robot.client.hello
    var host by remember { mutableStateOf(e.settings.tcpHost) }
    Text(if (e.robot.connected) "${e.robot.label} · ${h?.name ?: "menunggu HELLO"}" else "terputus", fontSize = 13.sp)
    if (t != null) Text("mode ${t.modeName} (${t.mode}) · roll ${"%.1f".format(t.rollDeg)}° pitch ${"%.1f".format(t.pitchDeg)}° · servo ${t.fbOk}/22" +
        (if (t.obstacleBlock) " · MAJU DIBLOKIR" else ""), fontSize = 13.sp, color = if (t.obstacleBlock) Color(0xFFF0AD4E) else Color.Unspecified)
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        Button(onClick = { e.robot.connectUsb() }) { Text("USB-OTG") }
        OutlinedTextField(host, { host = it }, Modifier.width(150.dp), singleLine = true, label = { Text("IP Wi-Fi") })
        Button(onClick = { e.settings.tcpHost = host; e.robot.connectTcp(host) }) { Text("Wi-Fi") }
        if (e.robot.connected) OutlinedButton(onClick = { e.robot.disconnect() }) { Text("Putus") }
    }
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        OutlinedButton(onClick = { e.robot.cmd(Lap.CMD_RESET) }) { Text("HOME") }
        OutlinedButton(onClick = { e.robot.cmd(Lap.CMD_GO) }) { Text("Seimbang") }
        OutlinedButton(onClick = { e.robot.setParam(Lap.P_GAIT_ENABLE, 1f) }) { Text("gaitEnable=1") }
        Button(onClick = { e.robot.cmd(Lap.CMD_STOP) }, colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF8B2E2B))) { Text("LEPAS SERVO") }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun BehaviorCard(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int) = Section("Perilaku · ${e.behavior.behavior.activity.name.lowercase()}") {
    Text(e.behavior.lastReason, fontSize = 13.sp)
    val front = e.obstacles.lastMap?.frontMin() ?: ObstacleMap.FREE
    Text("rintangan depan: ${if (front >= ObstacleMap.FREE) "bebas" else "$front mm"}" +
        (e.obstacles.lastPerson?.let { " · orang ${it.distMm} mm, ${"%.0f".format(it.bearingDeg)}°" } ?: ""), fontSize = 13.sp)
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
        Button(onClick = { e.behavior.submit(Request.Wander) }) { Text("Jelajah") }
        Button(onClick = { e.behavior.submit(Request.Follow) }) { Text("Ikuti saya") }
        Button(onClick = { e.behavior.submit(Request.Stop) }, colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF8B2E2B))) { Text("Berhenti") }
        OutlinedButton(onClick = { e.behavior.submit(Request.Wave) }) { Text("Lambai") }
        OutlinedButton(onClick = { e.behavior.submit(Request.Describe) }) { Text("Lihat apa?") }
    }
    Text("Isyarat: telapak terbuka/kepal = berhenti · jempol = ikuti · telunjuk ke atas = jelajah · V = lambai · jempol bawah = HOME",
        fontSize = 11.sp, color = Color.Gray)
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
private fun SensesCard(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int) = Section("Indera") {
    var name by remember { mutableStateOf("") }
    var msg by remember { mutableStateOf("") }
    Text("model AI: ${e.vision.status}", fontSize = 13.sp)
    Toggle("Isyarat tangan", e.settings.gestures) { e.settings.gestures = it }
    Toggle("Wajah", e.settings.faces) { e.settings.faces = it }
    Toggle("Dengar perintah suara", e.settings.speech) { e.settings.speech = it; e.speech.enableListening(it) }
    Toggle("Wajib kata bangun (\"robot ...\")", e.settings.requireWake) { e.settings.requireWake = it }
    Toggle("Robot bicara (TTS)", e.settings.tts) { e.settings.tts = it }
    Text("suara: ${e.speech.status}" + (if (e.speech.lastHeard.isNotEmpty()) " · \"${e.speech.lastHeard}\"" else ""), fontSize = 12.sp)
    FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalArrangement = Arrangement.Center) {
        OutlinedTextField(name, { name = it }, Modifier.width(150.dp), singleLine = true, label = { Text("nama wajah") })
        Button(onClick = { if (name.isNotBlank()) msg = e.vision.enrollFace(name.trim()) }) { Text("Daftarkan") }
        OutlinedButton(onClick = { e.vision.forgetFaces(); msg = "daftar wajah dihapus" }) { Text("Hapus") }
    }
    Text(msg.ifEmpty { "dikenal: ${e.vision.knownNames().joinToString().ifEmpty { "-" }}" }, fontSize = 12.sp)
}

@Composable
private fun SettingsCard(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int, restartCamera: () -> Unit) = Section("Pengaturan") {
    var open by remember { mutableStateOf(false) }
    val cams = remember { e.camera.list() }
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text("Kamera: ", fontSize = 13.sp)
        Box {
            OutlinedButton(onClick = { open = true }) { Text(e.camera.active?.label ?: "pilih") }
            DropdownMenu(open, { open = false }) {
                cams.forEach { c -> DropdownMenuItem(text = { Text(c.label) }, onClick = { open = false; e.settings.cameraId = c.id; restartCamera() }) }
            }
        }
    }
    if (cams.none { it.facing.startsWith("USB") }) Text("Kamera USB-UVC tidak terlihat oleh Camera2 di HP ini (lihat README).", fontSize = 11.sp, color = Color.Gray)
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text("Rotasi tambahan: ${e.settings.extraRotation}°", fontSize = 13.sp)
        Spacer(Modifier.width(8.dp))
        OutlinedButton(onClick = { e.settings.extraRotation = (e.settings.extraRotation + 90) % 360; restartCamera() }) { Text("+90°") }
    }
    LabeledSlider("Kemiringan kamera ke bawah", e.settings.mountPitchDeg, 0f..45f, "°") { e.settings.mountPitchDeg = it }
    LabeledSlider("Tinggi lensa di atas sumbu kaki", e.settings.camOffsetMm, 150f..320f, " mm") { e.settings.camOffsetMm = it }
    LabeledSlider("Jarak henti (aplikasi)", e.settings.obstacleStopMm.toFloat(), 250f..800f, " mm") { e.settings.obstacleStopMm = it.toInt() }
    Toggle("Leumpang Bus WebSocket :${BusServer.PORT}", e.settings.busServer) { e.setBusServer(it) }
    if (e.settings.busServer) Text("ws://${BusServer.ip()}:${BusServer.PORT} · ${e.busServer.clients} klien", fontSize = 12.sp, fontFamily = FontFamily.Monospace)
}

@Composable
private fun LogCard(e: Engine, @Suppress("UNUSED_PARAMETER") tick: Int) = Section("Log") {
    Column { e.logTail(14).forEach { Text(it, fontSize = 11.sp, fontFamily = FontFamily.Monospace, color = Color.LightGray) } }
}

@Composable
private fun Toggle(label: String, value: Boolean, onChange: (Boolean) -> Unit) {
    var v by remember { mutableStateOf(value) }
    Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.fillMaxWidth().height(36.dp)) {
        Text(label, Modifier.weight(1f), fontSize = 13.sp)
        Switch(v, { v = it; onChange(it) })
    }
}

@Composable
private fun LabeledSlider(label: String, value: Float, range: ClosedFloatingPointRange<Float>, unit: String, onChange: (Float) -> Unit) {
    var v by remember { mutableStateOf(value) }
    Column {
        Text("$label: ${"%.0f".format(v)}$unit", fontSize = 13.sp)
        Slider(v, { v = it }, valueRange = range, onValueChangeFinished = { onChange(v) })
    }
}
