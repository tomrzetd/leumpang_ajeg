// Modul tautan robot: LAP lewat USB-OTG (CDC-ACM / CP210x / CH340) atau TCP Wi-Fi (ESP32).
// Menerbitkan robot/* ke Leumpang Bus dan meneruskan robot/ext/out ke robot (EXT).
package id.leumpang.mata

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbManager
import android.os.Build
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import id.leumpang.bus.Bus
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleInfo
import id.leumpang.bus.Subscription
import id.leumpang.lap.Ack
import id.leumpang.lap.Lap
import id.leumpang.lap.LapClient
import id.leumpang.lap.Perception
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

class RobotLinkModule(private val ctx: Context, private val log: (String) -> Unit) : Module {
    override val info = ModuleInfo(
        "robot", "Tautan LAP ke platform leumpang-ajeg (USB-OTG / Wi-Fi)",
        listOf("robot/link", "robot/hello", "robot/telemetry", "robot/log", "robot/ext/+"),
        listOf("robot/ext/out"),
    )

    @Volatile var label = "terputus"; private set
    @Volatile private var writer: ((ByteArray) -> Unit)? = null
    val client = LapClient { b -> writer?.invoke(b) }
    private var bus: Bus? = null
    private var extSub: Subscription? = null
    private val timer = Executors.newSingleThreadScheduledExecutor()
    private var usbPort: UsbSerialPort? = null
    private var usbIo: SerialInputOutputManager? = null
    private var socket: Socket? = null
    private var lastTele = 0L

    override fun start(bus: Bus) {
        this.bus = bus
        client.addListener(object : LapClient.Listener {
            override fun onPacket(type: Int) {
                when (type) {
                    Lap.HELLO -> client.hello?.let { h ->
                        bus.publish("robot/hello", J.obj("name" to h.name, "board" to h.board, "caps" to h.caps,
                            "fw" to "${h.fw.first}.${h.fw.second}.${h.fw.third}"), "robot")
                    }
                    Lap.TELEMETRY -> client.telemetry?.let { t ->
                        val now = System.currentTimeMillis()
                        if (now - lastTele >= 100) {                       // bus: maks 10 Hz
                            lastTele = now
                            bus.publish("robot/telemetry", J.obj("mode" to t.mode, "modeName" to t.modeName,
                                "roll" to t.rollDeg, "pitch" to t.pitchDeg, "autoH" to t.autoH, "flags" to t.flags,
                                "obstacleBlock" to t.obstacleBlock, "fbOk" to t.fbOk, "loopUs" to t.loopAvgUs), "robot", retain = true)
                        }
                    }
                }
            }
            override fun onLog(line: String) { bus.publish("robot/log", J.obj("line" to line.trim()), "robot", retain = false); log("robot: ${line.trim()}") }
            override fun onExt(topic: Int, data: ByteArray) {
                bus.publish("robot/ext/%04x".format(topic), J.obj("topic" to topic, "hex" to data.joinToString("") { "%02x".format(it) },
                    "text" to String(data, Charsets.UTF_8)), "robot", retain = false)
            }
            override fun onAck(ack: Ack) { if (ack.status != Lap.ACK_OK) log("ACK 0x%02x status %d".format(ack.type, ack.status)) }
        })
        extSub = bus.subscribe("robot/ext/out", "robot") { m ->
            val topic = J.num(m.data, "topic")?.toInt() ?: return@subscribe
            val hex = J.str(m.data, "hex")
            val data = hex?.chunked(2)?.map { it.toInt(16).toByte() }?.toByteArray() ?: (J.str(m.data, "text") ?: "").toByteArray()
            if (connected) client.ext(topic, data)
        }
        timer.scheduleAtFixedRate({ if (connected) client.ping(); publishLink() }, 1, 1, TimeUnit.SECONDS)
    }

    val connected get() = writer != null
    val alive get() = connected && client.alive()

    private fun publishLink() = bus?.publish("robot/link", J.obj("connected" to connected, "alive" to alive, "label" to label,
        "rx" to client.rxPackets, "crcErrors" to client.decoder.crcErrors), "robot")

    private fun attached(lbl: String, w: (ByteArray) -> Unit) {
        writer = w; label = lbl
        client.begin(teleHz = 20, servoHz = 2, gaitHz = 10, name = "leumpang-mata")
        log("terhubung: $lbl"); publishLink()
    }

    fun disconnect() {
        writer = null
        try { usbIo?.stop() } catch (_: Exception) {}
        try { usbPort?.close() } catch (_: Exception) {}
        try { socket?.close() } catch (_: Exception) {}
        usbIo = null; usbPort = null; socket = null
        label = "terputus"; publishLink()
    }

    // ---- USB-OTG -------------------------------------------------------------
    fun usbDevices(): List<String> {
        val um = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
        return UsbSerialProber.getDefaultProber().findAllDrivers(um).map {
            "%s %04x:%04x".format(it.javaClass.simpleName.removeSuffix("SerialDriver"), it.device.vendorId, it.device.productId)
        }
    }

    fun connectUsb(baud: Int = 115200) {
        disconnect()
        val um = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
        val drv = UsbSerialProber.getDefaultProber().findAllDrivers(um).firstOrNull()
        if (drv == null) { log("USB: tidak ada board robot di OTG"); return }
        if (!um.hasPermission(drv.device)) {
            val action = "id.leumpang.mata.USB_PERMISSION"
            val flags = if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE else 0
            val pi = PendingIntent.getBroadcast(ctx, 0, Intent(action).setPackage(ctx.packageName), flags)
            val rx = object : BroadcastReceiver() {
                override fun onReceive(c: Context, i: Intent) {
                    ctx.unregisterReceiver(this)
                    if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) connectUsb(baud) else log("USB: izin ditolak")
                }
            }
            if (Build.VERSION.SDK_INT >= 33) ctx.registerReceiver(rx, IntentFilter(action), Context.RECEIVER_NOT_EXPORTED)
            else @Suppress("UnspecifiedRegisterReceiverFlag") ctx.registerReceiver(rx, IntentFilter(action))
            um.requestPermission(drv.device, pi)
            log("USB: meminta izin...")
            return
        }
        val conn = um.openDevice(drv.device) ?: run { log("USB: gagal membuka perangkat"); return }
        val port = drv.ports[0]
        try {
            port.open(conn)
            port.setParameters(baud, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            try { port.dtr = true; port.rts = true } catch (_: Exception) {}
        } catch (e: Exception) { log("USB: ${e.message}"); return }
        usbPort = port
        usbIo = SerialInputOutputManager(port, object : SerialInputOutputManager.Listener {
            override fun onNewData(data: ByteArray) = client.feed(data)
            override fun onRunError(e: Exception) { log("USB terputus: ${e.message}"); disconnect() }
        }).also { it.start() }
        attached("USB %04x:%04x".format(drv.device.vendorId, drv.device.productId)) { b ->
            try { port.write(b, 200) } catch (e: Exception) { log("USB tulis: ${e.message}") }
        }
    }

    // ---- TCP (Wi-Fi ESP32-S3/P4) ------------------------------------------------
    fun connectTcp(host: String, port: Int = Lap.TCP_PORT) {
        disconnect()
        Thread({
            try {
                val s = Socket(); s.tcpNoDelay = true
                s.connect(InetSocketAddress(host, port), 3000)
                socket = s
                val out = s.getOutputStream()
                attached("Wi-Fi $host:$port") { b -> try { synchronized(out) { out.write(b); out.flush() } } catch (_: Exception) {} }
                val inp = s.getInputStream(); val buf = ByteArray(4096)
                while (true) { val n = inp.read(buf); if (n < 0) break; client.feed(buf, n) }
            } catch (e: Exception) { log("TCP $host: ${e.message}") }
            if (socket != null) disconnect()
        }, "lap-tcp").start()
    }

    // ---- aksi (dipakai modul perilaku & persepsi) --------------------------------
    fun gait(vx: Float, yaw: Float, ttl: Int) { if (connected) client.gaitIntent(vx, yaw, ttl) }
    fun cmd(id: Int) { if (connected) client.cmd(id) }
    fun motion(slot: Int) { if (connected) client.motionPlay(slot, true) }
    fun perception(p: Perception) { if (connected && (client.hello?.has(Lap.CAP_PERCEPTION) == true)) client.perception(p) }
    fun setParam(id: Int, v: Float) { if (connected) client.setParam(id, v) }

    override fun stop() { extSub?.cancel(); timer.shutdownNow(); disconnect() }
}
