// Klien LAP sisi HP: transport apa pun (USB-OTG serial, TCP Wi-Fi) cukup memberi `write` dan memanggil feed().
package id.leumpang.lap

class LapClient(private val write: (ByteArray) -> Unit) {
    interface Listener {
        fun onPacket(type: Int) {}
        fun onLog(line: String) {}
        fun onExt(topic: Int, data: ByteArray) {}
        fun onAck(ack: Ack) {}
    }

    private val dec = Decoder()
    private var seq = 0
    private val listeners = java.util.concurrent.CopyOnWriteArrayList<Listener>()

    @Volatile var hello: Hello? = null; private set
    @Volatile var telemetry: Telemetry? = null; private set
    @Volatile var servo: ServoState? = null; private set
    @Volatile var gait: GaitState? = null; private set
    @Volatile var config: Config? = null; private set
    @Volatile var motions: List<MotionSlot> = emptyList(); private set
    @Volatile var lastRxMs = 0L; private set
    @Volatile var rxPackets = 0L; private set

    fun addListener(l: Listener) { listeners += l }
    fun removeListener(l: Listener) { listeners -= l }
    val decoder get() = dec

    fun alive(nowMs: Long = System.currentTimeMillis()) = telemetry != null && nowMs - lastRxMs < 3000

    @Synchronized private fun send(type: Int, body: ByteArray = ByteArray(0)) {
        write(Lap.encodeFrame(type, seq, body))
        seq = (seq + 1) and 0xFF
    }

    /** 0x00 (pindah mode LAP di port serial bersama) + HELLO + STREAM_CFG */
    fun begin(teleHz: Int = 20, servoHz: Int = 5, gaitHz: Int = 10, name: String = "leumpang-mata") {
        write(byteArrayOf(0))
        send(Lap.HELLO, Hello(role = Lap.ROLE_PHONE, name = name).enc())
        send(Lap.STREAM_CFG, Msg.streamCfg(teleHz, servoHz, gaitHz, true))
        cmd(Lap.CMD_MOTION_LIST)
    }

    fun feed(data: ByteArray, len: Int = data.size) {
        for (i in 0 until len) {
            val p = dec.feed(data[i]) ?: continue
            handle(p)
        }
    }

    private fun handle(p: Packet) {
        rxPackets++
        lastRxMs = System.currentTimeMillis()
        when (p.type) {
            Lap.HELLO -> hello = Hello.dec(p.body) ?: hello
            Lap.TELEMETRY -> telemetry = Telemetry.dec(p.body) ?: telemetry
            Lap.SERVO_STATE -> servo = ServoState.dec(p.body) ?: servo
            Lap.GAIT_STATE -> gait = GaitState.dec(p.body) ?: gait
            Lap.CONFIG -> config = Config.dec(p.body) ?: config
            Lap.MOTION_STATUS -> motions = decodeMotionStatus(p.body) ?: motions
            Lap.LOG -> { val s = String(p.body, Charsets.UTF_8); listeners.forEach { it.onLog(s) } }
            Lap.ACK -> Ack.dec(p.body)?.let { a -> listeners.forEach { it.onAck(a) } }
            Lap.EXT -> if (p.body.size >= 2) {
                val topic = (p.body[0].toInt() and 0xFF) or ((p.body[1].toInt() and 0xFF) shl 8)
                val d = p.body.copyOfRange(2, p.body.size)
                listeners.forEach { it.onExt(topic, d) }
            }
        }
        listeners.forEach { it.onPacket(p.type) }
    }

    fun ping() = send(Lap.PING)
    fun cmd(id: Int) = send(Lap.CMD, Msg.cmd(id))
    fun setParam(id: Int, v: Float) = send(Lap.SET_PARAM, Msg.setParam(id, v))
    fun setTrim(bus: Int, slot: Int, v: Int) = send(Lap.SET_TRIM, Msg.setTrim(bus, slot, v))
    fun gaitIntent(vxMm: Float, yawDeg: Float, ttlMs: Int) = send(Lap.GAIT_INTENT, Msg.gaitIntent(vxMm, 0f, yawDeg, ttlMs))
    fun perception(p: Perception) = send(Lap.PERCEPTION, p.enc())
    fun motionPlay(slot: Int, play: Boolean = true) = send(Lap.MOTION_PLAY, Msg.motionPlay(slot, play))
    fun poseTarget(src: Array<IntArray>, mask: Long, ttlMs: Int) = send(Lap.POSE_TARGET, Msg.poseTarget(src, mask, ttlMs))
    fun ext(topic: Int, data: ByteArray) = send(Lap.EXT, Msg.ext(topic, data))
    fun simImu(rollDeg: Float, pitchDeg: Float) = send(Lap.SIM_IMU, Msg.simImu(rollDeg, pitchDeg))
}
