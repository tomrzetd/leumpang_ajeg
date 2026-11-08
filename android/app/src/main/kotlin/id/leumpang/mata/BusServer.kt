// Leumpang Bus untuk perangkat lain: WebSocket ws://<ip-HP>:8765 (JSON, lihat docs/BUS.md).
package id.leumpang.mata

import id.leumpang.bus.Bus
import id.leumpang.bus.BusSession
import id.leumpang.bus.J
import id.leumpang.bus.Module
import id.leumpang.bus.ModuleHost
import id.leumpang.bus.ModuleInfo
import org.java_websocket.WebSocket
import org.java_websocket.handshake.ClientHandshake
import org.java_websocket.server.WebSocketServer
import java.net.Inet4Address
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.util.concurrent.ConcurrentHashMap

class BusServer(private val host: ModuleHost, private val settings: Settings, private val log: (String) -> Unit) : Module {
    override val info = ModuleInfo("bus-ws", "Leumpang Bus WebSocket :$PORT untuk studio PC, skrip, dan board sensor", listOf("app/bus"), emptyList())
    private var server: WebSocketServer? = null
    private val sessions = ConcurrentHashMap<WebSocket, BusSession>()
    val clients get() = sessions.size

    override fun start(bus: Bus) {
        val s = object : WebSocketServer(InetSocketAddress(PORT)) {
            override fun onOpen(conn: WebSocket, hs: ClientHandshake) {
                sessions[conn] = BusSession(bus, host, { txt -> if (conn.isOpen) conn.send(txt) }, settings.busToken.ifBlank { null })
                publish(bus)
            }
            override fun onClose(conn: WebSocket, code: Int, reason: String?, remote: Boolean) { sessions.remove(conn)?.close(); publish(bus) }
            override fun onMessage(conn: WebSocket, message: String) { sessions[conn]?.onText(message) }
            override fun onError(conn: WebSocket?, ex: Exception) { log("bus-ws: ${ex.message}") }
            override fun onStart() { log("Leumpang Bus: ws://${ip()}:$PORT"); publish(bus) }
        }
        s.isReuseAddr = true
        s.connectionLostTimeout = 10
        s.start()
        server = s
    }

    private fun publish(bus: Bus) = bus.publish("app/bus", J.obj("url" to "ws://${ip()}:$PORT", "clients" to sessions.size,
        "auth" to settings.busToken.isNotBlank()), "bus-ws")

    override fun stop() { try { server?.stop(500) } catch (_: Exception) {}; server = null; sessions.clear() }

    companion object {
        const val PORT = 8765
        fun ip(): String = try {
            NetworkInterface.getNetworkInterfaces().toList().flatMap { it.inetAddresses.toList() }
                .firstOrNull { !it.isLoopbackAddress && it is Inet4Address }?.hostAddress ?: "?"
        } catch (_: Exception) { "?" }
    }
}
