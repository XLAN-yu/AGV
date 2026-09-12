package com.roverone.controller

import okhttp3.OkHttpClient
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okhttp3.mockwebserver.MockResponse
import okhttp3.mockwebserver.MockWebServer
import org.junit.Assert.*
import org.junit.Test
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit

class SocketTransportTest {
    @Test fun realWebSocketHandshakeAndThreeCommandRoundTrip() {
        val received = LinkedBlockingQueue<String>()
        val connected = CountDownLatch(1)
        val client = OkHttpClient()
        MockWebServer().use { server ->
            server.enqueue(MockResponse().withWebSocketUpgrade(object : WebSocketListener() {
                override fun onOpen(webSocket: WebSocket, response: Response) {
                    webSocket.send("""{"type":"robot_status","event":"connected"}""")
                }
                override fun onMessage(webSocket: WebSocket, text: String) {
                    received.offer(text)
                }
                override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                    webSocket.close(code, reason)
                }
            }))
            server.start()
            val ep = Endpoint.parse(server.url("/").toString())
            val transport = SocketTransport(client, { connected.countDown() }, {})
            try {
                transport.connect(ep)
                assertTrue(connected.await(3, TimeUnit.SECONDS))
                val request = server.takeRequest(3, TimeUnit.SECONDS)!!
                assertEquals("/ws", request.path)
                assertEquals(ep.origin, request.getHeader("Origin"))
                for (message in listOf(Protocol.drive(1, .2, -.3), Protocol.estop(2), Protocol.clear(3))) {
                    assertTrue(transport.send(message))
                    assertEquals(message, received.poll(3, TimeUnit.SECONDS))
                }
                transport.close()
                assertFalse(transport.send(Protocol.drive(4, 1.0, 0.0)))
            } finally {
                transport.cancel()
                client.dispatcher.executorService.shutdown()
                client.connectionPool.evictAll()
            }
        }
    }
    @Test fun closeReasonsExplainAccessAndControllerConflicts() {
        assertTrue(SocketTransport.closeReason(4403, "").contains("ROVER_ALLOWED_ORIGINS"))
        assertTrue(SocketTransport.closeReason(4409, "").contains("占用"))
    }
}
