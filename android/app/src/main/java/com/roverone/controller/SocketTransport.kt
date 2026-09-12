package com.roverone.controller

import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import okio.ByteString

/** One connection per instance; callback threading is owned by the caller. */
interface TextTransport {
    fun send(text: String): Boolean
    fun close()
    fun cancel()
}

class SocketTransport(
    private val client: OkHttpClient,
    private val onText: (String) -> Unit,
    private val onEnded: (String) -> Unit
) : TextTransport {
    @Volatile private var socket: WebSocket? = null
    @Volatile private var open = false
    fun connect(endpoint: Endpoint) {
        val request = Request.Builder().url(endpoint.webSocket)
            .header("Origin", endpoint.origin).build()
        socket = client.newWebSocket(request, object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) { open = true }
            override fun onMessage(webSocket: WebSocket, text: String) { onText(text) }
            override fun onMessage(webSocket: WebSocket, bytes: ByteString) {
                webSocket.cancel()
                onEnded("网关发送了不支持的二进制消息")
            }
            override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
                open = false
                webSocket.close(code, reason)
                onEnded(closeReason(code, reason))
            }
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
                open = false
                onEnded("连接失败：" + (t.message ?: t.javaClass.simpleName))
            }
        })
    }
    /** Never append commands behind stalled motion; caller cancels and locks on false. */
    @Synchronized override fun send(text: String): Boolean {
        val ws = socket ?: return false
        return open && ws.queueSize() == 0L && ws.send(text)
    }
    override fun close() { open = false; socket?.close(1000, "controller stopped") }
    override fun cancel() { open = false; socket?.cancel(); socket = null }
    companion object {
        fun closeReason(code: Int, reason: String) = when (code) {
            4403 -> "来源被拒绝：请核对网关 ROVER_ALLOWED_ORIGINS"
            4409 -> "控制权被占用：请先关闭其他控制网页或 App"
            else -> "连接已关闭 ($code) " + reason.take(120)
        }
    }
}
