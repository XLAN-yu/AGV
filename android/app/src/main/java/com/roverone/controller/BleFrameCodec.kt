package com.roverone.controller

import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.atomic.AtomicInteger

/** BLE-only fragmentation. Reassembled payloads remain the exact gateway JSON protocol. */
object BleFrameCodec {
    const val VERSION: Byte = 1
    const val HEADER_SIZE = 5
    private val ids = AtomicInteger(0)

    fun encode(text: String, mtu: Int = 247): List<ByteArray> {
        val payload = text.toByteArray(Charsets.UTF_8)
        val capacity = (mtu - 3 - HEADER_SIZE).coerceAtLeast(1)
        val chunks = ((payload.size + capacity - 1) / capacity).coerceAtLeast(1)
        require(chunks <= 255) { "BLE 消息过大" }
        val id = ids.updateAndGet { (it + 1) and 0xFFFF }
        return List(chunks) { index ->
            val from = index * capacity
            val to = minOf(payload.size, from + capacity)
            ByteBuffer.allocate(HEADER_SIZE + to - from).order(ByteOrder.LITTLE_ENDIAN)
                .put(VERSION).putShort(id.toShort()).put(index.toByte()).put(chunks.toByte())
                .put(payload, from, to - from).array()
        }
    }
}

class BleFrameAssembler {
    private var messageId = -1
    private var nextIndex = 0
    private var chunkCount = 0
    private var output = ByteArrayOutputStream()

    @Synchronized fun accept(frame: ByteArray): String? {
        require(frame.size >= BleFrameCodec.HEADER_SIZE) { "BLE 分片过短" }
        val header = ByteBuffer.wrap(frame).order(ByteOrder.LITTLE_ENDIAN)
        require(header.get() == BleFrameCodec.VERSION) { "BLE 分片版本不支持" }
        val id = header.short.toInt() and 0xFFFF
        val index = header.get().toInt() and 0xFF
        val count = header.get().toInt() and 0xFF
        require(count > 0 && index < count) { "BLE 分片序号无效" }
        if (id != messageId) {
            require(index == 0) { "BLE 首分片缺失" }
            messageId = id; nextIndex = 0; chunkCount = count; output = ByteArrayOutputStream()
        }
        require(count == chunkCount && index == nextIndex) { "BLE 分片不连续" }
        output.write(frame, BleFrameCodec.HEADER_SIZE, frame.size - BleFrameCodec.HEADER_SIZE)
        nextIndex++
        if (nextIndex != chunkCount) return null
        val text = output.toByteArray().toString(Charsets.UTF_8)
        messageId = -1; nextIndex = 0; chunkCount = 0; output = ByteArrayOutputStream()
        return text
    }
}
