package com.roverone.controller

import org.junit.Assert.*
import org.junit.Test

class BleFrameCodecTest {
    @Test fun unicodeRoundTripAcrossFragments() {
        val source = """{"type":"robot_status","text":"${"遥测".repeat(120)}"}"""
        val frames = BleFrameCodec.encode(source, 64)
        assertTrue(frames.size > 1)
        val assembler = BleFrameAssembler()
        val values = frames.map(assembler::accept)
        assertTrue(values.dropLast(1).all { it == null })
        assertEquals(source, values.last())
    }

    @Test fun missingFragmentRejected() {
        val frames = BleFrameCodec.encode("x".repeat(300), 50)
        val assembler = BleFrameAssembler()
        assertNull(assembler.accept(frames[0]))
        assertThrows(IllegalArgumentException::class.java) { assembler.accept(frames[2]) }
    }

    @Test fun malformedHeaderRejected() {
        assertThrows(IllegalArgumentException::class.java) { BleFrameAssembler().accept(byteArrayOf(1, 2)) }
    }
}
