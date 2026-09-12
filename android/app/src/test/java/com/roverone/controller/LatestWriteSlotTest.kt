package com.roverone.controller

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class LatestWriteSlotTest {
    @Test
    fun `busy slot keeps only newest value`() {
        val slot = LatestWriteSlot<String>()

        assertEquals("first", slot.submit("first"))
        assertNull(slot.submit("old waiting"))
        assertNull(slot.submit("newest"))
        assertEquals("newest", slot.complete())
        assertNull(slot.complete())
        assertEquals("after idle", slot.submit("after idle"))
    }

    @Test
    fun `reset drops in flight and queued values`() {
        val slot = LatestWriteSlot<String>()
        slot.submit("first")
        slot.submit("queued")

        slot.reset()

        assertEquals("fresh", slot.submit("fresh"))
        assertNull(slot.complete())
    }
}
