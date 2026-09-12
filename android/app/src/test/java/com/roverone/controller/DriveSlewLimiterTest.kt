package com.roverone.controller

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class DriveSlewLimiterTest {
    @Test fun accelerationRampsButStopIsImmediate() {
        val limiter = DriveSlewLimiter()
        val first = limiter.step(Drive(linear = .5, lateral = -.5, angular = 1.2), 80)
        assertEquals(.064, first.linear, .0001)
        assertEquals(-.064, first.lateral, .0001)
        assertEquals(.288, first.angular, .0001)
        val second = limiter.step(Drive(linear = .5, lateral = -.5, angular = 1.2), 80)
        assertTrue(second.linear > first.linear)
        assertEquals(Drive(), limiter.step(Drive(), 80))
    }

    @Test fun reversalRestartsFromZeroWithoutOvershoot() {
        val limiter = DriveSlewLimiter()
        limiter.step(Drive(linear = .5), 100)
        val reversed = limiter.step(Drive(linear = -.5), 100)
        assertEquals(-.08, reversed.linear, .0001)
    }
}
