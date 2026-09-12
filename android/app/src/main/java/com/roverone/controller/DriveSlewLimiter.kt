package com.roverone.controller

import kotlin.math.abs

/** Limits acceleration current spikes while every stop remains immediate. */
internal class DriveSlewLimiter(
    private val linearRatePerSecond: Double = 0.8,
    private val lateralRatePerSecond: Double = 0.8,
    private val angularRatePerSecond: Double = 3.6,
) {
    private var current = Drive()

    @Synchronized fun reset() { current = Drive() }

    @Synchronized fun step(target: Drive, elapsedMs: Long): Drive {
        if (target == Drive()) {
            current = Drive()
            return current
        }
        val seconds = elapsedMs.coerceIn(1L, 200L) / 1000.0
        current = Drive(
            approach(current.linear, target.linear, linearRatePerSecond * seconds),
            approach(current.angular, target.angular, angularRatePerSecond * seconds),
            approach(current.lateral, target.lateral, lateralRatePerSecond * seconds),
        )
        return current
    }

    private fun approach(previous: Double, target: Double, step: Double): Double {
        if (target == 0.0) return 0.0
        val start = if (previous * target < 0.0) 0.0 else previous
        val difference = target - start
        return if (abs(difference) <= step) target else start + if (difference > 0.0) step else -step
    }
}
