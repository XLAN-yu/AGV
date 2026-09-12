package com.roverone.controller

/**
 * A one-in-flight, one-latest-value slot for callback-based transports.
 * New values replace the waiting value, so stale motion commands never queue.
 */
internal class LatestWriteSlot<T : Any> {
    private var inFlight = false
    private var latest: T? = null

    @Synchronized
    fun submit(value: T): T? {
        if (inFlight) {
            latest = value
            return null
        }
        inFlight = true
        return value
    }

    @Synchronized
    fun complete(): T? {
        val next = latest
        latest = null
        if (next == null) inFlight = false
        return next
    }

    @Synchronized
    fun reset() {
        inFlight = false
        latest = null
    }
}
