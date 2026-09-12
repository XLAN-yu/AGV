package com.roverone.controller

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

class PathingTest {
    private val metresFor156Counts = 2.0 * PI * 0.0325 / 10.0

    @Test fun encoderAndImuFusionIntegratesForwardFromRelativeOrigin() {
        val estimator = FuzzyPoseEstimator()
        estimator.ingest(Telemetry(yaw = 1.1, encoders = listOf(0, 0, 0, 0)))
        val pose = estimator.ingest(Telemetry(yaw = 1.1, encoders = listOf(156, 156, 156, 156)))

        assertTrue(pose.ready)
        assertEquals(metresFor156Counts, pose.xMetres, 0.0001)
        assertEquals(0.0, pose.yMetres, 0.0001)
        assertEquals(0.0, pose.headingRad, 0.0001)
        assertTrue(pose.confidence > 0.4)
    }

    @Test fun mecanumEncoderPatternIntegratesLateralMotion() {
        val estimator = FuzzyPoseEstimator()
        estimator.ingest(Telemetry(yaw = 0.0, encoders = listOf(0, 0, 0, 0)))
        val pose = estimator.ingest(Telemetry(yaw = 0.0, encoders = listOf(156, -156, -156, 156)))

        assertEquals(0.0, pose.xMetres, 0.0001)
        assertEquals(-metresFor156Counts, pose.yMetres, 0.0001)
    }

    @Test fun segmentLabelsDescribeTheRecordedCommand() {
        assertEquals("前进", PathSegment(Drive(linear = .1), 500).label())
        assertEquals("左平移", PathSegment(Drive(lateral = -.1), 500).label())
        assertEquals("右转", PathSegment(Drive(angular = -.4), 500).label())
        assertEquals("驻车", PathSegment(Drive(), 500).label())
    }
}
