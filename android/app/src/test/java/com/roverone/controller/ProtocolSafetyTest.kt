package com.roverone.controller

import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test

class ProtocolSafetyTest {
    private fun ready(sim: Boolean = false) = Gateway(sim, !sim, true, false, false)
    private fun engine(sim: Boolean = false): SafetyEngine = SafetyEngine(sim).also {
        it.ingest(Status("connected", ready(sim), null, null, true), 0)
        it.ingest(Status("telemetry", ready(sim), Telemetry(simulated = sim), null, false), 100)
    }
    private fun keys(raw: String) = JSONObject(raw).keys().asSequence().toSet()
    @Test fun exactCommandFields() {
        assertEquals(setOf("type", "linear", "angular", "seq"), keys(Protocol.drive(126, .2, .45)))
        assertEquals(setOf("type", "linear", "lateral", "angular", "seq"),
            keys(Protocol.drive(126, .2, .45, -.3)))
        assertEquals(-.3, JSONObject(Protocol.drive(126, .2, .45, -.3)).getDouble("lateral"), .0001)
        assertEquals(setOf("type", "seq"), keys(Protocol.estop(127)))
        assertEquals(setOf("type", "seq", "confirm"), keys(Protocol.clear(128)))
        assertEquals(true, JSONObject(Protocol.clear(128)).getBoolean("confirm"))
        assertEquals(setOf("type", "seq", "confirm"), keys(Protocol.shutdownOrangePi(129)))
        assertEquals(true, JSONObject(Protocol.shutdownOrangePi(129)).getBoolean("confirm"))
        assertEquals(setOf("type", "enabled", "target", "seq"),
            keys(Protocol.visionFollow(130, true, "red")))
        assertEquals(4294967295L, JSONObject(Protocol.drive(4294967295L, 0.0, 0.0)).getLong("seq"))
    }
    @Test fun invalidMotionRejected() {
        for (v in listOf(Double.NaN, Double.POSITIVE_INFINITY, -1.01, 1.01)) {
            assertThrows(IllegalArgumentException::class.java) { Protocol.drive(1, v, 0.0) }
        }
        assertThrows(IllegalArgumentException::class.java) { Protocol.drive(-1, 0.0, 0.0) }
        assertThrows(IllegalArgumentException::class.java) { Protocol.drive(1, 0.0, 3.01) }
        assertThrows(IllegalArgumentException::class.java) { Protocol.drive(1, 0.0, 0.0, 1.01) }
    }
    @Test fun twelveVoltHardwareDisplayUsesDividerCalibration() {
        assertEquals(12.05, displayedBatteryVoltage(24.1, simulated = false)!!, 0.0001)
        assertEquals(12.4, displayedBatteryVoltage(12.4, simulated = true)!!, 0.0001)
        assertNull(displayedBatteryVoltage(null, simulated = false))
    }
    @Test fun hardwareFaultCodesHaveActionableDescriptions() {
        assertTrue(roverFaultDescription(0x30, 0.18).contains("18 cm"))
        assertTrue(roverFaultDescription(0x30, 0.18).contains("直线后退"))
        assertTrue(roverFaultDescription(0x31).contains("TRIG"))
        assertTrue(roverFaultDescription(0x11).contains("左前轮"))
    }
    @Test fun originsAndPortsNormalized() {
        val ep = Endpoint.parse("172.20.10.2:8000")
        assertEquals("http://172.20.10.2:8000", ep.origin)
        assertEquals("ws://172.20.10.2:8000/ws", ep.webSocket)
        assertEquals("http://172.20.10.2:8000/health", ep.health)
        assertEquals("http://10.42.0.1", Endpoint.parse("ws://10.42.0.1:80/ws").origin)
        assertEquals("wss://rover-one.local/ws", Endpoint.parse("https://rover-one.local").webSocket)
    }
    @Test fun externalOrAmbiguousAddressRejected() {
        for (s in listOf("https://example.com", "http://8.8.8.8", "http://192.168.1.1:99999",
            "ftp://192.168.1.1", "http://user@192.168.1.1", "http://192.168.1.1/?a=1", "http://192.168.1.1/else")) {
            assertThrows(Exception::class.java) { Endpoint.parse(s) }
        }
    }
    @Test fun startsLockedAndRequiresTelemetry() {
        val e = SafetyEngine(false)
        assertFalse(e.arm(0))
        e.ingest(Status("connected", ready(), null, null, true), 1)
        assertFalse(e.arm(1))
        e.ingest(Status("telemetry", ready(), Telemetry(), null, false), 100)
        assertTrue(e.arm(100))
        assertEquals(Drive(), e.command)
    }
    @Test fun frontObstacleAllowsOnlySlowStraightManualRetreat() {
        val e = SafetyEngine(false)
        val obstacleGateway = ready().copy(obstacleBlocked = true)
        e.ingest(Status("connected", obstacleGateway, null, null, true), 0)
        e.ingest(Status("telemetry", obstacleGateway,
            Telemetry(distance = .18, faultCode = 0x30), null, false), 100)
        assertTrue(e.arm(100))

        e.directional(PathDirection.FORWARD, true, .5f, 100)
        assertEquals(Drive(), e.command)
        e.directional(PathDirection.BACKWARD, true, .5f, 100)
        assertEquals(-.15, e.command.linear, .0001)
        assertEquals(0.0, e.command.angular, .0001)
        assertEquals(0.0, e.command.lateral, .0001)
        e.stick(.2f, -1f, .15f, 100)
        assertEquals(Drive(), e.command)
        assertFalse(e.replayCommand(Drive(-.1, 0.0, 0.0), 100))
        assertFalse(e.armed)
    }
    @Test fun ackCannotRefreshStaleTelemetry() {
        val e = engine()
        assertTrue(e.arm(100))
        e.ingest(Status("ack", ready(), null, Ack(1, true, false, "gateway", "drive", null), false), 999)
        assertEquals(100L, e.lastTelemetryAt)
        e.tick(1001)
        assertFalse(e.armed)
        assertEquals(Drive(), e.command)
    }
    @Test fun replayKeepsTheActualSafetyAbortReason() {
        val e = engine()
        assertTrue(e.arm(100))
        assertFalse(e.replayCommand(Drive(linear = .2), 1001))
        assertTrue(e.note.contains("900 ms"))
        assertFalse(e.note.contains("前方障碍"))
    }
    @Test fun freshTelemetryNeverAutomaticallyRearms() {
        val e = engine()
        e.arm(100); e.tick(1001)
        e.ingest(Status("telemetry", ready(), Telemetry(), null, false), 1100)
        assertFalse(e.armed)
        assertTrue(e.arm(1100))
    }
    @Test fun modeMismatchBlocksMotionBothWays() {
        for (sim in listOf(false, true)) {
            val e = SafetyEngine(sim)
            e.ingest(Status("connected", ready(!sim), null, null, true), 0)
            e.ingest(Status("telemetry", ready(!sim), Telemetry(simulated = !sim), null, false), 1)
            assertFalse(e.arm(1))
        }
    }
    @Test fun realModeRequiresUartSimulationDoesNot() {
        val e = engine()
        e.ingest(Status("telemetry", ready().copy(uartConnected = false), Telemetry(), null, false), 120)
        assertFalse(e.arm(120))
        assertTrue(engine(true).arm(120))
    }
    @Test fun serialReconnectClearsFreshness() {
        val e = engine()
        e.arm(100)
        e.ingest(Status("serial_state", ready(), null, null, false), 110)
        assertNull(e.lastTelemetryAt); assertFalse(e.armed)
        assertFalse(e.arm(110))
    }
    @Test fun watchdogEventLocksButIdleTrippedDoesNotBlockArm() {
        val e = engine()
        assertTrue(e.arm(120)) // initial watchdog is unknown/tripped, not a permanent interlock
        e.ingest(Status("watchdog", ready().copy(watchdog = "tripped"), null, null, false), 150)
        assertFalse(e.armed)
        assertTrue(e.arm(150))
    }
    @Test fun stickHasDirectionDeadZoneAndFiniteLimits() {
        val e = engine()
        e.arm(100); e.stick(1f, 1f, .2f, 100)
        assertEquals(.2, e.command.linear, .0001)
        assertEquals(-1.8, e.command.angular, .0001)
        e.stick(.02f, .02f, .2f, 100); assertEquals(Drive(), e.command)
        e.stick(Float.NaN, 1f, .2f, 100); assertFalse(e.armed)
        assertEquals(Drive(), e.command)
    }
    @Test fun strafeCombinesWithJoystickAndReturnsToZero() {
        val e = engine()
        e.arm(100)
        e.stick(0f, .5f, .2f, 100)
        e.strafe(-1f, .2f, 100)
        assertEquals(.1, e.command.linear, .0001)
        assertEquals(-.2, e.command.lateral, .0001)
        e.strafe(0f, .2f, 100)
        assertEquals(0.0, e.command.lateral, .0001)
        assertEquals(.1, e.command.linear, .0001)
        e.stick(0f, 0f, .2f, 100)
        assertEquals(Drive(), e.command)
    }
    @Test fun speedChangeKeepsAnArmedSessionAndRecalculatesDrive() {
        val e = engine()
        assertTrue(e.arm(100))
        e.stick(0f, 1f, .15f, 100)
        e.updateSpeed(.5f, 120)
        assertTrue(e.armed)
        assertEquals(.5, e.command.linear, .0001)
    }
    @Test fun turnRateUsesTheFasterFineStandardAndHighProfiles() {
        val e = engine()
        e.arm(100)
        e.stick(1f, 0f, .15f, 100)
        assertEquals(-1.8, e.command.angular, .0001)
        e.stick(1f, 0f, .30f, 110)
        assertEquals(-2.4, e.command.angular, .0001)
        e.stick(1f, 0f, .50f, 120)
        assertEquals(-3.0, e.command.angular, .0001)
    }
    @Test fun recordedDirectionButtonsUseOneAxisAndReleaseToZero() {
        val e = engine()
        assertTrue(e.arm(100))
        e.directional(PathDirection.STRAFE_LEFT, true, .2f, 100)
        assertEquals(0.0, e.command.linear, .0001)
        assertEquals(-.2, e.command.lateral, .0001)
        e.directional(PathDirection.STRAFE_LEFT, false, .2f, 120)
        assertEquals(Drive(), e.command)
        e.directional(PathDirection.TURN_RIGHT, true, .2f, 140)
        assertEquals(-1.8, e.command.angular, .0001)
    }
    @Test fun estopBlocksOrdinaryDrive() {
        val e = engine()
        e.arm(100); e.estop(); e.stick(0f, 1f, .5f, 100)
        assertFalse(e.armed); assertFalse(e.arm(100)); assertEquals(Drive(), e.command)
    }
    @Test fun clearRequiresMatchingStm32AppliedAckAndNewTelemetry() {
        val e = engine()
        e.estop()
        val latched = ready().copy(estop = true)
        e.ingest(Status("telemetry", latched, Telemetry(estop = true), null, false), 100)
        assertTrue(e.requestClear(20, 100))
        e.ingest(Status("ack", latched.copy(clearPending = true), null,
            Ack(20, true, false, "gateway", "awaiting_stm32_ack", null), false), 120)
        assertEquals(20L, e.pendingClear); assertFalse(e.arm(120))
        e.ingest(Status("ack", ready(), null, Ack(19, true, true, "stm32", "estop_cleared", 3), false), 130)
        assertEquals(20L, e.pendingClear)
        e.ingest(Status("ack", ready(), null, Ack(20, true, false, "stm32", "estop_cleared", 3), false), 140)
        assertEquals(20L, e.pendingClear)
        e.ingest(Status("ack", ready(), null, Ack(20, true, true, "stm32", "estop_cleared", 3), false), 150)
        assertNull(e.pendingClear); assertFalse(e.localEstop); assertFalse(e.armed)
        assertFalse(e.arm(150))
        e.ingest(Status("telemetry", ready(), Telemetry(), null, false), 180)
        assertTrue(e.arm(180))
    }
    @Test fun timeoutAndLateClearAckRemainLocked() {
        val e = engine(); e.estop(); assertTrue(e.requestClear(30, 100))
        e.tick(1600)
        assertNull(e.pendingClear); assertTrue(e.localEstop)
        e.ingest(Status("ack", ready(), null, Ack(30, true, true, "stm32", "estop_cleared", 3), false), 1700)
        assertTrue(e.localEstop); assertFalse(e.arm(1700))
    }
    @Test fun dryRunClearAcceptedWithoutClaimingHardwareApplied() {
        val e = engine(true); e.estop(); assertTrue(e.requestClear(4, 100))
        e.ingest(Status("ack", ready(true), null, Ack(4, true, false, "dry_run", "estop_cleared", null), false), 150)
        assertNull(e.pendingClear); assertFalse(e.localEstop); assertFalse(e.armed)
    }
    @Test fun hardwareFaultStopsAnArmedSession() {
        val e = engine(); e.arm(100)
        e.ingest(Status("telemetry", ready().copy(faultCode = 17, faultLatched = true),
            Telemetry(faultCode = 17), null, false), 150)
        assertFalse(e.armed); assertTrue(e.canClear(150))
    }
    @Test fun rejectedObstacleClearExplainsHowToRecover() {
        val e = engine()
        val blocked = ready().copy(estop = true, faultLatched = true, faultCode = 0x30)
        e.ingest(Status("telemetry", blocked,
            Telemetry(distance = .18, estop = true, faultCode = 0x30), null, false), 120)
        assertTrue(e.requestClear(9, 120))
        e.ingest(Status("ack", blocked, null,
            Ack(9, false, true, "stm32", "stm32_rejected_hardware_fault", 3), false), 140)
        assertTrue(e.note.contains("18 cm"))
        assertTrue(e.note.contains("30 cm"))
    }
    @Test fun realAndSimulationPayloadsDecodeWithoutInventingData() {
        val g = """{"mode":"uart","simulated":false,"uart_connected":true,"serial_ready":true,"estop_latched":false,"hardware_fault_latched":false,"hardware_fault_code":0,"clear_estop_pending":false,"watchdog":"healthy","serial_error":null}"""
        val t = """{"battery_v":12.1,"ultrasonic_m":null,"measured_linear":0.2,"measured_angular":0.3,"imu_yaw":0.4,"estop":false,"fault_code":0,"encoders":{"front_left":1,"front_right":-2,"rear_left":3,"rear_right":4}}"""
        val real = Protocol.status("""{"type":"robot_status","event":"telemetry","gateway":$g,"telemetry":$t}""")
        assertEquals(12.1, real.telemetry!!.battery!!, 0.0)
        assertNull(real.telemetry.distance)
        assertEquals(listOf(1L, -2L, 3L, 4L), real.telemetry.encoders)
        val sg = g.replace("\"uart\"", "\"dry_run\"").replace("\"simulated\":false", "\"simulated\":true").replace("\"uart_connected\":true", "\"uart_connected\":false")
        val st = """{"simulated":true,"batteryVoltage":12.4,"distanceCm":120,"estop":false}"""
        val sim = Protocol.status("""{"type":"robot_status","event":"telemetry","gateway":$sg,"telemetry":$st}""")
        assertEquals(1.2, sim.telemetry!!.distance!!, 0.0)
        assertNull(sim.telemetry.linear); assertEquals(List(4) { null }, sim.telemetry.encoders)
        assertFalse(sim.gateway!!.uartConnected)
    }
    @Test fun visionStatusDecodesWithoutAffectingTelemetryContract() {
        val g = """{"mode":"uart","simulated":false,"uart_connected":true,"serial_ready":true,"estop_latched":false,"hardware_fault_latched":false,"hardware_fault_code":0,"clear_estop_pending":false,"watchdog":"healthy","serial_error":null,"vision":{"enabled":true,"ready":true,"target":"red","error":null,"observation":{"found":true,"horizontal_error":-0.2,"size_ratio":0.03},"command":{"linear":0.06,"angular":0.18,"reason":"following"}}}"""
        val status = Protocol.status("""{"type":"robot_status","event":"vision","gateway":$g}""")
        assertTrue(status.gateway!!.vision!!.enabled)
        assertTrue(status.gateway.vision!!.found)
        assertEquals(-0.2, status.gateway.vision!!.horizontalError!!, 0.0)
        assertEquals(0.06, status.gateway.vision!!.linear, 0.0)
    }
    @Test fun rejectedPacketDoesNotRequireGateway() {
        val rejected = Protocol.status("""{"type":"robot_status","event":"rejected","ack":{"accepted":false,"reason":"controller_busy"}}""")
        assertNull(rejected.gateway)
        assertEquals("controller_busy", rejected.ack!!.reason)
        assertNull(rejected.ack.seq)
    }
    @Test fun serialReconnectCancelsClearAndIgnoresOldAck() {
        val e = engine(); e.estop(); assertTrue(e.requestClear(7, 100))
        e.ingest(Status("serial_state", ready(), null, null, false), 110)
        assertNull(e.pendingClear)
        e.ingest(Status("ack", ready(), null, Ack(7, true, true, "stm32", "estop_cleared", 3), false), 120)
        assertTrue(e.localEstop); assertFalse(e.arm(120))
    }
    @Test fun faultOnlyClearRetainsInterlockOnTimeout() {
        val e = engine()
        e.ingest(Status("telemetry", ready().copy(faultLatched = true, faultCode = 17),
            Telemetry(faultCode = 17), null, false), 100)
        assertTrue(e.requestClear(8, 100))
        e.tick(1600)
        e.ingest(Status("telemetry", ready(), Telemetry(), null, false), 1650)
        assertTrue(e.localEstop); assertFalse(e.arm(1650))
    }
}
