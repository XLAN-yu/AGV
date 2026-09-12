package com.roverone.controller

/** Single-threaded state machine. Times are monotonic milliseconds, never wall clock. */
class SafetyEngine(val simulation: Boolean) {
    var granted = false; private set
    var gateway: Gateway? = null; private set
    var telemetry: Telemetry? = null; private set
    var lastTelemetryAt: Long? = null; private set
    var lastStatusAt: Long? = null; private set
    var armed = false; private set
    var command = Drive(); private set
    var localEstop = false; private set
    var pendingClear: Long? = null; private set
    var pendingObstacleGuard: Long? = null; private set
    private var stickX = 0f
    private var stickY = 0f
    private var strafeX = 0f
    private var clearStartedAt = 0L
    private var sequence = 0L
    var note = "连接后保持锁定"; private set

    @Synchronized fun nextSequence(): Long {
        check(sequence <= 0xFFFFFFFFL) { "命令序号已用尽，请重新连接" }
        return sequence++
    }
    fun ingest(status: Status, now: Long) {
        lastStatusAt = now
        if (status.event == "connected") { granted = status.controlGranted; lock("连接成功，请右滑启用") }
        if (status.event in listOf("connected", "serial_state")) {
            pendingClear = null
            lastTelemetryAt = null
            telemetry = null
            lock("等待新的小车遥测")
        }
        status.gateway?.let { gateway = it }
        if (status.telemetry != null) { telemetry = status.telemetry; lastTelemetryAt = now }
        if (obstacleRetreatOnly() && armed) {
            if (command.linear >= 0.0 || command.angular != 0.0 || command.lateral != 0.0) {
                resetInputs(); command = Drive()
            }
            note = "前方遇障：松开操作，只可直线后退"
        }
        if (status.event == "watchdog") lock("网关看门狗已停车，请重新启用")
        val ack = status.ack
        if (pendingObstacleGuard != null && ack != null && ack.seq == pendingObstacleGuard) {
            pendingObstacleGuard = null
            note = if (ack.accepted && ack.applied && ack.ackedType == 5) {
                if (gateway?.obstacleGuardEnabled == true) "超声波自动停车已开启" else "超声波自动停车已关闭"
            } else "自动停车设置失败：${ack.reason}"
        }
        if (pendingClear != null && ack != null && ack.seq == pendingClear) {
            if (!ack.accepted) {
                pendingClear = null
                note = if (ack.reason == "stm32_rejected_hardware_fault") {
                    "解除失败：${currentFaultDescription()}"
                } else {
                    "解除失败：${ack.reason}"
                }
            } else if ((simulation && ack.stage == "dry_run" && ack.reason == "estop_cleared") ||
                (!simulation && ack.stage == "stm32" && ack.applied && ack.ackedType == 3 && ack.reason == "estop_cleared")) {
                pendingClear = null
                localEstop = false
                lock("解除已确认，请重新向右滑动启用")
                // Require a STATUS after the MCU ACK before enabling motion again.
                lastTelemetryAt = null
            }
        }
        if (armed && blockedReason(now) != null) lock(blockedReason(now)!!)
    }
    fun blockedReason(now: Long): String? {
        val g = gateway ?: return "等待网关状态"
        if (!granted) return "尚未获得控制权"
        if (g.simulated != simulation) return if (g.simulated) "网关是仿真模式，请切换 App 模式后重连" else "网关是实车模式，请切换 App 模式后重连"
        if (!g.serialReady || (!simulation && !g.uartConnected)) return "UART 尚未就绪"
        if (!g.serialError.isNullOrBlank()) return "串口异常：${g.serialError}"
        val gatewayFault = g.faultCode
        val telemetryFault = telemetry?.faultCode ?: 0
        val directionalObstacle = obstacleRetreatOnly()
        if (g.faultLatched ||
            (gatewayFault != 0 && !(directionalObstacle && gatewayFault == 0x30)) ||
            (telemetryFault != 0 && !(directionalObstacle && telemetryFault == 0x30))) {
            return currentFaultDescription()
        }
        if (localEstop || g.estop || telemetry?.estop == true) return "急停已锁定"
        if (pendingClear != null || g.clearPending) return "等待解除确认"
        val age = lastTelemetryAt?.let { now - it } ?: return "等待小车遥测"
        if (age !in 0..900) return "超过 900 ms 无新遥测，已锁定"
        return null
    }
    fun arm(now: Long): Boolean {
        val blocked = blockedReason(now)
        if (blocked != null) { note = blocked; return false }
        resetInputs()
        armed = true; command = Drive(); note = when {
            simulation -> "仿真控制已启用"
            obstacleRetreatOnly() -> "前方遇障：只可直线后退，最大 0.15 m/s"
            else -> "控制已启用，松手即停"
        }
        return true
    }
    fun stick(x: Float, y: Float, speed: Float, now: Long) {
        if (!x.isFinite() || !y.isFinite() || !speed.isFinite()) { lock("输入无效"); return }
        stickX = x.coerceIn(-1f, 1f)
        stickY = y.coerceIn(-1f, 1f)
        updateCommand(speed, now)
    }
    fun strafe(x: Float, speed: Float, now: Long) {
        if (!x.isFinite() || !speed.isFinite()) { lock("输入无效"); return }
        strafeX = x.coerceIn(-1f, 1f)
        updateCommand(speed, now)
    }
    fun updateSpeed(speed: Float, now: Long) {
        if (!speed.isFinite()) { lock("速度设置无效"); return }
        updateCommand(speed, now)
    }
    fun directional(direction: PathDirection, pressed: Boolean, speed: Float, now: Long) {
        if (!pressed) {
            resetInputs(); command = Drive(); return
        }
        resetInputs()
        when (direction) {
            PathDirection.FORWARD -> stickY = 1f
            PathDirection.BACKWARD -> stickY = -1f
            PathDirection.TURN_LEFT -> stickX = -1f
            PathDirection.TURN_RIGHT -> stickX = 1f
            PathDirection.STRAFE_LEFT -> strafeX = -1f
            PathDirection.STRAFE_RIGHT -> strafeX = 1f
        }
        updateCommand(speed, now)
    }
    fun replayCommand(drive: Drive, now: Long): Boolean {
        if (!armed) return false
        val blocked = blockedReason(now)
        if (blocked != null) {
            lock("路径复现中止：$blocked")
            return false
        }
        if (obstacleRetreatOnly()) {
            lock("路径复现遇到前方障碍，已停止；请到控制页手动后退")
            return false
        }
        resetInputs()
        command = Drive(
            drive.linear.coerceIn(-1.0, 1.0),
            drive.angular.coerceIn(-3.0, 3.0),
            drive.lateral.coerceIn(-1.0, 1.0)
        )
        return true
    }
    private fun updateCommand(speed: Float, now: Long) {
        if (!armed || blockedReason(now) != null) { command = Drive(); return }
        fun dead(v: Float) = if (kotlin.math.abs(v) < 0.08f) 0.0 else v.coerceIn(-1f, 1f).toDouble()
        val forward = dead(stickY)
        val turn = dead(stickX)
        val lateral = dead(strafeX)
        val speedLimit = speed.coerceIn(0.05f, 0.5f)
        val proposed = Drive(if (forward == 0.0) 0.0 else forward * speedLimit,
            if (turn == 0.0) 0.0 else -turn * turnRateFor(speedLimit),
            if (lateral == 0.0) 0.0 else lateral * speedLimit)
        command = if (obstacleRetreatOnly()) {
            if (proposed.linear < 0.0 && proposed.angular == 0.0 && proposed.lateral == 0.0) {
                proposed.copy(linear = proposed.linear.coerceAtLeast(-0.15))
            } else {
                note = "前方遇障：只可直线后退"
                Drive()
            }
        } else proposed
    }
    private fun turnRateFor(speed: Float): Double = when {
        speed <= 0.20f -> 1.8 // 精细：提高低速转向响应，仍保留精确微调空间
        speed <= 0.35f -> 2.4 // 标准
        else -> 3.0           // 高速：协议允许的安全上限
    }
    private fun resetInputs() { stickX = 0f; stickY = 0f; strafeX = 0f }
    fun lock(reason: String) { armed = false; resetInputs(); command = Drive(); note = reason }
    fun estop() { localEstop = true; pendingClear = null; lock("急停已发送，等待小车确认") }
    fun canClear(now: Long): Boolean {
        val g = gateway ?: return false
        return granted && g.simulated == simulation && g.serialReady && (simulation || g.uartConnected) &&
            g.serialError.isNullOrBlank() && pendingClear == null && !g.clearPending &&
            lastStatusAt?.let { now - it in 0..499 } == true &&
            (localEstop || g.estop || g.faultLatched ||
                (g.faultCode != 0 && g.faultCode != 0x30))
    }
    fun requestClear(seq: Long, now: Long): Boolean {
        if (!canClear(now)) return false
        // Keep a local interlock even when resetting only a hardware fault.
        localEstop = true
        lock("等待 STM32 解除确认")
        pendingClear = seq; clearStartedAt = now
        return true
    }
    fun canConfigureObstacleGuard(now: Long): Boolean {
        val g = gateway ?: return false
        return !armed && granted && g.simulated == simulation && g.serialReady &&
            (simulation || g.uartConnected) && g.serialError.isNullOrBlank() &&
            (g.simulated || g.obstacleGuardSupported) &&
            pendingClear == null && pendingObstacleGuard == null &&
            lastStatusAt?.let { now - it in 0..499 } == true
    }
    fun requestObstacleGuard(seq: Long, enabled: Boolean, now: Long): Boolean {
        if (!canConfigureObstacleGuard(now)) return false
        pendingObstacleGuard = seq
        note = if (enabled) "正在开启超声波自动停车" else "正在关闭超声波自动停车"
        return true
    }
    fun canShutdownOrangePi(now: Long): Boolean {
        val g = gateway ?: return false
        return !armed && granted && g.simulated == simulation &&
            lastStatusAt?.let { now - it in 0..900 } == true
    }
    private fun currentFaultDescription(): String {
        val liveCode = telemetry?.faultCode ?: 0
        val code = if (liveCode != 0) liveCode else gateway?.faultCode ?: 0
        return roverFaultDescription(code, telemetry?.distance)
    }
    fun obstacleRetreatOnly(): Boolean {
        val g = gateway ?: return false
        val liveCode = telemetry?.faultCode ?: 0
        return g.obstacleBlocked && !g.faultLatched &&
            (liveCode == 0x30 || g.faultCode == 0x30 || liveCode == 0)
    }
    fun tick(now: Long) {
        if (pendingClear != null && now - clearStartedAt >= 1500) {
            pendingClear = null; lock("解除确认超时，仍保持锁定")
        }
        if (armed) blockedReason(now)?.let { lock(it) }
    }
}
