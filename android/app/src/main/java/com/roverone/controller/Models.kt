package com.roverone.controller

enum class Connection { DISCONNECTED, CONNECTING, CONNECTED }
enum class ControlLink { WIFI, BLE }
data class Vision(
    val enabled: Boolean = false, val ready: Boolean = false, val target: String = "red",
    val error: String? = null, val found: Boolean = false,
    val horizontalError: Double? = null, val sizeRatio: Double? = null,
    val linear: Double = 0.0, val angular: Double = 0.0, val reason: String = "disabled"
)
data class Gateway(
    val simulated: Boolean = false, val uartConnected: Boolean = false,
    val serialReady: Boolean = false, val estop: Boolean = true,
    val faultLatched: Boolean = true, val faultCode: Int = 0,
    val clearPending: Boolean = false, val watchdog: String = "unknown",
    val serialError: String? = null, val obstacleGuardEnabled: Boolean = true,
    val obstacleGuardSupported: Boolean = false, val obstacleBlocked: Boolean = false,
    val vision: Vision? = null
)
data class Telemetry(
    val battery: Double? = null, val distance: Double? = null,
    val linear: Double? = null, val angular: Double? = null, val yaw: Double? = null,
    val encoders: List<Long?> = List(4) { null }, val simulated: Boolean = false,
    val estop: Boolean = false, val faultCode: Int = 0
)
data class Ack(val seq: Long?, val accepted: Boolean, val applied: Boolean,
    val stage: String, val reason: String, val ackedType: Int?)
data class Status(val event: String, val gateway: Gateway?, val telemetry: Telemetry?,
    val ack: Ack?, val controlGranted: Boolean)
data class Drive(
    val linear: Double = 0.0,
    val angular: Double = 0.0,
    val lateral: Double = 0.0
)

/** Current rover hardware reports the 12 V rail through a 2:1 divider. */
fun displayedBatteryVoltage(raw: Double?, simulated: Boolean): Double? =
    raw?.let { if (simulated) it else it * 0.5 }

fun roverFaultDescription(code: Int, distanceMetres: Double? = null): String = when (code) {
    0 -> "无故障"
    1 -> "STM32 通信超时，等待控制链路恢复"
    2 -> "物理急停未释放"
    0x11 -> "左前轮编码器故障"
    0x12 -> "右前轮编码器故障"
    0x13 -> "左后轮编码器故障"
    0x14 -> "右后轮编码器故障"
    0x20 -> "STM32 UART 接收故障"
    0x30 -> {
        val distance = distanceMetres?.takeIf { it.isFinite() }
            ?.let { "（当前约 ${kotlin.math.round(it * 100).toInt()} cm）" }
            .orEmpty()
        "前方 20 cm 内检测到障碍物$distance；新固件只允许不超过 0.15 m/s 的直线后退。若仍显示急停锁存，请移到 30 cm 外等待 0.5 秒后解除"
    }
    0x31 -> "超声波传感器无有效数据，请检查供电、TRIG 和 ECHO 接线"
    else -> "STM32 硬件故障，代码 0x${code.toString(16).uppercase().padStart(2, '0')}"
}

data class RoverUiState(
    val address: String = "http://10.42.0.1", val simulation: Boolean = false,
    val controlLink: ControlLink = ControlLink.WIFI, val bleAddress: String = "",
    val connection: Connection = Connection.DISCONNECTED, val network: String = "未连接",
    val gateway: Gateway? = null, val telemetry: Telemetry? = null,
    val telemetryAgeMs: Long? = null, val armed: Boolean = false,
    val canArm: Boolean = false, val canClear: Boolean = false, val clearPending: Boolean = false,
    val canConfigureObstacleGuard: Boolean = false, val obstacleGuardPending: Boolean = false,
    val canVisionFollow: Boolean = false,
    val canShutdownOrangePi: Boolean = false,
    val pose: RoverPose = RoverPose(), val recordedPaths: List<RecordedPath> = emptyList(),
    val recordingPath: Boolean = false, val replayingPath: Boolean = false,
    val canRecordPath: Boolean = false, val canReplayPath: Boolean = false,
    val command: Drive = Drive(), val speedLimit: Float = 0.15f,
    val message: String = "选择 Wi-Fi 或 BLE 备用链路后连接小车",
    val lastAck: String = "尚无确认", val logs: List<String> = emptyList()
)
