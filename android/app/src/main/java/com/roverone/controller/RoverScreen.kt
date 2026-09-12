package com.roverone.controller

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.Article
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.automirrored.filled.ArrowForward
import androidx.compose.material.icons.filled.BatteryFull
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.Link
import androidx.compose.material.icons.filled.PowerSettingsNew
import androidx.compose.material.icons.filled.Router
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material.icons.filled.SportsEsports
import androidx.compose.material.icons.filled.StopCircle
import androidx.compose.material.icons.filled.SwapHoriz
import androidx.compose.material.icons.filled.VerticalAlignCenter
import androidx.compose.material.icons.filled.Wifi
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.pointer.PointerEventPass
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.Lifecycle
import java.util.Locale
import kotlin.math.PI
import kotlin.math.sqrt

private val AppBackground = Color(0xFF101112)
private val Card = Color(0xFF1B1C1E)
private val CardRaised = Color(0xFF252629)
private val NavGlass = Color(0xE8172130)
private val Teal = Color(0xFF4D8DFF)
private val TealSoft = Color(0xFF1C3154)
private val TextPrimary = Color(0xFFF2F3F3)
private val TextMuted = Color(0xFF9B9EA3)
private val Divider = Color(0xFF303236)
private val Danger = Color(0xFFFF6268)
private val Success = Color(0xFF43D17D)
private val Warning = Color(0xFFFFC869)

private val appColors = darkColorScheme(
    primary = Teal, onPrimary = Color(0xFF06211E), background = AppBackground,
    surface = Card, onSurface = TextPrimary, onSurfaceVariant = TextMuted,
    secondaryContainer = TealSoft, onSecondaryContainer = Teal, error = Danger
)

private enum class AppPage { CONNECTION, CONTROL, PATH, RECORDS }
private enum class ControlMode { JOYSTICK, THROTTLE }

private fun fmt(value: Double?, digits: Int = 2): String = value?.let {
    String.format(Locale.US, "%." + digits + "f", it)
} ?: "—"

private fun headingDegrees(yaw: Double?): String {
    if (yaw == null) return "—"
    val degrees = ((yaw * 180.0 / PI) % 360.0 + 360.0) % 360.0
    return String.format(Locale.US, "%.0f°", degrees)
}

private fun batteryVoltage(state: RoverUiState): Double? = displayedBatteryVoltage(
    state.telemetry?.battery,
    state.gateway?.simulated ?: state.simulation
)

@Composable
fun RoverScreen(
    state: RoverUiState, onAddress: (String) -> Unit, onBleAddress: (String) -> Unit,
    onControlLink: (ControlLink) -> Unit, onSimulation: (Boolean) -> Unit,
    onConnect: () -> Unit, onDisconnect: () -> Unit, onOpenWifi: () -> Unit, onOpenBluetooth: () -> Unit,
    onArm: () -> Unit, onStop: () -> Unit, onEstop: () -> Unit, onClear: () -> Unit,
    onObstacleGuard: (Boolean) -> Unit, onVisionFollow: (Boolean, String) -> Unit,
    onShutdownOrangePi: () -> Unit,
    onStick: (Float, Float) -> Unit, onStrafe: (Float) -> Unit, onSpeed: (Float) -> Unit,
    onWheelTest: (Int, Int) -> Unit,
    onTogglePathRecording: () -> Unit, onPathDirection: (PathDirection, Boolean) -> Unit,
    onReplayPath: (String) -> Unit, onRenamePath: (String, String) -> Unit, onDeletePath: (String) -> Unit,
    onSetPathSegmentDuration: (String, Int, Long) -> Unit, onAddParkingAfter: (String, Int, Long) -> Unit,
    onDeletePathSegment: (String, Int) -> Unit
) {
    var pageName by rememberSaveable { mutableStateOf(AppPage.CONNECTION.name) }
    var editingPathId by rememberSaveable { mutableStateOf<String?>(null) }
    var clearDialog by remember { mutableStateOf(false) }
    var shutdownDialog by remember { mutableStateOf(false) }
    val page = AppPage.valueOf(pageName)

    fun selectPage(next: AppPage) {
        if (page == AppPage.CONTROL && next != AppPage.CONTROL &&
            (state.armed || state.gateway?.vision?.enabled == true)) onStop()
        if (next != AppPage.PATH || page == AppPage.PATH) editingPathId = null
        pageName = next.name
    }

    MaterialTheme(colorScheme = appColors) {
        Scaffold(
            containerColor = AppBackground,
            topBar = { RoverTopBar(state) },
            bottomBar = { FloatingNavigation(page, ::selectPage) }
        ) { insets ->
            Box(
                Modifier.fillMaxSize().padding(insets).padding(horizontal = 14.dp).padding(bottom = 8.dp)
            ) {
                when (page) {
                    AppPage.CONNECTION -> ConnectionPage(
                        state, onAddress, onBleAddress, onControlLink, onSimulation,
                        onConnect, onDisconnect, onOpenWifi, onOpenBluetooth,
                        onGoControl = { selectPage(AppPage.CONTROL) },
                        onShutdownRequest = { shutdownDialog = true }
                    )
                    AppPage.CONTROL -> ControlPage(
                        state, onArm, onStop, onEstop, { clearDialog = true }, onObstacleGuard,
                        onVisionFollow, onStick, onStrafe, onSpeed,
                        onWheelTest,
                        onTogglePathRecording, onPathDirection
                    )
                    AppPage.PATH -> {
                        val editing = state.recordedPaths.firstOrNull { it.id == editingPathId }
                        if (editing == null) PathLibraryPage(
                            state, onArm, onEstop,
                            onStartRecording = {
                                if (!state.recordingPath && state.canRecordPath) {
                                    onTogglePathRecording()
                                    selectPage(AppPage.CONTROL)
                                }
                            },
                            onOpenPath = { editingPathId = it },
                            onReplayPath = onReplayPath,
                            onRenamePath = onRenamePath,
                            onDeletePath = onDeletePath
                        )
                        else PathEditorPage(
                            state, editing, onBack = { editingPathId = null }, onArm, onEstop, onReplayPath,
                            onSetPathSegmentDuration, onAddParkingAfter, onDeletePathSegment, onRenamePath
                        )
                    }
                    AppPage.RECORDS -> RecordsPage(state)
                }
            }
        }

        if (clearDialog) AlertDialog(
            onDismissRequest = { clearDialog = false },
            title = { Text("确认解除急停 / 故障？") },
            text = { Text("请先排除故障并确认小车周围安全。小车确认后仍保持锁定，需要重新向右滑动启用。") },
            confirmButton = {
                TextButton(onClick = { clearDialog = false; onClear() }, enabled = state.canClear) {
                    Text("确认解除")
                }
            },
            dismissButton = { TextButton(onClick = { clearDialog = false }) { Text("取消") } }
        )
        if (shutdownDialog) AlertDialog(
            onDismissRequest = { shutdownDialog = false },
            title = { Text("关闭香橙派？") },
            text = { Text("将先向 STM32 发送急停，再关闭香橙派。Wi-Fi 与 BLE 会断开；重新使用前需重新供电启动香橙派并解除急停。") },
            confirmButton = {
                TextButton(onClick = { shutdownDialog = false; onShutdownOrangePi() }, enabled = state.canShutdownOrangePi) {
                    Text("确认关机", color = Danger)
                }
            },
            dismissButton = { TextButton(onClick = { shutdownDialog = false }) { Text("取消") } }
        )
    }
}

@Composable
private fun RoverTopBar(state: RoverUiState) {
    Column(
        Modifier.fillMaxWidth().statusBarsPadding()
            .padding(start = 20.dp, end = 20.dp, top = 12.dp, bottom = 10.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        Row(
            Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(11.dp)) {
                Surface(shape = CircleShape, color = Teal, modifier = Modifier.size(42.dp)) {
                    Box(contentAlignment = Alignment.Center) {
                        Icon(Icons.Filled.Router, null, tint = Color(0xFF092421), modifier = Modifier.size(24.dp))
                    }
                }
                Column {
                    Text("REMOTE AGV", fontSize = 20.sp, fontWeight = FontWeight.Black, letterSpacing = 1.6.sp)
                    Text("12 V AGV 无线控制台", color = TextMuted, fontSize = 11.sp)
                }
            }
            ConnectionBadge(state)
        }

        AnimatedVisibility(state.connection == Connection.CONNECTED) {
            Surface(
                color = Card,
                shape = RoundedCornerShape(19.dp),
                modifier = Modifier.fillMaxWidth().border(1.dp, Divider, RoundedCornerShape(19.dp))
            ) {
                Row(
                    Modifier.fillMaxWidth().padding(vertical = 11.dp, horizontal = 5.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    HeaderMetric(
                        Icons.Filled.BatteryFull,
                        fmt(batteryVoltage(state), 1) + if (batteryVoltage(state) != null) " V" else "",
                        "12 V 电池", Modifier.weight(1f)
                    )
                    VerticalDivider()
                    HeaderMetric(
                        Icons.Filled.Sensors, headingDegrees(state.telemetry?.yaw),
                        "IMU 航向", Modifier.weight(1f)
                    )
                    VerticalDivider()
                    val fresh = state.telemetryAgeMs?.let { it < 500 } == true
                    HeaderMetric(
                        Icons.Filled.Link, if (fresh) "实时" else "等待",
                        if (state.gateway?.simulated == true) "仿真链路" else "车辆链路",
                        Modifier.weight(1f), if (fresh) Success else Danger
                    )
                }
            }
        }
        AnimatedVisibility(state.connection == Connection.CONNECTED) {
            Surface(
                color = Card, shape = RoundedCornerShape(19.dp),
                modifier = Modifier.fillMaxWidth().border(1.dp, Divider, RoundedCornerShape(19.dp))
            ) {
                Row(
                    Modifier.fillMaxWidth().padding(vertical = 11.dp, horizontal = 5.dp),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    HeaderMetric(
                        Icons.Filled.SportsEsports, if (state.armed) "控制中" else "已锁定", "控制状态",
                        Modifier.weight(1f), if (state.armed) Success else TextMuted
                    )
                    VerticalDivider()
                    HeaderMetric(
                        Icons.Filled.Sensors, fmt(state.telemetry?.yaw, 3), "航向弧度",
                        Modifier.weight(1f)
                    )
                    VerticalDivider()
                    HeaderMetric(
                        Icons.Filled.Link, fmt(state.telemetry?.angular, 2), "实测角速度",
                        Modifier.weight(1f)
                    )
                }
            }
        }
    }
}

@Composable
private fun ConnectionBadge(state: RoverUiState) {
    val (label, color) = when (state.connection) {
        Connection.CONNECTED -> "已连接" to Success
        Connection.CONNECTING -> "连接中" to Warning
        Connection.DISCONNECTED -> "未连接" to TextMuted
    }
    Surface(color = CardRaised, shape = RoundedCornerShape(18.dp)) {
        Row(
            Modifier.padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(7.dp)
        ) {
            Surface(color = color, shape = CircleShape, modifier = Modifier.size(7.dp)) {}
            Text(label, color = color, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
        }
    }
}

@Composable
private fun HeaderMetric(
    icon: ImageVector, value: String, label: String, modifier: Modifier = Modifier,
    valueColor: Color = TextPrimary
) {
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(5.dp)) {
            Icon(icon, null, tint = Teal, modifier = Modifier.size(15.dp))
            Text(value, color = valueColor, fontWeight = FontWeight.Bold, fontSize = 16.sp, maxLines = 1)
        }
        Text(label, color = TextMuted, fontSize = 10.sp)
    }
}

@Composable
private fun VerticalDivider() {
    Box(Modifier.width(1.dp).height(32.dp).background(Divider))
}

@Composable
private fun ConnectionPage(
    state: RoverUiState, onAddress: (String) -> Unit, onBleAddress: (String) -> Unit,
    onControlLink: (ControlLink) -> Unit, onSimulation: (Boolean) -> Unit,
    onConnect: () -> Unit, onDisconnect: () -> Unit, onOpenWifi: () -> Unit, onOpenBluetooth: () -> Unit,
    onGoControl: () -> Unit, onShutdownRequest: () -> Unit
) {
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(bottom = 20.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        PageHeading("连接小车", "Wi-Fi 主链路 / BLE 备用链路")
        AppCard {
            ConnectionModeSelector(
                selected = state.controlLink,
                enabled = state.connection == Connection.DISCONNECTED,
                onSelected = onControlLink
            )
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                Surface(shape = CircleShape, color = TealSoft, modifier = Modifier.size(46.dp)) {
                    Box(contentAlignment = Alignment.Center) {
                        Icon(if (state.controlLink == ControlLink.WIFI) Icons.Filled.Wifi else Icons.Filled.Bluetooth,
                            null, tint = Teal, modifier = Modifier.size(25.dp))
                    }
                }
                Column(Modifier.weight(1f)) {
                    Text(if (state.controlLink == ControlLink.WIFI) "局域网连接" else "香橙派 BLE 安全桥",
                        fontWeight = FontWeight.Bold, fontSize = 17.sp)
                    Text(state.network, color = TextMuted, fontSize = 12.sp, maxLines = 1, overflow = TextOverflow.Ellipsis)
                }
            }
            if (state.controlLink == ControlLink.WIFI) {
                OutlinedTextField(
                    state.address, onAddress, Modifier.fillMaxWidth(),
                    enabled = state.connection == Connection.DISCONNECTED, singleLine = true,
                    label = { Text("网关地址") }, placeholder = { Text("http://10.42.0.1") },
                    shape = RoundedCornerShape(15.dp)
                )
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(
                        onClick = { onAddress("http://10.42.0.1") },
                        enabled = state.connection == Connection.DISCONNECTED,
                        modifier = Modifier.weight(1f)
                    ) { Text("小车热点") }
                    OutlinedButton(onClick = onOpenWifi, modifier = Modifier.weight(1f)) { Text("Wi-Fi 设置") }
                }
                HorizontalDivider(color = Divider)
                Row(
                    Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column(Modifier.weight(1f)) {
                        Text("仿真网关", fontWeight = FontWeight.Medium)
                        Text("仅用于协议与界面测试", color = TextMuted, fontSize = 11.sp)
                    }
                    Switch(state.simulation, onSimulation, enabled = state.connection == Connection.DISCONNECTED)
                }
            } else {
                OutlinedTextField(
                    state.bleAddress, onBleAddress, Modifier.fillMaxWidth(),
                    enabled = state.connection == Connection.DISCONNECTED, singleLine = true,
                    label = { Text("已配对香橙派 BLE 地址") }, placeholder = { Text("AA:BB:CC:DD:EE:FF") },
                    shape = RoundedCornerShape(15.dp)
                )
                OutlinedButton(onClick = onOpenBluetooth, modifier = Modifier.fillMaxWidth()) {
                    Icon(Icons.Filled.Bluetooth, null); Spacer(Modifier.width(8.dp)); Text("打开蓝牙设置并配对")
                }
                Text("BLE 只替代手机到香橙派的链路；所有指令仍进入原 WebSocket 网关、UART 与 STM32 看门狗。",
                    color = TextMuted, fontSize = 11.sp)
            }
            Button(
                onClick = if (state.connection == Connection.DISCONNECTED) onConnect else onDisconnect,
                modifier = Modifier.fillMaxWidth().heightIn(min = 54.dp),
                colors = if (state.connection == Connection.DISCONNECTED) ButtonDefaults.buttonColors()
                else ButtonDefaults.buttonColors(containerColor = CardRaised, contentColor = TextPrimary)
            ) {
                Icon(if (state.connection == Connection.DISCONNECTED) Icons.Filled.PowerSettingsNew else Icons.Filled.StopCircle, null)
                Spacer(Modifier.width(8.dp))
                Text(if (state.connection == Connection.DISCONNECTED) "连接小车" else "断开连接", fontWeight = FontWeight.Bold)
            }
        }

        if (state.connection == Connection.CONNECTED) {
            AppCard {
                Text("车辆在线", color = Success, fontWeight = FontWeight.Bold)
                Text(state.message, color = TextMuted, fontSize = 13.sp)
                Button(onClick = onGoControl, modifier = Modifier.fillMaxWidth()) {
                    Icon(Icons.Filled.SportsEsports, null)
                    Spacer(Modifier.width(8.dp))
                    Text("进入控制")
                }
                OutlinedButton(
                    onClick = onShutdownRequest,
                    enabled = state.canShutdownOrangePi,
                    modifier = Modifier.fillMaxWidth(),
                    colors = ButtonDefaults.outlinedButtonColors(contentColor = Danger),
                    border = BorderStroke(1.dp, Danger.copy(alpha = 0.7f))
                ) {
                    Icon(Icons.Filled.PowerSettingsNew, null)
                    Spacer(Modifier.width(8.dp))
                    Text("关闭香橙派")
                }
                Text("仅在已停车、控制未启用时可执行。", color = TextMuted, fontSize = 11.sp)
            }
        } else {
            Text(
                if (state.controlLink == ControlLink.WIFI)
                    "手机连接 REMOTE AGV 小车热点（SSID：ROVER-ONE）时使用 10.42.0.1；同一 Wi-Fi 下填写香橙派实际局域网地址。"
                else "先在系统蓝牙设置中与 REMOTE AGV（设备名：ROVER-ONE-BLE）配对，再填写设备地址连接。",
                color = TextMuted, fontSize = 12.sp, modifier = Modifier.padding(horizontal = 8.dp)
            )
        }
    }
}

@Composable
private fun ConnectionModeSelector(
    selected: ControlLink,
    enabled: Boolean,
    onSelected: (ControlLink) -> Unit
) {
    Row(
        Modifier.fillMaxWidth().height(52.dp)
            .border(1.dp, Color(0xFF6C7076), RoundedCornerShape(26.dp))
            .padding(3.dp),
        horizontalArrangement = Arrangement.spacedBy(3.dp)
    ) {
        ConnectionModeOption(
            label = "Wi-Fi", icon = Icons.Filled.Wifi,
            selected = selected == ControlLink.WIFI, enabled = enabled,
            modifier = Modifier.weight(1f), onClick = { onSelected(ControlLink.WIFI) }
        )
        ConnectionModeOption(
            label = "BLE 备用", icon = Icons.Filled.Bluetooth,
            selected = selected == ControlLink.BLE, enabled = enabled,
            modifier = Modifier.weight(1f), onClick = { onSelected(ControlLink.BLE) }
        )
    }
}

@Composable
private fun ConnectionModeOption(
    label: String,
    icon: ImageVector,
    selected: Boolean,
    enabled: Boolean,
    modifier: Modifier = Modifier,
    onClick: () -> Unit
) {
    Surface(
        onClick = onClick, enabled = enabled, modifier = modifier.fillMaxHeight(),
        color = if (selected) TealSoft else Color.Transparent,
        contentColor = if (selected) Teal else TextMuted,
        shape = RoundedCornerShape(22.dp)
    ) {
        Row(
            Modifier.fillMaxSize().padding(horizontal = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center
        ) {
            Icon(icon, null, modifier = Modifier.size(19.dp))
            Spacer(Modifier.width(7.dp))
            Text(label, maxLines = 1, fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
        }
    }
}

@Composable
private fun ControlModeSelector(selected: ControlMode, onSelected: (ControlMode) -> Unit) {
    Row(
        Modifier.fillMaxWidth().height(46.dp).background(CardRaised, RoundedCornerShape(15.dp)).padding(3.dp),
        horizontalArrangement = Arrangement.spacedBy(3.dp)
    ) {
        listOf(
            Triple(ControlMode.JOYSTICK, Icons.Filled.Gamepad, "摇杆控制"),
            Triple(ControlMode.THROTTLE, Icons.Filled.VerticalAlignCenter, "油门控制")
        ).forEach { (mode, icon, label) ->
            Surface(
                onClick = { onSelected(mode) }, modifier = Modifier.weight(1f).fillMaxHeight(),
                color = if (selected == mode) TealSoft else Color.Transparent,
                contentColor = if (selected == mode) Teal else TextMuted,
                shape = RoundedCornerShape(12.dp)
            ) {
                Row(
                    Modifier.fillMaxSize(), verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.Center
                ) {
                    Icon(icon, null, modifier = Modifier.size(18.dp))
                    Spacer(Modifier.width(7.dp))
                    Text(label, fontSize = 13.sp, fontWeight = FontWeight.SemiBold, maxLines = 1)
                }
            }
        }
    }
}

@Composable
private fun ControlPage(
    state: RoverUiState, onArm: () -> Unit, onStop: () -> Unit, onEstop: () -> Unit,
    onClearRequest: () -> Unit, onObstacleGuard: (Boolean) -> Unit,
    onVisionFollow: (Boolean, String) -> Unit,
    onStick: (Float, Float) -> Unit, onStrafe: (Float) -> Unit,
    onSpeed: (Float) -> Unit, onWheelTest: (Int, Int) -> Unit,
    onTogglePathRecording: () -> Unit,
    onPathDirection: (PathDirection, Boolean) -> Unit
) {
    var modeName by rememberSaveable { mutableStateOf(ControlMode.JOYSTICK.name) }
    var visionTarget by rememberSaveable { mutableStateOf("red") }
    val mode = ControlMode.valueOf(modeName)
    val vision = state.gateway?.vision
    val visionEnabled = vision?.enabled == true
    fun selectMode(next: ControlMode) {
        if (next == mode) return
        onStick(0f, 0f)
        onStrafe(0f)
        modeName = next.name
    }
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(bottom = 20.dp),
        verticalArrangement = Arrangement.spacedBy(11.dp)
    ) {
        AppCard(contentPadding = PaddingValues(horizontal = 14.dp, vertical = 13.dp)) {
            Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text("超声波自动停车", fontWeight = FontWeight.Bold)
                    Text(
                        when {
                            state.gateway?.obstacleGuardSupported == false && state.gateway?.simulated == false -> "需烧录支持开关的 STM32 固件"
                            state.gateway?.obstacleGuardEnabled == true -> "前方 20 cm 停车；遇障只允许 ≤0.15 m/s 直线后退"
                            else -> "已关闭；急停和通信超时仍有效"
                        },
                        color = if (state.gateway?.obstacleGuardSupported == false && state.gateway?.simulated == false) Warning
                            else if (state.gateway?.obstacleGuardEnabled == true) Success else Warning,
                        fontSize = 11.sp
                    )
                }
                Switch(
                    checked = state.gateway?.obstacleGuardEnabled == true,
                    onCheckedChange = onObstacleGuard,
                    enabled = state.canConfigureObstacleGuard,
                )
            }
            if (!state.canConfigureObstacleGuard) {
                Text("控制启用后不可修改；请先停车并锁定", color = TextMuted, fontSize = 11.sp)
            }
            HorizontalDivider(color = Divider)
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
                    Column(Modifier.weight(1f)) {
                        Text("摄像头目标跟随", fontWeight = FontWeight.Bold)
                        Text(
                            when {
                                vision == null -> "当前网关不支持摄像头跟随"
                                vision?.error != null -> "错误：${vision?.error ?: ""}"
                                visionEnabled && vision?.ready != true -> "正在打开摄像头…"
                                visionEnabled && vision?.found == true -> "已锁定目标 · 横向误差 ${fmt(vision?.horizontalError)}"
                                visionEnabled -> "未找到目标，车辆保持停车"
                                else -> "最高 0.12 m/s；丢失目标立即停车"
                            },
                            color = if (visionEnabled && vision?.found == true) Success
                                else if (vision?.error != null) Danger else TextMuted,
                            fontSize = 11.sp
                        )
                    }
                    Button(
                        onClick = { onVisionFollow(!visionEnabled, if (visionEnabled) vision?.target ?: visionTarget else visionTarget) },
                        enabled = visionEnabled || state.canVisionFollow,
                        colors = if (visionEnabled) ButtonDefaults.buttonColors(containerColor = Danger)
                            else ButtonDefaults.buttonColors()
                    ) { Text(if (visionEnabled) "停止" else "启动") }
                }
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    listOf("red" to "红色", "green" to "绿色", "blue" to "蓝色", "person" to "人员")
                        .forEach { (value, label) ->
                            FilterChip(
                                selected = (if (visionEnabled) vision?.target else visionTarget) == value,
                                onClick = { visionTarget = value },
                                enabled = !visionEnabled,
                                label = { Text(label, fontSize = 11.sp) },
                                modifier = Modifier.weight(1f)
                            )
                        }
                }
                Text("色块模式请使用单一、醒目的专用标志。触发前方障碍后自动跟随只能停车；需要后退时先停止跟随，再用手动控制直线后退。",
                    color = TextMuted, fontSize = 10.sp)
            }
            HorizontalDivider(color = Divider)
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                listOf(0.15f to "精细", 0.30f to "标准", 0.50f to "高速").forEach { (speed, name) ->
                    FilterChip(
                        selected = kotlin.math.abs(state.speedLimit - speed) < 0.01f,
                        onClick = { onSpeed(speed) },
                        label = { Text(name + " " + fmt(speed.toDouble(), 2)) },
                        modifier = Modifier.weight(1f)
                    )
                }
            }
            if (state.recordingPath) {
                PathDirectionControls(state.armed, onPathDirection)
            } else {
                ControlModeSelector(mode, ::selectMode)
                if (mode == ControlMode.JOYSTICK) {
                    Box(Modifier.fillMaxWidth(), contentAlignment = Alignment.Center) {
                        Joystick(state.armed, onStick)
                    }
                } else {
                    ThrottleAndTurnControls(state.armed, onStick)
                }
                StrafeControls(state.armed, onStrafe)
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceAround) {
                CompactMetric("目标线速度", fmt(state.command.linear), "m/s")
                CompactMetric("目标角速度", fmt(state.command.angular), "rad/s")
                CompactMetric("目标横移", fmt(state.command.lateral), "m/s")
            }
            if (state.recordingPath) {
                Button(
                    onClick = onTogglePathRecording,
                    colors = ButtonDefaults.buttonColors(containerColor = Danger, contentColor = Color(0xFF240205)),
                    modifier = Modifier.fillMaxWidth()
                ) { Text("结束路径录制", fontWeight = FontWeight.Bold) }
            }
            SwipeToArm(state.canArm && !state.armed, state.armed, onArm)
        }

        WheelCalibrationPanel(state, onWheelTest)

        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(10.dp)) {
            OutlinedButton(onClick = onStop, modifier = Modifier.weight(1f).heightIn(min = 52.dp)) {
                Text("停车 / 锁定")
            }
            Button(
                onClick = onEstop,
                colors = ButtonDefaults.buttonColors(containerColor = Danger, contentColor = Color(0xFF240205)),
                modifier = Modifier.weight(1f).heightIn(min = 52.dp)
            ) {
                Icon(Icons.Filled.StopCircle, null)
                Spacer(Modifier.width(6.dp))
                Text("急停", fontWeight = FontWeight.Black)
            }
        }
        OutlinedButton(
            onClick = onClearRequest,
            enabled = state.canClear,
            modifier = Modifier.fillMaxWidth().heightIn(min = 50.dp),
            border = BorderStroke(1.dp, if (state.canClear) Teal else Divider),
            colors = ButtonDefaults.outlinedButtonColors(contentColor = if (state.canClear) Teal else TextMuted)
        ) {
            Text(if (state.clearPending) "等待解除确认…" else "解除急停 / 清除故障", fontWeight = FontWeight.Bold)
        }
    }
}

@Composable
private fun WheelCalibrationPanel(state: RoverUiState, onWheelTest: (Int, Int) -> Unit) {
    val gateway = state.gateway
    val enabled = state.connection == Connection.CONNECTED && !state.armed &&
        gateway?.serialReady == true && !gateway.estop && !gateway.faultLatched &&
        !gateway.obstacleBlocked && !state.recordingPath && !state.replayingPath
    val wheels = listOf(
        Triple(0, "C", "左前"), Triple(1, "A", "右前"),
        Triple(2, "B", "左后"), Triple(3, "D", "右后")
    )
    AppCard {
        Text("单轮测试与极性校准", fontWeight = FontWeight.Bold, fontSize = 17.sp)
        Text("先架空四个车轮。每次仅点动一个逻辑轮位，30 RPM、180 ms；正向应使该轮朝车辆前进方向旋转。",
            color = TextMuted, fontSize = 11.sp)
        wheels.chunked(2).forEach { rowWheels ->
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                rowWheels.forEach { (wheel, channel, position) ->
                    Column(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                        Text("$channel · $position", fontWeight = FontWeight.SemiBold)
                        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                            OutlinedButton({ onWheelTest(wheel, 1) }, enabled = enabled,
                                modifier = Modifier.weight(1f)) { Text("正转") }
                            OutlinedButton({ onWheelTest(wheel, -1) }, enabled = enabled,
                                modifier = Modifier.weight(1f)) { Text("反转") }
                        }
                    }
                }
            }
        }
        Text(if (enabled) "可测试：观察实际转向及顶部四路编码器变化" else "请连接小车、停车锁定并清除急停/故障",
            color = if (enabled) Success else Warning, fontSize = 11.sp)
    }
}

@Composable
private fun PathLibraryPage(
    state: RoverUiState,
    onArm: () -> Unit,
    onEstop: () -> Unit,
    onStartRecording: () -> Unit,
    onOpenPath: (String) -> Unit,
    onReplayPath: (String) -> Unit,
    onRenamePath: (String, String) -> Unit,
    onDeletePath: (String) -> Unit
) {
    var renamePathId by remember { mutableStateOf<String?>(null) }
    var renameText by remember { mutableStateOf("") }
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(bottom = 20.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        PageHeading("路径", "相对定位与已保存的操作路径")
        AppCard(contentPadding = PaddingValues(horizontal = 14.dp, vertical = 10.dp)) {
            SwipeToArm(state.canArm && !state.armed, state.armed, onArm)
        }
        AppCard {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
                Column {
                    Text("相对位置估计", fontWeight = FontWeight.Bold, fontSize = 17.sp)
                    Text("四轮编码器 + IMU 航向融合", color = TextMuted, fontSize = 11.sp)
                }
                Text(
                    if (state.pose.ready) "● 可用" else "等待遥测",
                    color = if (state.pose.ready) Success else TextMuted,
                    fontSize = 12.sp, fontWeight = FontWeight.Bold
                )
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceAround) {
                CompactMetric("前向 X", fmt(state.pose.xMetres, 2), "m")
                CompactMetric("左向 Y", fmt(state.pose.yMetres, 2), "m")
                CompactMetric("相对航向", headingDegrees(state.pose.headingRad), "deg")
            }
            Text(
                if (state.pose.ready) "置信度 ${(state.pose.confidence * 100.0).toInt()}% · 起点为本次连接或复现开始处"
                else "等待四路编码器与 IMU 航向数据",
                color = if (state.pose.ready) Success else TextMuted, fontSize = 11.sp
            )
            Text("这是相对推算；车轮打滑、碰撞或抬起都会累积误差。", color = TextMuted, fontSize = 11.sp)
        }

        Button(
            onClick = onStartRecording,
            enabled = state.canRecordPath,
            modifier = Modifier.fillMaxWidth().heightIn(min = 54.dp)
        ) {
            Text("开始录制路径", fontWeight = FontWeight.Bold)
        }

        AppCard {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
                Column {
                    Text("已保存路径", fontWeight = FontWeight.Bold, fontSize = 17.sp)
                    Text("点击任一路径进入编辑与试验复现", color = TextMuted, fontSize = 11.sp)
                }
                Text(
                    when {
                        state.recordingPath -> "录制中"
                        state.recordedPaths.isNotEmpty() -> "${state.recordedPaths.size} 条"
                        else -> "暂无路径"
                    },
                    color = when {
                        state.recordingPath -> Success
                        state.recordedPaths.isNotEmpty() -> Teal
                        else -> TextMuted
                    }, fontWeight = FontWeight.Bold, fontSize = 12.sp
                )
            }
            when {
                state.recordingPath -> {
                    Surface(color = Success.copy(alpha = 0.12f), shape = RoundedCornerShape(16.dp)) {
                        Text("正在记录方向按键。结束后会以当前时间自动命名并保存为一条新路径。",
                            color = Success, fontSize = 12.sp, modifier = Modifier.padding(13.dp))
                    }
                }
                state.recordedPaths.isEmpty() -> {
                    Surface(color = CardRaised, shape = RoundedCornerShape(16.dp)) {
                        Text("点击上方“开始录制路径”后会自动进入控制页。初代录制只接受前进、后退、左转、右转和左右平移按键。",
                            color = TextMuted, fontSize = 12.sp, modifier = Modifier.padding(13.dp))
                    }
                }
                else -> {
                    state.recordedPaths.asReversed().forEach { path ->
                        Surface(
                            onClick = { onOpenPath(path.id) }, color = CardRaised,
                            shape = RoundedCornerShape(16.dp), modifier = Modifier.fillMaxWidth()
                        ) {
                            Column(Modifier.padding(13.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                                    Text(path.name, fontWeight = FontWeight.Bold, maxLines = 1, overflow = TextOverflow.Ellipsis,
                                        modifier = Modifier.weight(1f))
                                    Text("${path.segments.size} 段", color = Teal, fontSize = 12.sp)
                                }
                                Text(
                                    "总时长 ${fmt(path.segments.sumOf { it.durationMs } / 1000.0, 1)} s · 点击编辑",
                                    color = TextMuted, fontSize = 11.sp
                                )
                                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
                                    Button(
                                        onClick = { if (state.replayingPath) onEstop() else onReplayPath(path.id) },
                                        enabled = state.replayingPath || state.canReplayPath,
                                        colors = if (state.replayingPath) ButtonDefaults.buttonColors(
                                            containerColor = Danger, contentColor = Color(0xFF240205)
                                        ) else ButtonDefaults.buttonColors(),
                                        contentPadding = PaddingValues(horizontal = 13.dp, vertical = 0.dp)
                                    ) {
                                        if (state.replayingPath) Icon(Icons.Filled.StopCircle, null, Modifier.size(16.dp))
                                        Text(if (state.replayingPath) "急停" else "复现路径", fontSize = 11.sp)
                                    }
                                    Row {
                                        TextButton(onClick = { renamePathId = path.id; renameText = path.name }) {
                                            Text("改名", fontSize = 11.sp)
                                        }
                                        TextButton(onClick = { onDeletePath(path.id) }) {
                                            Text("删除", color = Danger, fontSize = 11.sp)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    val renameId = renamePathId
    if (renameId != null) AlertDialog(
        onDismissRequest = { renamePathId = null }, title = { Text("修改路径名称") },
        text = { OutlinedTextField(renameText, { renameText = it }, label = { Text("路径名称") }, singleLine = true) },
        confirmButton = { TextButton(onClick = { onRenamePath(renameId, renameText); renamePathId = null },
            enabled = renameText.trim().isNotEmpty()) { Text("保存") } },
        dismissButton = { TextButton(onClick = { renamePathId = null }) { Text("取消") } }
    )
}

@Composable
private fun PathEditorPage(
    state: RoverUiState, path: RecordedPath, onBack: () -> Unit, onArm: () -> Unit,
    onEstop: () -> Unit, onReplayPath: (String) -> Unit,
    onSetDuration: (String, Int, Long) -> Unit, onAddParkingAfter: (String, Int, Long) -> Unit,
    onDeleteSegment: (String, Int) -> Unit, onRenamePath: (String, String) -> Unit
) {
    var durationTarget by remember { mutableIntStateOf(-1) }
    var parkingAfter by remember { mutableIntStateOf(-1) }
    var durationText by remember { mutableStateOf("") }
    var renameOpen by remember { mutableStateOf(false) }
    var renameText by remember(path.id) { mutableStateOf(path.name) }
    fun editDuration(index: Int) { durationTarget = index; parkingAfter = -1; durationText = fmt(path.segments[index].durationMs / 1000.0, 2) }
    fun insertParking(index: Int) { parkingAfter = index; durationTarget = -1; durationText = "1.00" }
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(bottom = 20.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = onBack) { Text("‹ 路径列表") }
            Text(if (state.replayingPath) "复现中" else "编辑", color = if (state.replayingPath) Success else Teal,
                fontWeight = FontWeight.Bold, fontSize = 12.sp)
        }
        AppCard {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text(path.name, fontWeight = FontWeight.Black, fontSize = 21.sp, maxLines = 1, overflow = TextOverflow.Ellipsis)
                    Text("${path.segments.size} 个操作 · ${fmt(path.segments.sumOf { it.durationMs } / 1000.0, 1)} s", color = TextMuted, fontSize = 11.sp)
                }
                TextButton(onClick = { renameOpen = true }, enabled = !state.replayingPath) { Text("改名") }
            }
            SwipeToArm(state.canArm && !state.armed, state.armed, onArm)
            Button(
                onClick = { if (state.replayingPath) onEstop() else onReplayPath(path.id) },
                enabled = state.replayingPath || state.canReplayPath,
                colors = if (state.replayingPath) ButtonDefaults.buttonColors(
                    containerColor = Danger, contentColor = Color(0xFF240205)
                ) else ButtonDefaults.buttonColors(),
                modifier = Modifier.fillMaxWidth().heightIn(min = 50.dp)
            ) {
                if (state.replayingPath) Icon(Icons.Filled.StopCircle, null)
                Text(if (state.replayingPath) "急停" else "试验复现路径", fontWeight = FontWeight.Bold)
            }
            Text("试验会按当前编辑内容执行；结束、停车或急停后自动锁定。", color = TextMuted, fontSize = 11.sp)
        }
        Text("操作流程", fontWeight = FontWeight.Bold, modifier = Modifier.padding(start = 4.dp))
        Surface(color = TealSoft, shape = RoundedCornerShape(14.dp), modifier = Modifier.fillMaxWidth()) {
            Text("● 起点  相对坐标 (0.00, 0.00)", color = Teal, fontSize = 12.sp, fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(horizontal = 13.dp, vertical = 10.dp))
        }
        path.segments.forEachIndexed { index, segment ->
            Text("↓", color = Teal, modifier = Modifier.fillMaxWidth(), textAlign = androidx.compose.ui.text.style.TextAlign.Center)
            Surface(color = CardRaised, shape = RoundedCornerShape(16.dp), modifier = Modifier.fillMaxWidth()) {
                Column(Modifier.padding(13.dp), verticalArrangement = Arrangement.spacedBy(7.dp)) {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text("${index + 1}. ${segment.label()}", fontWeight = FontWeight.Bold)
                        TextButton(onClick = { editDuration(index) }, enabled = !state.replayingPath,
                            contentPadding = PaddingValues(horizontal = 5.dp, vertical = 0.dp)) {
                            Text("${fmt(segment.durationMs / 1000.0, 2)} s", color = Teal, fontFamily = FontFamily.Monospace)
                        }
                    }
                    Text(
                        if (segment.drive == Drive()) "保持停车，结束后再执行下一操作"
                        else "预计到达：X ${fmt(segment.endPose.xMetres, 2)} m · Y ${fmt(segment.endPose.yMetres, 2)} m · ${headingDegrees(segment.endPose.headingRad)}",
                        color = TextMuted, fontSize = 11.sp
                    )
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.End) {
                        TextButton(onClick = { onDeleteSegment(path.id, index) }, enabled = !state.replayingPath) {
                            Text("删除操作", color = Danger, fontSize = 11.sp)
                        }
                    }
                }
            }
            if (index < path.segments.lastIndex) {
                TextButton(onClick = { insertParking(index) }, enabled = !state.replayingPath,
                    modifier = Modifier.fillMaxWidth()) { Text("＋ 在此后加入驻车时间", color = Teal, fontSize = 12.sp) }
            }
        }
        Text("↓", color = Teal, modifier = Modifier.fillMaxWidth(), textAlign = androidx.compose.ui.text.style.TextAlign.Center)
        Surface(color = TealSoft, shape = RoundedCornerShape(14.dp), modifier = Modifier.fillMaxWidth()) {
            Text("■ 终点", color = Teal, fontSize = 12.sp, fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(horizontal = 13.dp, vertical = 10.dp))
        }
    }
    if (durationTarget >= 0 || parkingAfter >= 0) {
        val seconds = durationText.toDoubleOrNull()
        AlertDialog(
            onDismissRequest = { durationTarget = -1; parkingAfter = -1 },
            title = { Text(if (parkingAfter >= 0) "加入驻车时间" else "设置操作时间") },
            text = { OutlinedTextField(durationText, { durationText = it }, label = { Text("秒（0.12 到 300）") }, singleLine = true) },
            confirmButton = {
                TextButton(
                    onClick = {
                        val duration = (seconds!! * 1000.0).toLong()
                        if (durationTarget >= 0) onSetDuration(path.id, durationTarget, duration)
                        else onAddParkingAfter(path.id, parkingAfter, duration)
                        durationTarget = -1; parkingAfter = -1
                    }, enabled = seconds != null && seconds in 0.12..300.0
                ) { Text("保存") }
            },
            dismissButton = { TextButton(onClick = { durationTarget = -1; parkingAfter = -1 }) { Text("取消") } }
        )
    }
    if (renameOpen) AlertDialog(
        onDismissRequest = { renameOpen = false }, title = { Text("修改路径名称") },
        text = { OutlinedTextField(renameText, { renameText = it }, label = { Text("路径名称") }, singleLine = true) },
        confirmButton = { TextButton(onClick = { onRenamePath(path.id, renameText); renameOpen = false },
            enabled = renameText.trim().isNotEmpty()) { Text("保存") } },
        dismissButton = { TextButton(onClick = { renameOpen = false }) { Text("取消") } }
    )
}

@Composable
private fun RecordsPage(state: RoverUiState) {
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(bottom = 20.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        PageHeading("运行记录", "连接事件、指令确认与车辆遥测")
        AppCard {
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
                Text("当前车辆状态", fontWeight = FontWeight.Bold, fontSize = 17.sp)
                val fresh = state.telemetryAgeMs?.let { it < 500 } == true
                Text(
                    if (fresh) "● 实时" else if (state.connection == Connection.CONNECTED) "等待遥测" else "未连接",
                    color = if (fresh) Success else TextMuted, fontSize = 12.sp
                )
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceAround) {
                CompactMetric("电池电压", fmt(batteryVoltage(state), 1), "V")
                CompactMetric("超声距离", fmt(state.telemetry?.distance, 2), "m")
                CompactMetric("IMU 航向", headingDegrees(state.telemetry?.yaw), "deg")
            }
            HorizontalDivider(color = Divider)
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceAround) {
                CompactMetric("线速度", fmt(state.telemetry?.linear, 2), "m/s")
                CompactMetric("角速度", fmt(state.telemetry?.angular, 2), "rad/s")
                CompactMetric("故障码", state.gateway?.faultCode?.toString() ?: "—", "code")
            }
            Text("编码器 FL / FR / RL / RR", color = TextMuted, fontSize = 11.sp)
            Text(
                state.telemetry?.encoders?.joinToString("  /  ") { it?.toString() ?: "—" }
                    ?: "—  /  —  /  —  /  —",
                fontFamily = FontFamily.Monospace, fontSize = 13.sp
            )
            Text(state.lastAck, color = TextMuted, fontSize = 11.sp)
        }

        Text("事件记录", fontWeight = FontWeight.Bold, modifier = Modifier.padding(start = 4.dp, top = 2.dp))
        if (state.logs.isEmpty()) {
            AppCard { Text("暂无记录", color = TextMuted, modifier = Modifier.fillMaxWidth()) }
        } else {
            state.logs.forEachIndexed { index, entry ->
                Surface(color = Card, shape = RoundedCornerShape(18.dp), modifier = Modifier.fillMaxWidth()) {
                    Row(
                        Modifier.padding(horizontal = 15.dp, vertical = 13.dp),
                        horizontalArrangement = Arrangement.spacedBy(11.dp), verticalAlignment = Alignment.Top
                    ) {
                        Surface(
                            color = if (index == 0) Teal else CardRaised,
                            shape = CircleShape, modifier = Modifier.size(8.dp).padding(top = 3.dp)
                        ) {}
                        Column {
                            Text(
                                if (index == 0) "最新事件" else "历史事件",
                                color = if (index == 0) Teal else TextMuted, fontSize = 10.sp
                            )
                            Text(entry, fontSize = 13.sp, lineHeight = 19.sp)
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun PageHeading(title: String, subtitle: String) {
    Column(Modifier.padding(horizontal = 4.dp, vertical = 4.dp)) {
        Text(title, fontSize = 25.sp, fontWeight = FontWeight.Black)
        Text(subtitle, color = TextMuted, fontSize = 12.sp)
    }
}

@Composable
private fun AppCard(
    contentPadding: PaddingValues = PaddingValues(16.dp),
    content: @Composable ColumnScope.() -> Unit
) {
    Surface(color = Card, shape = RoundedCornerShape(24.dp), modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(contentPadding), verticalArrangement = Arrangement.spacedBy(11.dp), content = content)
    }
}

@Composable
private fun CompactMetric(label: String, value: String, unit: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(label, color = TextMuted, fontSize = 10.sp)
        Text(value, fontSize = 18.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold, maxLines = 1)
        Text(unit, color = TextMuted, fontSize = 9.sp)
    }
}

@Composable
private fun FloatingNavigation(selected: AppPage, onSelected: (AppPage) -> Unit) {
    Box(
        Modifier.fillMaxWidth().navigationBarsPadding().padding(horizontal = 18.dp, vertical = 10.dp),
        contentAlignment = Alignment.Center
    ) {
        Surface(
            color = NavGlass, shape = RoundedCornerShape(34.dp), shadowElevation = 18.dp,
            modifier = Modifier.fillMaxWidth()
                .border(1.dp, Color(0xFF5474A8), RoundedCornerShape(34.dp))
                .border(0.5.dp, Color(0x665E9BFF), RoundedCornerShape(33.dp))
        ) {
            Row(
                Modifier.fillMaxWidth().height(68.dp).padding(6.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                NavigationItem("连接", Icons.Filled.Wifi, selected == AppPage.CONNECTION, Modifier.weight(1f)) {
                    onSelected(AppPage.CONNECTION)
                }
                NavigationItem("控制", Icons.Filled.SportsEsports, selected == AppPage.CONTROL, Modifier.weight(1f)) {
                    onSelected(AppPage.CONTROL)
                }
                NavigationItem("路径", Icons.Filled.Router, selected == AppPage.PATH, Modifier.weight(1f)) {
                    onSelected(AppPage.PATH)
                }
                NavigationItem("记录", Icons.AutoMirrored.Filled.Article, selected == AppPage.RECORDS, Modifier.weight(1f)) {
                    onSelected(AppPage.RECORDS)
                }
            }
        }
    }
}

@Composable
private fun NavigationItem(
    label: String, icon: ImageVector, selected: Boolean, modifier: Modifier = Modifier, onClick: () -> Unit
) {
    Surface(
        onClick = onClick, color = if (selected) Teal else Color.Transparent,
        contentColor = if (selected) Color(0xFF06211E) else TextPrimary,
        shape = RoundedCornerShape(27.dp), modifier = modifier.fillMaxHeight()
    ) {
        Column(
            Modifier.fillMaxSize(), horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center
        ) {
            Icon(icon, label, modifier = Modifier.size(if (selected) 25.dp else 22.dp))
            Text(label, fontSize = 11.sp, fontWeight = if (selected) FontWeight.Bold else FontWeight.Medium)
        }
    }
}

@Composable
private fun SwipeToArm(enabled: Boolean, armed: Boolean, onArm: () -> Unit) {
    var progress by remember { mutableFloatStateOf(0f) }
    val latest by rememberUpdatedState(onArm)
    LaunchedEffect(armed) { if (armed) progress = 1f else progress = 0f }
    Surface(
        modifier = Modifier.fillMaxWidth().heightIn(min = 52.dp)
            .semantics { contentDescription = "向右滑动启用控制" }
            .pointerInput(enabled, armed) {
                if (enabled && !armed) awaitEachGesture {
                    val down = awaitFirstDown(false, PointerEventPass.Initial)
                    down.consume()
                    val startX = down.position.x
                    var completed = false
                    try {
                        while (true) {
                            val event = awaitPointerEvent(PointerEventPass.Initial)
                            val point = event.changes.find { it.id == down.id } ?: break
                            point.consume()
                            progress = ((point.position.x - startX) / (size.width * 0.72f)).coerceIn(0f, 1f)
                            if (progress >= 0.82f) {
                                completed = true
                                latest()
                                break
                            }
                            if (!point.pressed) break
                        }
                    } finally { if (!completed) progress = 0f }
                }
            },
        shape = RoundedCornerShape(50), color = if (armed) Success else if (enabled) Teal else CardRaised
    ) {
        BoxWithConstraints(Modifier.fillMaxWidth().padding(horizontal = 5.dp, vertical = 4.dp)) {
            val thumb = 44.dp
            val thumbOffset = (maxWidth - thumb) * progress
            Surface(
                color = if (armed) Success else if (enabled) Color(0xFF72A6FF) else Divider,
                shape = CircleShape,
                modifier = Modifier.align(Alignment.CenterStart).offset(x = thumbOffset).size(thumb)
            ) {
                Box(contentAlignment = Alignment.Center) {
                    Icon(Icons.AutoMirrored.Filled.ArrowForward, null, tint = Color(0xFF06211E))
                }
            }
            Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                Text(
                    if (armed) "控制已启用" else if (progress > 0f) "继续向右滑动…" else "向右滑动启用控制",
                    color = if (enabled || armed) Color(0xFF06211E) else TextMuted,
                    fontWeight = FontWeight.Bold, lineHeight = 18.sp,
                    textAlign = androidx.compose.ui.text.style.TextAlign.Center,
                    modifier = Modifier.fillMaxWidth()
                )
            }
        }
    }
}

@Composable
private fun ThrottleAndTurnControls(enabled: Boolean, onStick: (Float, Float) -> Unit) {
    var throttle by remember { mutableFloatStateOf(0f) }
    var turn by remember { mutableFloatStateOf(0f) }
    val latest by rememberUpdatedState(onStick)
    fun emit(nextTurn: Float = turn, nextThrottle: Float = throttle) {
        turn = nextTurn
        throttle = nextThrottle
        latest(turn, throttle)
    }
    LaunchedEffect(enabled) {
        if (!enabled) emit(0f, 0f)
    }
    DisposableEffect(Unit) { onDispose { latest(0f, 0f) } }

    Row(
        Modifier.fillMaxWidth().height(232.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceEvenly
    ) {
        MomentaryControlButton(
            enabled = enabled, label = "左转", icon = Icons.AutoMirrored.Filled.ArrowBack,
            modifier = Modifier.size(76.dp), onPressed = { emit(if (it) -1f else 0f) }
        )
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text("前进", color = TextMuted, fontSize = 11.sp)
            VerticalThrottle(
                enabled = enabled,
                value = throttle,
                onValue = { emit(nextThrottle = it) }
            )
            Text("后退", color = TextMuted, fontSize = 11.sp)
        }
        MomentaryControlButton(
            enabled = enabled, label = "右转", icon = Icons.AutoMirrored.Filled.ArrowForward,
            modifier = Modifier.size(76.dp), onPressed = { emit(if (it) 1f else 0f) }
        )
    }
}

@Composable
private fun VerticalThrottle(enabled: Boolean, value: Float, onValue: (Float) -> Unit) {
    val latest by rememberUpdatedState(onValue)
    Canvas(
        Modifier.width(82.dp).height(192.dp)
            .semantics { contentDescription = "前后油门，上推前进，下拉后退，松手归零" }
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                val travel = size.height * 0.39f
                val center = size.height / 2f
                fun update(y: Float) { latest(((center - y) / travel).coerceIn(-1f, 1f)) }
                try {
                    awaitEachGesture {
                        val down = awaitFirstDown(false, PointerEventPass.Initial)
                        down.consume()
                        update(down.position.y)
                        try {
                            while (true) {
                                val event = awaitPointerEvent(PointerEventPass.Initial)
                                val point = event.changes.find { it.id == down.id }
                                event.changes.forEach { it.consume() }
                                if (point == null || !point.pressed || event.changes.count { it.pressed } > 1) break
                                update(point.position.y)
                            }
                        } finally { latest(0f) }
                    }
                } finally { latest(0f) }
            }
    ) {
        val center = Offset(size.width / 2f, size.height / 2f)
        val travel = size.height * 0.39f
        val knob = Offset(center.x, center.y - value.coerceIn(-1f, 1f) * travel)
        drawRoundRect(
            color = Color(0xFF121416),
            topLeft = Offset(size.width * 0.24f, 0f),
            size = androidx.compose.ui.geometry.Size(size.width * 0.52f, size.height),
            cornerRadius = androidx.compose.ui.geometry.CornerRadius(size.width * 0.26f)
        )
        drawLine(Divider, Offset(center.x, 12.dp.toPx()), Offset(center.x, size.height - 12.dp.toPx()), 3.dp.toPx())
        drawLine(Teal.copy(alpha = 0.65f), center, knob, 7.dp.toPx())
        drawCircle(if (enabled) Teal.copy(alpha = 0.18f) else CardRaised, 28.dp.toPx(), knob)
        drawCircle(if (enabled) Teal else Color(0xFF5B5E62), 19.dp.toPx(), knob)
    }
}

@Composable
private fun PathDirectionControls(
    enabled: Boolean,
    onDirection: (PathDirection, Boolean) -> Unit
) {
    Column(
        Modifier.fillMaxWidth(),
        verticalArrangement = Arrangement.spacedBy(9.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("录制方向按键", color = Teal, fontWeight = FontWeight.Bold, fontSize = 12.sp)
        MomentaryControlButton(
            enabled = enabled, label = "前进", icon = Icons.AutoMirrored.Filled.ArrowForward,
            modifier = Modifier.width(156.dp).height(50.dp),
            onPressed = { onDirection(PathDirection.FORWARD, it) }
        )
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(9.dp)) {
            MomentaryControlButton(
                enabled = enabled, label = "左转", icon = Icons.AutoMirrored.Filled.ArrowBack,
                modifier = Modifier.weight(1f).height(50.dp),
                onPressed = { onDirection(PathDirection.TURN_LEFT, it) }
            )
            MomentaryControlButton(
                enabled = enabled, label = "右转", icon = Icons.AutoMirrored.Filled.ArrowForward,
                modifier = Modifier.weight(1f).height(50.dp),
                onPressed = { onDirection(PathDirection.TURN_RIGHT, it) }
            )
        }
        MomentaryControlButton(
            enabled = enabled, label = "后退", icon = Icons.AutoMirrored.Filled.ArrowBack,
            modifier = Modifier.width(156.dp).height(50.dp),
            onPressed = { onDirection(PathDirection.BACKWARD, it) }
        )
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(9.dp)) {
            MomentaryControlButton(
                enabled = enabled, label = "左平移", icon = Icons.AutoMirrored.Filled.ArrowBack,
                modifier = Modifier.weight(1f).height(50.dp),
                onPressed = { onDirection(PathDirection.STRAFE_LEFT, it) }
            )
            MomentaryControlButton(
                enabled = enabled, label = "右平移", icon = Icons.AutoMirrored.Filled.ArrowForward,
                modifier = Modifier.weight(1f).height(50.dp),
                onPressed = { onDirection(PathDirection.STRAFE_RIGHT, it) }
            )
        }
    }
}

@Composable
private fun StrafeControls(enabled: Boolean, onStrafe: (Float) -> Unit) {
    Row(
        Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(10.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        MomentaryControlButton(
            enabled = enabled, label = "左平移", icon = Icons.AutoMirrored.Filled.ArrowBack,
            modifier = Modifier.weight(1f).height(52.dp), onPressed = { onStrafe(if (it) -1f else 0f) }
        )
        Surface(color = TealSoft, shape = CircleShape, modifier = Modifier.size(42.dp)) {
            Box(contentAlignment = Alignment.Center) {
                Icon(Icons.Filled.SwapHoriz, null, tint = Teal, modifier = Modifier.size(23.dp))
            }
        }
        MomentaryControlButton(
            enabled = enabled, label = "右平移", icon = Icons.AutoMirrored.Filled.ArrowForward,
            modifier = Modifier.weight(1f).height(52.dp), onPressed = { onStrafe(if (it) 1f else 0f) }
        )
    }
}

@Composable
private fun MomentaryControlButton(
    enabled: Boolean,
    label: String,
    icon: ImageVector,
    modifier: Modifier = Modifier,
    onPressed: (Boolean) -> Unit
) {
    var pressed by remember { mutableStateOf(false) }
    val latest by rememberUpdatedState(onPressed)
    LaunchedEffect(enabled) {
        if (!enabled && pressed) { pressed = false; latest(false) }
    }
    Surface(
        modifier = modifier
            .semantics { contentDescription = "$label，按住动作，松手停车" }
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                try {
                    awaitEachGesture {
                        val down = awaitFirstDown(false, PointerEventPass.Initial)
                        down.consume()
                        pressed = true
                        latest(true)
                        try {
                            while (true) {
                                val event = awaitPointerEvent(PointerEventPass.Initial)
                                val point = event.changes.find { it.id == down.id }
                                event.changes.forEach { it.consume() }
                                if (point == null || !point.pressed) break
                            }
                        } finally { pressed = false; latest(false) }
                    }
                } finally { pressed = false; latest(false) }
            },
        shape = RoundedCornerShape(16.dp),
        color = if (pressed && enabled) Teal else CardRaised,
        contentColor = if (pressed && enabled) Color(0xFF06211E) else if (enabled) TextPrimary else TextMuted,
        border = BorderStroke(1.dp, if (pressed && enabled) Teal else Divider)
    ) {
        Row(
            Modifier.fillMaxSize().padding(horizontal = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center
        ) {
            Icon(icon, null, modifier = Modifier.size(19.dp))
            Spacer(Modifier.width(6.dp))
            Text(label, fontSize = 12.sp, fontWeight = FontWeight.Bold, maxLines = 1)
        }
    }
}

@Composable
private fun Joystick(enabled: Boolean, onStick: (Float, Float) -> Unit) {
    var knob by remember { mutableStateOf(Offset.Zero) }
    val latest by rememberUpdatedState(onStick)
    LaunchedEffect(enabled) { if (!enabled) { knob = Offset.Zero; latest(0f, 0f) } }
    DisposableEffect(Unit) { onDispose { latest(0f, 0f) } }
    Canvas(
        Modifier.size(218.dp)
            .semantics { contentDescription = "驾驶摇杆，上为前进，下为后退，松手停车" }
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                val radius = minOf(size.width, size.height) / 2f * 0.70f
                val center = Offset(size.width / 2f, size.height / 2f)
                fun update(position: Offset) {
                    val delta = position - center
                    val length = sqrt(delta.x * delta.x + delta.y * delta.y)
                    knob = if (length > radius) delta * (radius / length) else delta
                    latest(knob.x / radius, -knob.y / radius)
                }
                try {
                    awaitEachGesture {
                        val down = awaitFirstDown(false, PointerEventPass.Initial)
                        down.consume()
                        update(down.position)
                        try {
                            while (true) {
                                val event = awaitPointerEvent(PointerEventPass.Initial)
                                val point = event.changes.find { it.id == down.id }
                                event.changes.forEach { it.consume() }
                                if (point == null || !point.pressed || event.changes.count { it.pressed } > 1) break
                                update(point.position)
                            }
                        } finally { knob = Offset.Zero; latest(0f, 0f) }
                    }
                } finally { knob = Offset.Zero; latest(0f, 0f) }
            }
    ) {
        val radius = size.minDimension / 2f
        val center = Offset(size.width / 2f, size.height / 2f)
        drawCircle(Color(0xFF121416), radius * 0.96f)
        drawCircle(Color(0xFF3B3D40), radius * 0.89f, style = Stroke(1.4.dp.toPx()))
        drawCircle(Color(0xFF2D3033), radius * 0.55f, style = Stroke(1.dp.toPx()))
        drawLine(Divider, Offset(center.x, 16.dp.toPx()), Offset(center.x, size.height - 16.dp.toPx()))
        drawLine(Divider, Offset(16.dp.toPx(), center.y), Offset(size.width - 16.dp.toPx(), center.y))
        if (enabled) drawLine(Teal.copy(alpha = 0.55f), center, center + knob, strokeWidth = 3.dp.toPx())
        drawCircle(if (enabled) Teal.copy(alpha = 0.16f) else CardRaised, radius * 0.30f, center + knob)
        drawCircle(if (enabled) Teal else Color(0xFF5B5E62), radius * 0.21f, center + knob)
        drawCircle(if (enabled) Color(0xFF06211E) else TextMuted, radius * 0.05f, center + knob)
    }
}
