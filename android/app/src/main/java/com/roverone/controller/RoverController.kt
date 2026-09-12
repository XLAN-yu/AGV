package com.roverone.controller

import android.app.Application
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import okhttp3.OkHttpClient
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

class RoverController(application: Application) : AndroidViewModel(application) {
    private data class MotionPulse(val armed: Boolean = false, val drive: Drive = Drive(),
        val telemetryAt: Long? = null)
    private val prefs = application.getSharedPreferences("rover_connection", 0)
    private val mutable = MutableStateFlow(RoverUiState(
        address = prefs.getString("address", "http://10.42.0.1")!!,
        bleAddress = prefs.getString("ble_address", "")!!,
        controlLink = runCatching { ControlLink.valueOf(prefs.getString("control_link", ControlLink.WIFI.name)!!) }
            .getOrDefault(ControlLink.WIFI)
    ))
    val state = mutable.asStateFlow()
    private val main = Handler(Looper.getMainLooper())
    private val connectivity = application.getSystemService(ConnectivityManager::class.java)
    private var engine = SafetyEngine(false)
    private val poseEstimator = FuzzyPoseEstimator()
    private val pathRecorder = PathRecorder(PathStore(application))
    @Volatile private var transport: TextTransport? = null
    private var ticker: Job? = null
    private var replayJob: Job? = null
    private var replayingPath = false
    @Volatile private var session = 0L
    private val motionPulse = AtomicReference(MotionPulse())
    private val driveLimiter = DriveSlewLimiter()
    private val uiRefreshQueued = AtomicBoolean(false)
    private var selectedNetwork: Network? = null
    private var callback: ConnectivityManager.NetworkCallback? = null
    private var rememberedEstop = false
    private var focused = false
    private val http = OkHttpClient.Builder().connectTimeout(5, TimeUnit.SECONDS)
        .readTimeout(0, TimeUnit.MILLISECONDS).pingInterval(3, TimeUnit.SECONDS).build()
    private fun now() = SystemClock.elapsedRealtime()
    private fun log(text: String) {
        mutable.value = mutable.value.copy(message = text, logs = (listOf(text) + mutable.value.logs).take(30))
    }
    private fun refresh() {
        val t = now()
        val live = mutable.value.connection == Connection.CONNECTED
        val paths = pathRecorder.paths
        mutable.value = mutable.value.copy(
            gateway = if (live) engine.gateway else null, telemetry = if (live) engine.telemetry else null,
            telemetryAgeMs = if (live) engine.lastTelemetryAt?.let { (t - it).coerceAtLeast(0) } else null,
            armed = live && engine.armed,
            canArm = live && engine.blockedReason(t) == null && engine.gateway?.vision?.enabled != true,
            canClear = live && engine.canClear(t),
            clearPending = live && engine.pendingClear != null,
            canConfigureObstacleGuard = live && engine.canConfigureObstacleGuard(t),
            obstacleGuardPending = live && engine.pendingObstacleGuard != null,
            canVisionFollow = live && !state.value.simulation && !engine.armed &&
                !pathRecorder.recording && !replayingPath && engine.blockedReason(t) == null &&
                engine.gateway?.vision != null,
            canShutdownOrangePi = live && engine.canShutdownOrangePi(t),
            pose = poseEstimator.pose, recordedPaths = paths,
            recordingPath = pathRecorder.recording, replayingPath = replayingPath,
            canRecordPath = live && !state.value.simulation && poseEstimator.pose.ready &&
                !pathRecorder.recording && !replayingPath,
            canReplayPath = live && !state.value.simulation && !pathRecorder.recording && !replayingPath &&
                engine.blockedReason(t) == null && !engine.obstacleRetreatOnly() &&
                paths.any { it.segments.isNotEmpty() },
            command = engine.command
        )
        publishMotion()
    }
    private fun publishMotion() {
        val live = mutable.value.connection == Connection.CONNECTED
        motionPulse.set(MotionPulse(live && engine.armed, engine.command, engine.lastTelemetryAt))
    }
    fun address(value: String) {
        if (state.value.connection == Connection.DISCONNECTED) mutable.value = state.value.copy(address = value.take(180))
    }
    fun simulation(value: Boolean) {
        if (state.value.connection == Connection.DISCONNECTED && state.value.controlLink == ControlLink.WIFI)
            mutable.value = state.value.copy(simulation = value)
    }
    fun controlLink(value: ControlLink) {
        if (state.value.connection != Connection.DISCONNECTED) return
        prefs.edit().putString("control_link", value.name).apply()
        mutable.value = state.value.copy(controlLink = value, simulation = if (value == ControlLink.BLE) false else state.value.simulation)
    }
    fun bleAddress(value: String) {
        if (state.value.connection == Connection.DISCONNECTED)
            mutable.value = state.value.copy(bleAddress = value.trim().uppercase().take(17))
    }
    fun speed(value: Float) {
        if (value.isFinite()) {
            val speed = value.coerceIn(0.05f, 0.5f)
            val before = engine.command
            engine.updateSpeed(speed, now())
            mutable.value = state.value.copy(speedLimit = speed)
            if (engine.armed && engine.command == Drive() && before != Drive()) zero()
            if (pathRecorder.recording) pathRecorder.transition(engine.command, poseEstimator.pose, now())
            publishMotion()
            refresh()
        }
    }
    fun connect() {
        if (state.value.connection != Connection.DISCONNECTED) return
        val useBle = state.value.controlLink == ControlLink.BLE
        val endpoint = if (!useBle) try { Endpoint.parse(state.value.address) }
            catch (e: Exception) { log(e.message ?: "地址无效"); return } else null
        if (useBle && !state.value.bleAddress.matches(Regex("(?i)([0-9a-f]{2}:){5}[0-9a-f]{2}"))) {
            log("请输入已配对香橙派的 BLE 地址"); return
        }
        if (useBle) prefs.edit().putString("ble_address", state.value.bleAddress).apply()
        else prefs.edit().putString("address", endpoint!!.origin).apply()
        engine = SafetyEngine(state.value.simulation)
        poseEstimator.reset()
        if (rememberedEstop) engine.estop()
        mutable.value = state.value.copy(address = endpoint?.origin ?: state.value.address, connection = Connection.CONNECTING,
            gateway = null, telemetry = null, telemetryAgeMs = null, lastAck = "尚无确认")
        val token = ++session
        try {
            val wifi = if (useBle) null else connectivity.allNetworks.firstOrNull {
                connectivity.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) == true
            }
            val wired = if (useBle) null else connectivity.allNetworks.firstOrNull {
                connectivity.getNetworkCapabilities(it)?.hasTransport(NetworkCapabilities.TRANSPORT_ETHERNET) == true
            }
            selectedNetwork = if (useBle) null else wifi ?: wired
            val emulator = Build.FINGERPRINT.contains("generic") || Build.MODEL.contains("Emulator") || Build.MODEL.contains("sdk_gphone")
            // Bind only rover sockets. A phone hosting a hotspot may have no client Wi-Fi Network.
            val builder = http.newBuilder()
            selectedNetwork?.let { network ->
                builder.socketFactory(network.socketFactory).dns(object : okhttp3.Dns {
                    override fun lookup(hostname: String) = network.getAllByName(hostname).toList()
                })
            }
            mutable.value = state.value.copy(network = when {
                useBle -> "BLE 备用链路 · 仍经 STM32 安全链路"
                wifi != null -> "Wi-Fi 本地网络"
                wired != null -> "有线本地网络"
                emulator -> "模拟器网络"
                else -> "系统局域网路由（手机热点）"
            })
            if (!useBle) {
                val observer = object : ConnectivityManager.NetworkCallback() {
                    override fun onLost(network: Network) {
                        main.postDelayed({
                            if (token == session && network == selectedNetwork) {
                                disconnect("Wi-Fi 持续断开，控制已锁定", false)
                            }
                        }, 1200)
                    }
                }
                callback = observer
                connectivity.registerNetworkCallback(NetworkRequest.Builder()
                    .removeCapability(NetworkCapabilities.NET_CAPABILITY_NOT_VPN)
                    .removeCapability(NetworkCapabilities.NET_CAPABILITY_TRUSTED)
                    .removeCapability(NetworkCapabilities.NET_CAPABILITY_NOT_RESTRICTED).build(), observer)
            }
            val onText: (String) -> Unit = { raw -> main.post { if (token == session) receive(raw) } }
            val onEnded: (String) -> Unit = { message -> main.post { if (token == session) disconnect(message, false) } }
            transport = if (useBle) {
                BleTransport(getApplication(), state.value.bleAddress, onText, onEnded)
                    .also { it.connect(getApplication()) }
            } else {
                SocketTransport(builder.build(), onText, onEnded).also { it.connect(endpoint!!) }
            }
            log(if (useBle) "正在连接 BLE ${state.value.bleAddress}" else "正在连接 ${endpoint!!.origin}")
            ticker = viewModelScope.launch(Dispatchers.Default) {
                var previous = now()
                val started = previous
                var statusMissingLogged = false
                while (token == session) {
                    delay(if (useBle) 80 else 66)
                    val time = now()
                    val elapsed = time - previous
                    val pulse = motionPulse.get()
                    if (pulse.armed && elapsed > 400) {
                        motionPulse.compareAndSet(pulse, MotionPulse())
                        driveLimiter.reset()
                        if (!sendDrive(Drive())) break
                        main.post {
                            if (token == session && engine.armed) {
                                engine.lock("控制发送循环超过 400 ms，已停车并锁定")
                                log(engine.note)
                                refresh()
                            }
                        }
                        previous = time
                        continue
                    }
                    previous = time
                    if (pulse.armed) {
                        val age = pulse.telemetryAt?.let { time - it }
                        if (age == null || age !in 0..900) {
                            if (motionPulse.compareAndSet(pulse, pulse.copy(armed = false, drive = Drive()))) {
                                if (!sendDrive(Drive())) break
                                main.post {
                                    if (token == session && engine.armed) {
                                        engine.lock("超过 900 ms 无新遥测，已锁定")
                                        log(engine.note)
                                        refresh()
                                    }
                                }
                            }
                        } else if (!sendDrive(driveLimiter.step(pulse.drive, elapsed))) break
                    } else {
                        driveLimiter.reset()
                    }
                    if (uiRefreshQueued.compareAndSet(false, true)) main.post {
                        try {
                            if (token != session) return@post
                            val currentTime = now()
                            val wasArmed = engine.armed
                            val oldNote = engine.note
                            engine.tick(currentTime)
                            if (wasArmed && !engine.armed) zero()
                            if (oldNote != engine.note) log(engine.note)
                            if (state.value.connection == Connection.CONNECTING && currentTime - started >= 6000) {
                                disconnect(if (useBle) "BLE 连接超时：检查配对、地址及香橙派桥接服务" else "连接超时：检查地址、端口及是否连接同一 Wi-Fi", false)
                                return@post
                            }
                            if (state.value.connection == Connection.CONNECTED && engine.lastTelemetryAt == null &&
                                currentTime - started > 2000 && !statusMissingLogged) {
                                log(engine.blockedReason(currentTime) ?: "等待小车遥测")
                                statusMissingLogged = true
                            }
                            refresh()
                        } finally { uiRefreshQueued.set(false) }
                    }
                }
            }
        } catch (e: Exception) { disconnect("无法连接：" + e.message, false) }
        refresh()
    }
    private fun receive(raw: String) {
        val status = try { Protocol.status(raw) }
        catch (e: Exception) { disconnect("状态协议无效：" + e.message, false); return }
        if (status.event == "rejected") {
            val message = when (status.ack?.reason) {
                "origin_not_allowed" -> SocketTransport.closeReason(4403, "")
                "controller_busy" -> SocketTransport.closeReason(4409, "")
                else -> "网关拒绝连接：" + status.ack?.reason
            }
            disconnect(message, false)
            return
        }
        if (status.gateway == null || status.event == "protocol_error") {
            disconnect("网关报告协议错误，已锁定", false)
            return
        }
        val wasArmed = engine.armed
        val oldNote = engine.note
        val wasClearing = engine.pendingClear
        if (status.event in listOf("connected", "serial_state")) poseEstimator.reset()
        engine.ingest(status, now())
        status.telemetry?.let { poseEstimator.ingest(it) }
        if (status.event == "connected") {
            if (!status.controlGranted) { disconnect("网关未授予控制权", false); return }
            mutable.value = state.value.copy(connection = Connection.CONNECTED)
            if (rememberedEstop) send { Protocol.estop(it) } else zero()
            log("已连接；" + (engine.blockedReason(now()) ?: "右滑启用后可控制"))
        }
        if (status.event == "vision_error") {
            log("摄像头跟随已停车：" + (status.gateway.vision?.error ?: "摄像头错误"))
        }
        status.ack?.let { ack ->
            if (ack.reason != "unmatched_transport_seq") {
                val stage = when (ack.stage) { "stm32" -> "STM32"; "dry_run" -> "仿真"; else -> "网关" }
                mutable.value = state.value.copy(lastAck = stage + " · " + (if (ack.accepted) "已接收" else "拒绝") + " · " + ack.reason)
            }
            if (!ack.accepted && ack.seq != null && wasClearing != ack.seq && engine.armed) {
                engine.lock("命令被拒绝：" + ack.reason)
            }
        }
        if (wasClearing != null && engine.pendingClear == null && !engine.localEstop) rememberedEstop = false
        if (wasArmed && !engine.armed) zero()
        if (oldNote != engine.note && status.event != "connected") log(engine.note)
        refresh()
    }
    @Synchronized private fun send(encode: (Long) -> String): Boolean {
        val wire = try { encode(engine.nextSequence()) } catch (e: Exception) {
            failSend(e.message ?: "命令编码失败"); return false
        }
        if (transport?.send(wire) != true) {
            failSend("发送受阻，已断开以丢弃积压指令")
            return false
        }
        return true
    }
    private fun failSend(message: String) {
        val token = session
        if (Looper.myLooper() == Looper.getMainLooper()) disconnect(message, false)
        else main.post { if (token == session) disconnect(message, false) }
    }
    private fun sendDrive(drive: Drive) = send { Protocol.drive(it, drive.linear, drive.angular, drive.lateral) }
    private fun zero() {
        driveLimiter.reset()
        if (state.value.connection == Connection.CONNECTED) sendDrive(Drive())
    }
    fun arm() {
        if (!focused) { stop("窗口未处于前台，保持锁定"); return }
        if (state.value.connection == Connection.CONNECTED && engine.arm(now())) sendDrive(Drive())
        log(engine.note); refresh()
    }
    fun stick(x: Float, y: Float) {
        if (replayingPath || pathRecorder.recording) return
        val before = engine.command
        engine.stick(x, y, state.value.speedLimit, now())
        if (engine.armed && engine.command == Drive() && before != Drive()) zero()
        publishMotion()
        // The 66 ms tick publishes UI state. Do not recompose the whole dashboard for
        // every high-frequency pointer event; release still sends zero immediately.
    }
    fun strafe(x: Float) {
        if (replayingPath || pathRecorder.recording) return
        val before = engine.command
        engine.strafe(x, state.value.speedLimit, now())
        if (engine.armed && engine.command == Drive() && before != Drive()) zero()
        publishMotion()
    }
    fun stop(reason: String = "已停车并锁定") {
        if (replayingPath) { cancelReplay(reason); return }
        if (pathRecorder.recording) pathRecorder.transition(Drive(), poseEstimator.pose, now())
        engine.lock(reason)
        zero()
        log(reason); refresh()
    }
    fun estop() {
        if (pathRecorder.recording) stopPathRecording()
        if (replayingPath) cancelReplay("路径复现已被急停中止")
        rememberedEstop = true
        engine.estop()
        if (state.value.connection == Connection.CONNECTED) {
            if (send { Protocol.estop(it) }) log("急停已发送，硬件执行以 STM32 确认为准")
        } else log("本地已锁定；当前断线，无法确认硬件急停")
        refresh()
    }
    fun clearEstop() {
        val seq = engine.nextSequence()
        if (!engine.requestClear(seq, now())) { log("尚不满足解除条件"); refresh(); return }
        if (transport?.send(Protocol.clear(seq)) != true) disconnect("解除发送失败，保持锁定", false)
        else log("等待解除确认；确认后仍需重新启用")
        refresh()
    }
    fun wheelTest(wheel: Int, direction: Int) {
        val gateway = state.value.gateway
        val ready = state.value.connection == Connection.CONNECTED && !engine.armed &&
            !pathRecorder.recording && !replayingPath && gateway?.serialReady == true &&
            !gateway.estop && !gateway.faultLatched && !gateway.obstacleBlocked
        if (!ready || wheel !in 0..3 || direction !in setOf(-1, 1)) {
            log("单轮校准不可用：请停车锁定并清除故障")
            refresh()
            return
        }
        if (send { Protocol.wheelTest(it, wheel, direction * 30) }) {
            val channel = listOf("C 左前", "A 右前", "B 左后", "D 右后")[wheel]
            log("$channel ${if (direction > 0) "正向" else "反向"}点动 180 ms")
        }
        refresh()
    }
    fun obstacleGuard(enabled: Boolean) {
        if (state.value.connection != Connection.CONNECTED) return
        val seq = engine.nextSequence()
        if (!engine.requestObstacleGuard(seq, enabled, now())) {
            log("只能在控制启用前设置自动停车"); refresh(); return
        }
        if (transport?.send(Protocol.obstacleGuard(seq, enabled)) != true) {
            disconnect("自动停车设置发送失败，保持锁定", false)
        } else log(engine.note)
        refresh()
    }
    fun visionFollow(enabled: Boolean, target: String) {
        if (state.value.connection != Connection.CONNECTED || target !in setOf("red", "green", "blue", "person")) return
        if (enabled && !state.value.canVisionFollow) {
            log("请先停车、锁定并确认 UART 与遥测正常"); refresh(); return
        }
        if (enabled) {
            if (pathRecorder.recording) stopPathRecording()
            if (replayingPath) cancelReplay("切换到摄像头跟随")
            engine.lock("摄像头跟随启动中")
            zero()
        }
        if (send { Protocol.visionFollow(it, enabled, target) }) {
            log(if (enabled) "摄像头跟随启动中：$target" else "摄像头跟随已停止")
        }
        refresh()
    }
    fun shutdownOrangePi() {
        if (state.value.connection != Connection.CONNECTED || !engine.canShutdownOrangePi(now())) {
            log("请先停车并等待车辆状态稳定"); refresh(); return
        }
        if (send { Protocol.shutdownOrangePi(it) }) {
            log("香橙派关机命令已发送；小车已急停，连接将断开")
        }
        refresh()
    }
    fun togglePathRecording() {
        if (pathRecorder.recording) {
            stopPathRecording()
            return
        }
        if (state.value.connection != Connection.CONNECTED || state.value.simulation || replayingPath) {
            log("路径录制需要已连接的实车链路"); refresh(); return
        }
        if (!poseEstimator.pose.ready) {
            log("等待完整编码器与 IMU 遥测后再开始录制"); refresh(); return
        }
        poseEstimator.reset()
        pathRecorder.start(now())
        log("路径录制已开始；使用方向按键，向右滑动启用后再移动")
        refresh()
    }
    fun pathDirection(direction: PathDirection, pressed: Boolean) {
        if (!pathRecorder.recording || replayingPath) return
        val before = engine.command
        engine.directional(direction, pressed, state.value.speedLimit, now())
        pathRecorder.transition(engine.command, poseEstimator.pose, now())
        if (engine.armed && engine.command == Drive() && before != Drive()) zero()
        publishMotion(); refresh()
    }
    fun replayPath(pathId: String) {
        val path = pathRecorder.path(pathId)
        if (path == null || path.segments.isEmpty() || pathRecorder.recording || replayingPath) return
        if (!focused || state.value.connection != Connection.CONNECTED || state.value.simulation) {
            log("路径复现需要前台中的实车连接"); refresh(); return
        }
        if (!engine.armed && !engine.arm(now())) {
            log(engine.note); refresh(); return
        }
        if (engine.armed && engine.blockedReason(now()) != null) {
            log(engine.blockedReason(now())!!); refresh(); return
        }
        // A replay invoked from an already enabled control session begins with an
        // explicit zero command so it cannot inherit a held manual direction.
        engine.replayCommand(Drive(), now())
        zero()
        poseEstimator.reset()
        replayingPath = true
        publishMotion(); refresh()
        replayJob = viewModelScope.launch {
            var completed = true
            var terminalNote = "路径复现完成，已停车并锁定"
            try {
                replay@ for (index in path.segments.indices) {
                    val segment = path.segments[index]
                    if (!engine.replayCommand(segment.drive, now())) {
                        completed = false
                        terminalNote = engine.note
                        break
                    }
                    log("路径复现 ${index + 1}/${path.segments.size}：${segment.label()}")
                    publishMotion(); refresh()
                    val segmentEndsAt = now() + segment.durationMs
                    while (true) {
                        val remaining = segmentEndsAt - now()
                        if (remaining <= 0L) break
                        delay(minOf(100L, remaining))
                        if (!replayingPath || !engine.armed) {
                            completed = false
                            terminalNote = engine.note
                            break@replay
                        }
                        if (!engine.replayCommand(segment.drive, now())) {
                            completed = false
                            terminalNote = engine.note
                            break@replay
                        }
                        publishMotion()
                    }
                }
            } finally {
                if (replayingPath) {
                    if (!completed && terminalNote.isBlank()) terminalNote = "路径复现中止，已停车并锁定"
                    engine.lock(terminalNote)
                    zero()
                    replayingPath = false
                    replayJob = null
                    log(engine.note); refresh()
                }
            }
        }
    }
    fun renamePath(pathId: String, name: String) {
        if (!pathRecorder.recording && !replayingPath && name.trim().isNotEmpty()) {
            pathRecorder.rename(pathId, name); refresh()
        }
    }
    fun setPathSegmentDuration(pathId: String, index: Int, durationMs: Long) {
        if (!pathRecorder.recording && !replayingPath) {
            pathRecorder.setDuration(pathId, index, durationMs); refresh()
        }
    }
    fun addParkingAfter(pathId: String, index: Int, durationMs: Long) {
        if (!pathRecorder.recording && !replayingPath) {
            pathRecorder.addParkingAfter(pathId, index, durationMs); refresh()
        }
    }
    fun deletePathSegment(pathId: String, index: Int) {
        if (!pathRecorder.recording && !replayingPath) {
            pathRecorder.deleteSegment(pathId, index); refresh()
        }
    }
    fun deletePath(pathId: String) {
        if (!pathRecorder.recording && !replayingPath) {
            pathRecorder.deletePath(pathId); refresh()
        }
    }
    private fun stopPathRecording() {
        val path = pathRecorder.stop(poseEstimator.pose, now())
        log(if (path == null) "未录到有效移动，已取消路径录制" else "路径已保存：${path.segments.size} 段")
        refresh()
    }
    private fun cancelReplay(reason: String) {
        replayJob?.cancel(); replayJob = null
        if (replayingPath) {
            engine.lock(reason); zero(); replayingPath = false
            log(reason); refresh()
        }
    }
    fun disconnect(reason: String = "已断开，控制已锁定", graceful: Boolean = true) {
        if (pathRecorder.recording) stopPathRecording()
        replayJob?.cancel(); replayJob = null; replayingPath = false
        val old = transport
        val zeroSent = if (graceful && state.value.connection == Connection.CONNECTED)
            runCatching { old?.send(Protocol.drive(engine.nextSequence(), 0.0, 0.0)) == true }.getOrDefault(false)
            else false
        ++session
        ticker?.cancel(); ticker = null
        transport = null
        if (graceful && zeroSent) {
            old?.close()
            main.postDelayed({ old?.cancel() }, 150)
        } else old?.cancel()
        callback?.let { runCatching { connectivity.unregisterNetworkCallback(it) } }
        callback = null; selectedNetwork = null
        engine.lock(reason)
        mutable.value = state.value.copy(connection = Connection.DISCONNECTED, network = "未连接",
            armed = false, canArm = false, canClear = false, clearPending = false,
            canConfigureObstacleGuard = false, obstacleGuardPending = false, canShutdownOrangePi = false, command = Drive(),
            canVisionFollow = false,
            recordingPath = false, replayingPath = false, canRecordPath = false, canReplayPath = false,
            gateway = null, telemetry = null, telemetryAgeMs = null)
        publishMotion()
        log(reason)
    }
    fun focusChanged(hasFocus: Boolean) {
        focused = hasFocus
        if (!hasFocus && engine.armed) stop("窗口失焦，已停车并锁定")
    }
    fun background() { if (state.value.connection != Connection.DISCONNECTED) disconnect("App 已离开前台，请重新连接") }
    override fun onCleared() {
        disconnect("控制器已关闭")
        http.dispatcher.executorService.shutdown()
        http.connectionPool.evictAll()
        super.onCleared()
    }
}
