package com.roverone.controller

import org.json.JSONObject
import java.net.URI

/** JSON contract shared with the Python gateway. Zero-lateral commands remain V1-compatible. */
object Protocol {
    fun drive(seq: Long, linear: Double, angular: Double, lateral: Double = 0.0): String {
        require(linear.isFinite() && angular.isFinite() && lateral.isFinite())
        require(linear in -1.0..1.0 && angular in -3.0..3.0 && lateral in -1.0..1.0)
        return message("drive", seq).put("linear", linear).put("angular", angular).apply {
            if (lateral != 0.0) put("lateral", lateral)
        }.toString()
    }
    fun estop(seq: Long) = message("estop", seq).toString()
    fun clear(seq: Long) = message("clear_estop", seq).put("confirm", true).toString()
    fun obstacleGuard(seq: Long, enabled: Boolean) = message("obstacle_guard", seq).put("enabled", enabled).toString()
    fun visionFollow(seq: Long, enabled: Boolean, target: String): String {
        require(target in setOf("red", "green", "blue", "person"))
        return message("vision_follow", seq).put("enabled", enabled).put("target", target).toString()
    }
    fun shutdownOrangePi(seq: Long) = message("shutdown_orange_pi", seq).put("confirm", true).toString()
    fun wheelTest(seq: Long, wheel: Int, rpm: Int): String {
        require(wheel in 0..3 && rpm != 0 && kotlin.math.abs(rpm) <= 40)
        return message("wheel_test", seq).put("wheel", wheel).put("rpm", rpm).toString()
    }
    private fun message(type: String, seq: Long): JSONObject {
        require(seq in 0..0xFFFFFFFFL)
        return JSONObject().put("type", type).put("seq", seq)
    }
    fun status(raw: String): Status {
        require(raw.length <= 32768) { "状态帧过大" }
        val root = JSONObject(raw)
        require(root.getString("type") == "robot_status") { "未知网关协议" }
        val event = root.getString("event")
        val gateway = root.optJSONObject("gateway")?.let { g ->
            val simulated = g.getBoolean("simulated")
            require(g.getString("mode") == if (simulated) "dry_run" else "uart")
            val vision = g.optJSONObject("vision")?.let { v ->
                val observation = v.optJSONObject("observation")
                val command = v.optJSONObject("command")
                Vision(
                    enabled = v.optBoolean("enabled", false),
                    ready = v.optBoolean("ready", false),
                    target = v.optString("target", "red"),
                    error = v.stringOrNull("error"),
                    found = observation?.optBoolean("found", false) == true,
                    horizontalError = observation?.number("horizontal_error"),
                    sizeRatio = observation?.number("size_ratio"),
                    linear = command?.number("linear") ?: 0.0,
                    angular = command?.number("angular") ?: 0.0,
                    reason = command?.optString("reason", "disabled") ?: "disabled"
                )
            }
            Gateway(simulated, g.getBoolean("uart_connected"), g.getBoolean("serial_ready"),
                g.getBoolean("estop_latched"), g.getBoolean("hardware_fault_latched"),
                g.getInt("hardware_fault_code"), g.getBoolean("clear_estop_pending"),
                g.getString("watchdog"), g.stringOrNull("serial_error"),
                g.optBoolean("obstacle_guard_enabled", true),
                g.optBoolean("obstacle_guard_supported", false),
                g.optBoolean("obstacle_blocked", false), vision)
        }
        val telemetry = if (event == "telemetry") root.optJSONObject("telemetry")?.let { t ->
            val simulated = t.optBoolean("simulated", false)
            // A simulation must never satisfy a hardware telemetry freshness check.
            require(gateway != null && gateway.simulated == simulated)
            val enc = t.optJSONObject("encoders")
            if (!simulated) {
                listOf("battery_v", "measured_linear", "measured_angular", "imu_yaw").forEach {
                    require(t.number(it) != null) { "遥测缺少 $it" }
                }
                require(enc != null && t.has("estop") && t.has("fault_code"))
            }
            Telemetry(
                battery = t.number(if (simulated) "batteryVoltage" else "battery_v"),
                distance = if (simulated) t.number("distanceCm")?.div(100) else t.number("ultrasonic_m"),
                linear = t.number("measured_linear"), angular = t.number("measured_angular"),
                yaw = t.number("imu_yaw"), encoders = listOf("front_left", "front_right", "rear_left", "rear_right")
                    .map { key -> if (enc == null || enc.isNull(key)) null else enc.getLong(key) },
                simulated = simulated, estop = t.getBoolean("estop"), faultCode = t.optInt("fault_code", 0)
            )
        } else null
        val ack = root.optJSONObject("ack")?.let { a ->
            Ack(if (a.isNull("seq")) null else a.getLong("seq"), a.optBoolean("accepted", false),
                a.optBoolean("applied", false), a.optString("stage", "gateway"), a.optString("reason", ""),
                if (a.isNull("acked_type")) null else a.getInt("acked_type"))
        }
        return Status(event, gateway, telemetry, ack, root.optJSONObject("control")?.optBoolean("granted") == true)
    }
    private fun JSONObject.number(key: String): Double? {
        if (isNull(key)) return null
        val number = get(key) as? Number ?: error("$key 不是数值")
        return number.toDouble().also { require(it.isFinite()) { "$key 不是有限数值" } }
    }
    private fun JSONObject.stringOrNull(key: String) = if (isNull(key)) null else getString(key)
}

data class Endpoint(val origin: String, val webSocket: String, val health: String, val host: String) {
    companion object {
        fun parse(input: String): Endpoint {
            var address = input.trim()
            if (!address.contains("://")) address = "http://$address"
            address = address.replaceFirst(Regex("^ws://"), "http://").replaceFirst(Regex("^wss://"), "https://")
            val uri = URI(address)
            require(uri.scheme in listOf("http", "https")) { "请填写 http:// 或 https:// 局域网地址" }
            require(uri.userInfo == null && uri.query == null && uri.fragment == null) { "地址不能含账号、参数或片段" }
            require(uri.path in listOf("", "/", "/ws", "/health")) { "请填写控制页根地址" }
            val host = uri.host?.lowercase() ?: error("地址缺少有效 IP")
            val bytes = host.split('.').mapNotNull { it.toIntOrNull()?.takeIf { n -> n in 0..255 } }
            val localIp = bytes.size == 4 && (bytes[0] == 10 || bytes[0] == 127 ||
                (bytes[0] == 192 && bytes[1] == 168) || (bytes[0] == 172 && bytes[1] in 16..31) ||
                (bytes[0] == 169 && bytes[1] == 254))
            require(localIp || host.endsWith(".local") || host == "localhost") { "仅支持局域网 IP 或 .local 主机名" }
            require(uri.port == -1 || uri.port in 1..65535) { "端口应为 1–65535" }
            val port = if (uri.port == -1 || (uri.scheme == "http" && uri.port == 80) ||
                (uri.scheme == "https" && uri.port == 443)) "" else ":${uri.port}"
            val origin = "${uri.scheme}://$host$port"
            val ws = if (uri.scheme == "https") "wss" else "ws"
            return Endpoint(origin, "$ws://$host$port/ws", "$origin/health", host)
        }
    }
}
