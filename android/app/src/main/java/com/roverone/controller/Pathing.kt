package com.roverone.controller

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.UUID

/** Relative pose only: X points forward from the record origin and Y points left. */
data class RoverPose(
    val xMetres: Double = 0.0,
    val yMetres: Double = 0.0,
    val headingRad: Double = 0.0,
    val confidence: Double = 0.0,
    val ready: Boolean = false
)

enum class PathDirection { FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STRAFE_LEFT, STRAFE_RIGHT }

data class PathSegment(
    val drive: Drive,
    val durationMs: Long,
    val endPose: RoverPose = RoverPose()
) {
    fun label(): String = when {
        drive.linear > 0.0 -> "前进"
        drive.linear < 0.0 -> "后退"
        drive.lateral > 0.0 -> "右平移"
        drive.lateral < 0.0 -> "左平移"
        drive.angular > 0.0 -> "左转"
        drive.angular < 0.0 -> "右转"
        else -> "驻车"
    }
}

data class RecordedPath(val id: String, val name: String, val segments: List<PathSegment>)

/**
 * Dead reckoning with a conservative fuzzy fusion rule. Encoder deltas supply
 * translation while the IMU heading dominates whenever it disagrees with the
 * wheel-derived yaw, which reduces the effect of mecanum wheel slip.
 */
class FuzzyPoseEstimator {
    private val wheelRadiusMetres = 0.0325
    private val countsPerRevolution = 13.0 * 4.0 * 30.0
    private val wheelTrackMetres = 0.180
    private var previousEncoders: List<Long>? = null
    private var previousRawHeading: Double? = null
    var pose = RoverPose(); private set

    fun reset() {
        previousEncoders = null
        previousRawHeading = null
        pose = RoverPose()
    }

    fun ingest(telemetry: Telemetry): RoverPose {
        val encoders = telemetry.encoders
        val rawHeading = telemetry.yaw
        if (encoders.size != 4 || encoders.any { it == null } || rawHeading == null || !rawHeading.isFinite()) {
            return pose
        }
        val current = encoders.map { it!! }
        val previous = previousEncoders
        if (previous == null) {
            previousEncoders = current
            previousRawHeading = rawHeading
            // The first sample defines the record origin.  Keep heading relative to
            // that origin while retaining rawHeading internally for IMU deltas.
            pose = RoverPose(headingRad = 0.0, confidence = 0.45, ready = true)
            return pose
        }
        val metresPerCount = 2.0 * PI * wheelRadiusMetres / countsPerRevolution
        val delta = current.indices.map { (current[it] - previous[it]) * metresPerCount }
        val forward = (delta[0] + delta[1] + delta[2] + delta[3]) / 4.0
        val lateralRight = (delta[0] - delta[1] - delta[2] + delta[3]) / 4.0
        val encoderYaw = (-delta[0] + delta[1] - delta[2] + delta[3]) /
            (4.0 * (wheelTrackMetres * 0.5))
        val imuYaw = angleDifference(rawHeading, previousRawHeading ?: rawHeading)
        val disagreement = abs(angleDifference(imuYaw, encoderYaw))
        val imuWeight = if (disagreement < 0.18) 0.76 else 0.90
        val fusedYaw = imuYaw * imuWeight + encoderYaw * (1.0 - imuWeight)
        val heading = wrap(pose.headingRad + fusedYaw)
        val midHeading = wrap(pose.headingRad + fusedYaw * 0.5)
        val nextX = pose.xMetres + forward * cos(midHeading) + lateralRight * sin(midHeading)
        val nextY = pose.yMetres + forward * sin(midHeading) - lateralRight * cos(midHeading)
        val wheelSpread = delta.maxOrNull()!! - delta.minOrNull()!!
        val quality = (1.0 - (disagreement / 0.8).coerceIn(0.0, 0.7) -
            (abs(wheelSpread) / 0.25).coerceIn(0.0, 0.25)).coerceIn(0.2, 1.0)
        pose = RoverPose(nextX, nextY, heading, pose.confidence * 0.92 + quality * 0.08, true)
        previousEncoders = current
        previousRawHeading = rawHeading
        return pose
    }

    private fun wrap(value: Double): Double {
        var wrapped = value
        while (wrapped > PI) wrapped -= 2.0 * PI
        while (wrapped <= -PI) wrapped += 2.0 * PI
        return wrapped
    }
    private fun angleDifference(next: Double, previous: Double) = wrap(next - previous)
}

class PathStore(context: Context) {
    private val preferences = context.getSharedPreferences("rover_paths", Context.MODE_PRIVATE)
    fun loadAll(): List<RecordedPath> = runCatching {
        val stored = preferences.getString("paths", null)
        if (stored != null) {
            JSONObject(stored).getJSONArray("paths").map { decode(it as JSONObject) }
        } else {
            // Keep the last single-path version usable after upgrading to the library.
            val legacy = preferences.getString("last_path", null)?.let { decode(JSONObject(it)) }
            legacy?.let { listOf(it.copy(id = UUID.randomUUID().toString())) }.orEmpty()
                .also { if (it.isNotEmpty()) saveAll(it) }
        }
    }.getOrDefault(emptyList())

    fun saveAll(paths: List<RecordedPath>) {
        val array = JSONArray()
        paths.forEach { path -> array.put(encode(path)) }
        preferences.edit().putString("paths", JSONObject().put("paths", array).toString())
            .remove("last_path").apply()
    }

    private fun decode(root: JSONObject): RecordedPath {
        val segments = root.getJSONArray("segments").map { value ->
            val item = value as JSONObject
            PathSegment(
                Drive(item.getDouble("linear"), item.getDouble("angular"), item.getDouble("lateral")),
                item.getLong("duration_ms"),
                RoverPose(item.getDouble("x"), item.getDouble("y"), item.getDouble("heading"),
                    item.optDouble("confidence", 0.0), true)
            )
        }
        return RecordedPath(root.optString("id", UUID.randomUUID().toString()),
            root.optString("name", "已录制路径"), segments)
    }

    private fun encode(path: RecordedPath): JSONObject {
        val array = JSONArray()
        path.segments.forEach { segment ->
            array.put(JSONObject()
                .put("linear", segment.drive.linear).put("angular", segment.drive.angular)
                .put("lateral", segment.drive.lateral).put("duration_ms", segment.durationMs)
                .put("x", segment.endPose.xMetres).put("y", segment.endPose.yMetres)
                .put("heading", segment.endPose.headingRad).put("confidence", segment.endPose.confidence))
        }
        return JSONObject().put("id", path.id).put("name", path.name).put("segments", array)
    }
}

class PathRecorder(private val store: PathStore) {
    private data class Pending(val drive: Drive, val startedAtMs: Long)
    var paths: List<RecordedPath> = store.loadAll(); private set
    var recording = false; private set
    private var pending: Pending? = null
    private val working = mutableListOf<PathSegment>()

    fun start(@Suppress("UNUSED_PARAMETER") nowMs: Long) {
        working.clear(); pending = null; recording = true
    }
    fun transition(drive: Drive, pose: RoverPose, nowMs: Long) {
        if (!recording || pending?.drive == drive) return
        finishPending(pose, nowMs)
        if (drive != Drive()) pending = Pending(drive, nowMs)
    }
    fun stop(pose: RoverPose, nowMs: Long): RecordedPath? {
        if (!recording) return null
        finishPending(pose, nowMs)
        recording = false
        val saved = working.takeIf { it.isNotEmpty() }?.let { segments ->
            RecordedPath(UUID.randomUUID().toString(), automaticName(), segments.toList())
        }
        if (saved != null) {
            paths = paths + saved
            store.saveAll(paths)
        }
        return saved
    }
    fun path(id: String): RecordedPath? = paths.firstOrNull { it.id == id }
    fun rename(id: String, name: String) = replace(id) { it.copy(name = name.trim().take(40)) }
    fun setDuration(id: String, index: Int, durationMs: Long) {
        val path = path(id) ?: return
        if (index !in path.segments.indices) return
        val next = path.segments.toMutableList()
        val segment = next[index]
        next[index] = segment.copy(durationMs = durationMs.coerceIn(120L, 300_000L))
        replace(id) { path.copy(segments = next) }
    }
    fun addParkingAfter(id: String, index: Int, durationMs: Long) {
        val path = path(id) ?: return
        if (index !in 0 until path.segments.lastIndex) return
        val next = path.segments.toMutableList()
        next.add(index + 1, PathSegment(Drive(), durationMs.coerceIn(120L, 300_000L), path.segments[index].endPose))
        replace(id) { path.copy(segments = next) }
    }
    fun deleteSegment(id: String, index: Int) {
        val path = path(id) ?: return
        if (index !in path.segments.indices) return
        val next = path.segments.toMutableList().also { it.removeAt(index) }
        if (next.isEmpty()) deletePath(id) else replace(id) { path.copy(segments = next) }
    }
    fun deletePath(id: String) {
        paths = paths.filterNot { it.id == id }
        store.saveAll(paths)
    }
    private fun replace(id: String, transform: (RecordedPath) -> RecordedPath) {
        paths = paths.map { if (it.id == id) transform(it) else it }
        store.saveAll(paths)
    }
    private fun automaticName(): String = "路径 " + SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.CHINA).format(Date())
    private fun finishPending(pose: RoverPose, nowMs: Long) {
        val active = pending ?: return
        val duration = (nowMs - active.startedAtMs).coerceIn(120L, 300_000L)
        working += PathSegment(active.drive, duration, pose)
        pending = null
    }
}

private inline fun <T> JSONArray.map(transform: (Any) -> T): List<T> =
    List(length()) { transform(get(it)) }
