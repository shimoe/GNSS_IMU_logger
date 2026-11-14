package com.example.gnss_imu_logger.navigation

import kotlin.math.hypot

/** 方向付きスタートラインの局所ENU幾何計算。 */
class LapGeometry(start: EnuPosition, end: EnuPosition) {
    private val start2 = Vec2(start.eastM, start.northM)
    private val end2 = Vec2(end.eastM, end.northM)
    private val line = end2 - start2
    private val lengthM = hypot(line.x, line.y).also {
        require(it >= 1.0) { "スタートラインは1 m以上の長さで設定してください。" }
    }
    private val tangent = line / lengthM
    private val normal = Vec2(-tangent.y, tangent.x)

    fun signedDistance(position: EnuPosition): Double =
        (Vec2(position.eastM, position.northM) - start2).dot(normal)

    fun lineDistance(position: EnuPosition): Double {
        val point = Vec2(position.eastM, position.northM)
        val ratio = ((point - start2).dot(line) / line.dot(line)).coerceIn(0.0, 1.0)
        return (point - (start2 + line * ratio)).norm()
    }

    fun normalSpeed(velocity: EnuVelocity): Double =
        Vec2(velocity.eastMps, velocity.northMps).dot(normal)

    /** 無限ラインとの交差候補を返し、有限線分の受理範囲はLapDetectorで判定する。 */
    fun crossingCandidate(previous: EnuPosition, current: EnuPosition): LapCrossingCandidate {
        val movement = Vec2(current.eastM - previous.eastM, current.northM - previous.northM)
        val origin = Vec2(previous.eastM, previous.northM)
        val denominator = movement.cross(line)
        if (kotlin.math.abs(denominator) < 1e-12) return LapCrossingCandidate.none()
        val delta = start2 - origin
        val trackRatio = delta.cross(line) / denominator
        val lineRatio = delta.cross(movement) / denominator
        if (trackRatio !in 0.0..1.0) return LapCrossingCandidate.none()
        val endpointDistanceM = when {
            lineRatio < 0.0 -> -lineRatio * lengthM
            lineRatio > 1.0 -> (lineRatio - 1.0) * lengthM
            else -> 0.0
        }
        return LapCrossingCandidate(true, trackRatio, lineRatio, endpointDistanceM)
    }

    private data class Vec2(val x: Double, val y: Double) {
        operator fun plus(other: Vec2) = Vec2(x + other.x, y + other.y)
        operator fun minus(other: Vec2) = Vec2(x - other.x, y - other.y)
        operator fun times(scale: Double) = Vec2(x * scale, y * scale)
        operator fun div(scale: Double) = Vec2(x / scale, y / scale)
        fun dot(other: Vec2) = x * other.x + y * other.y
        fun cross(other: Vec2) = x * other.y - y * other.x
        fun norm() = hypot(x, y)
    }
}

data class LapCrossingCandidate(
    val crossedInfiniteLine: Boolean,
    val trackRatio: Double?,
    val lineRatio: Double?,
    val endpointDistanceM: Double?
) {
    companion object { fun none() = LapCrossingCandidate(false, null, null, null) }
}
