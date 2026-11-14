package com.example.gnss_imu_logger.navigation

import kotlin.math.hypot

/** 方向付きスタートラインの局所ENU幾何計算。 */
class LapGeometry(
    start: EnuPosition,
    end: EnuPosition,
    extensionM: Double
) {
    private val start2 = Vec2(start.eastM, start.northM)
    private val end2 = Vec2(end.eastM, end.northM)
    private val line = end2 - start2
    private val lengthM = hypot(line.x, line.y).also {
        require(it >= 1.0) { "スタートラインは1 m以上の長さで設定してください。" }
    }
    private val tangent = line / lengthM
    private val normal = Vec2(-tangent.y, tangent.x)
    private val extensionRatio = extensionM / lengthM

    fun signedDistance(position: EnuPosition): Double =
        (Vec2(position.eastM, position.northM) - start2).dot(normal)

    fun lineDistance(position: EnuPosition): Double {
        val point = Vec2(position.eastM, position.northM)
        val ratio = ((point - start2).dot(line) / line.dot(line)).coerceIn(0.0, 1.0)
        return (point - (start2 + line * ratio)).norm()
    }

    fun normalSpeed(velocity: EnuVelocity): Double =
        Vec2(velocity.eastMps, velocity.northMps).dot(normal)

    fun intersection(previous: EnuPosition, current: EnuPosition): LapIntersection {
        val movement = Vec2(current.eastM - previous.eastM, current.northM - previous.northM)
        val origin = Vec2(previous.eastM, previous.northM)
        val denominator = movement.cross(line)
        if (kotlin.math.abs(denominator) < 1e-12) return LapIntersection.none()
        val delta = start2 - origin
        val trackRatio = delta.cross(line) / denominator
        val lineRatio = delta.cross(movement) / denominator
        val intersects = trackRatio in 0.0..1.0 &&
            lineRatio in -extensionRatio..(1.0 + extensionRatio)
        return LapIntersection(intersects, trackRatio, lineRatio)
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

data class LapIntersection(
    val intersects: Boolean,
    val trackRatio: Double?,
    val lineRatio: Double?
) {
    companion object { fun none() = LapIntersection(false, null, null) }
}
