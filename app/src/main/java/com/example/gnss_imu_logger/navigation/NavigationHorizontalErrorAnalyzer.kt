package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 水平速度誤差を進行方向・横方向・pitch重力漏れ・GNSS速度不整合へ分解する。
 * 入力: GNSS更新前状態、GNSS速度・位置、Heading状態、気圧鉛直速度
 * 出力: 瞬時成分、5秒窓中央値、原因分類
 */
class NavigationHorizontalErrorAnalyzer(
    private val verticalObservations: List<NavigationVerticalObservation>,
    private val windowDurationNs: Long = 5_000_000_000L,
    private val minimumWindowSamples: Int = 4
) {
    private var previousGnss: NavigationGnssUpdate? = null
    private val window = ArrayDeque<HorizontalSample>()
    private var verticalIndex = 0

    fun analyze(
        stateBeforeGnss: NavigationState,
        gnss: NavigationGnssUpdate,
        directionState: NavigationHeadingDirectionState,
        trackingState: NavigationHeadingTrackingState
    ): NavigationHorizontalErrorRecord {
        val gnssVelocity = gnss.velocityEnuMps
        val bearing = gnss.bearingRad?.let {
            if (directionState == NavigationHeadingDirectionState.REVERSE) wrap(it + Math.PI) else it
        }
        val rejection = when {
            gnssVelocity == null -> "GNSS_VELOCITY_UNAVAILABLE"
            bearing == null -> "BEARING_UNAVAILABLE"
            directionState != NavigationHeadingDirectionState.FORWARD &&
                directionState != NavigationHeadingDirectionState.REVERSE -> "DIRECTION_UNRESOLVED"
            else -> ""
        }
        val forwardEast = bearing?.let(::sin)
        val forwardNorth = bearing?.let(::cos)
        val leftEast = bearing?.let { -cos(it) }
        val leftNorth = bearing?.let(::sin)
        val innovationEast = gnssVelocity?.let { stateBeforeGnss.velocityEnuMps.eastMps - it.eastMps }
        val innovationNorth = gnssVelocity?.let { stateBeforeGnss.velocityEnuMps.northMps - it.northMps }
        val alongInnovation = dot(innovationEast, innovationNorth, forwardEast, forwardNorth)
        val crossInnovation = dot(innovationEast, innovationNorth, leftEast, leftNorth)
        val alongAcceleration = dot(
            stateBeforeGnss.accelerationEnuMps2.x,
            stateBeforeGnss.accelerationEnuMps2.y,
            forwardEast,
            forwardNorth
        )
        val crossAcceleration = dot(
            stateBeforeGnss.accelerationEnuMps2.x,
            stateBeforeGnss.accelerationEnuMps2.y,
            leftEast,
            leftNorth
        )
        val horizontalSpeed = gnssVelocity?.let {
            sqrt(it.eastMps * it.eastMps + it.northMps * it.northMps)
        }
        val verticalSpeed = nearestVerticalSpeed(gnss.elapsedRealtimeNs)
        val roadGrade = if (horizontalSpeed != null && horizontalSpeed > 0.5 && verticalSpeed != null) {
            atan2(verticalSpeed, horizontalSpeed)
        } else null
        val pitch = stateBeforeGnss.attitudeBodyToEnu.toEulerAngles().pitchRad
        val forwardAxisEnu = stateBeforeGnss.attitudeBodyToEnu.rotate(
            BodyVector(1.0, 0.0, 0.0)
        )
        val vehicleGrade = atan2(
            forwardAxisEnu.z,
            kotlin.math.hypot(forwardAxisEnu.x, forwardAxisEnu.y)
        )
        val pitchGradeResidual = roadGrade?.let { wrap(vehicleGrade - it) }
        val gravityLeak = pitchGradeResidual?.let { 9.80665 * sin(it) }
        val positionVelocity = positionDifferenceVelocity(gnss)
        val consistencyError = if (positionVelocity != null && gnssVelocity != null) {
            horizontalNorm(
                positionVelocity.eastMps - gnssVelocity.eastMps,
                positionVelocity.northMps - gnssVelocity.northMps
            )
        } else null
        val positionBearing = positionVelocity?.let {
            if (horizontalNorm(it.eastMps, it.northMps) > 3.0) atan2(it.eastMps, it.northMps) else null
        }
        val bearingConsistency = if (bearing != null && positionBearing != null) {
            abs(wrap(bearing - positionBearing))
        } else null

        if (rejection.isEmpty() && alongInnovation != null && crossInnovation != null) {
            window.addLast(
                HorizontalSample(
                    gnss.elapsedRealtimeNs,
                    alongInnovation,
                    crossInnovation,
                    alongAcceleration ?: 0.0,
                    crossAcceleration ?: 0.0,
                    gravityLeak,
                    consistencyError,
                    bearingConsistency
                )
            )
            while (window.isNotEmpty() &&
                gnss.elapsedRealtimeNs - window.first().elapsedRealtimeNs > windowDurationNs
            ) window.removeFirst()
        }
        val summary = summarizeWindow()
        previousGnss = gnss
        return NavigationHorizontalErrorRecord(
            elapsedRealtimeNs = gnss.elapsedRealtimeNs,
            predictedVelocityEastMps = stateBeforeGnss.velocityEnuMps.eastMps,
            predictedVelocityNorthMps = stateBeforeGnss.velocityEnuMps.northMps,
            gnssVelocityEastMps = gnssVelocity?.eastMps,
            gnssVelocityNorthMps = gnssVelocity?.northMps,
            innovationEastMps = innovationEast,
            innovationNorthMps = innovationNorth,
            alongTrackInnovationMps = alongInnovation,
            crossTrackInnovationMps = crossInnovation,
            accelerationEastMps2 = stateBeforeGnss.accelerationEnuMps2.x,
            accelerationNorthMps2 = stateBeforeGnss.accelerationEnuMps2.y,
            alongTrackAccelerationMps2 = alongAcceleration,
            crossTrackAccelerationMps2 = crossAcceleration,
            estimatedPitchRad = pitch,
            vehicleGradeRad = vehicleGrade,
            roadGradeRad = roadGrade,
            pitchGradeResidualRad = pitchGradeResidual,
            gravityLeakMps2 = gravityLeak,
            positionDifferenceVelocityEastMps = positionVelocity?.eastMps,
            positionDifferenceVelocityNorthMps = positionVelocity?.northMps,
            gnssVelocityConsistencyErrorMps = consistencyError,
            gnssBearingConsistencyErrorRad = bearingConsistency,
            windowSampleCount = summary.sampleCount,
            alongTrackInnovationMedianMps = summary.alongMedian,
            crossTrackInnovationMedianMps = summary.crossMedian,
            gravityLeakMedianMps2 = summary.gravityMedian,
            cause = classify(summary),
            valid = rejection.isEmpty(),
            rejectionReason = rejection,
            headingTrackingState = trackingState
        )
    }

    private fun nearestVerticalSpeed(elapsedRealtimeNs: Long): Double? {
        if (verticalObservations.isEmpty()) return null
        while (verticalIndex + 1 < verticalObservations.size &&
            abs(verticalObservations[verticalIndex + 1].elapsedRealtimeNs - elapsedRealtimeNs) <=
            abs(verticalObservations[verticalIndex].elapsedRealtimeNs - elapsedRealtimeNs)
        ) verticalIndex++
        val nearest = verticalObservations[verticalIndex]
        return nearest.verticalSpeedMps.takeIf {
            abs(nearest.elapsedRealtimeNs - elapsedRealtimeNs) <= 500_000_000L
        }
    }

    private fun positionDifferenceVelocity(current: NavigationGnssUpdate): EnuVelocity? {
        val previous = previousGnss ?: return null
        val dtSec = (current.elapsedRealtimeNs - previous.elapsedRealtimeNs) / 1_000_000_000.0
        if (dtSec !in 0.5..2.0) return null
        val east = (current.positionEnuM.eastM - previous.positionEnuM.eastM) / dtSec
        val north = (current.positionEnuM.northM - previous.positionEnuM.northM) / dtSec
        if (horizontalNorm(east * dtSec, north * dtSec) < 3.0) return null
        return EnuVelocity(east, north, 0.0)
    }

    private fun summarizeWindow(): WindowSummary {
        if (window.size < minimumWindowSamples) return WindowSummary(window.size, null, null, null, null, null)
        return WindowSummary(
            sampleCount = window.size,
            alongMedian = window.map { it.alongInnovation }.median(),
            crossMedian = window.map { it.crossInnovation }.median(),
            gravityMedian = window.mapNotNull { it.gravityLeak }.medianOrNull(),
            consistencyMedian = window.mapNotNull { it.consistencyError }.medianOrNull(),
            bearingConsistencyMedian = window.mapNotNull { it.bearingConsistency }.medianOrNull()
        )
    }

    private fun classify(summary: WindowSummary): NavigationHorizontalErrorCause {
        val along = summary.alongMedian ?: return NavigationHorizontalErrorCause.INCONCLUSIVE
        val cross = summary.crossMedian ?: return NavigationHorizontalErrorCause.INCONCLUSIVE
        val gravity = summary.gravityMedian
        if ((summary.consistencyMedian ?: 0.0) >= 5.0 &&
            (summary.bearingConsistencyMedian ?: 0.0) >= Math.toRadians(20.0)
        ) return NavigationHorizontalErrorCause.GNSS_VELOCITY_INCONSISTENCY
        if (gravity != null && abs(gravity) >= 0.15 && abs(along) >= 2.0 && along * gravity > 0.0) {
            return NavigationHorizontalErrorCause.PITCH_GRAVITY_LEAK
        }
        if (abs(cross) >= 2.0 && abs(cross) >= abs(along)) {
            return NavigationHorizontalErrorCause.CROSS_TRACK_HEADING
        }
        if (abs(along) >= 2.0 && abs(along) >= 2.0 * abs(cross) && abs(gravity ?: 0.0) < 0.1) {
            return NavigationHorizontalErrorCause.ALONG_TRACK_ACCELERATION
        }
        return if (abs(along) >= 2.0 || abs(cross) >= 2.0) NavigationHorizontalErrorCause.MIXED
        else NavigationHorizontalErrorCause.INCONCLUSIVE
    }

    private fun dot(a: Double?, b: Double?, x: Double?, y: Double?): Double? =
        if (a != null && b != null && x != null && y != null) a * x + b * y else null
    private fun horizontalNorm(east: Double, north: Double) = sqrt(east * east + north * north)
    private fun wrap(value: Double) = atan2(sin(value), cos(value))
    private fun List<Double>.median(): Double {
        val values = sorted(); val middle = values.size / 2
        return if (values.size % 2 == 0) (values[middle - 1] + values[middle]) * 0.5 else values[middle]
    }
    private fun List<Double>.medianOrNull(): Double? = if (isEmpty()) null else median()

    private data class HorizontalSample(
        val elapsedRealtimeNs: Long,
        val alongInnovation: Double,
        val crossInnovation: Double,
        val alongAcceleration: Double,
        val crossAcceleration: Double,
        val gravityLeak: Double?,
        val consistencyError: Double?,
        val bearingConsistency: Double?
    )
    private data class WindowSummary(
        val sampleCount: Int,
        val alongMedian: Double?,
        val crossMedian: Double?,
        val gravityMedian: Double?,
        val consistencyMedian: Double?,
        val bearingConsistencyMedian: Double?
    )
}

enum class NavigationHorizontalErrorCause {
    ALONG_TRACK_ACCELERATION,
    CROSS_TRACK_HEADING,
    PITCH_GRAVITY_LEAK,
    GNSS_VELOCITY_INCONSISTENCY,
    MIXED,
    INCONCLUSIVE
}

data class NavigationHorizontalErrorRecord(
    val elapsedRealtimeNs: Long,
    val predictedVelocityEastMps: Double,
    val predictedVelocityNorthMps: Double,
    val gnssVelocityEastMps: Double?,
    val gnssVelocityNorthMps: Double?,
    val innovationEastMps: Double?,
    val innovationNorthMps: Double?,
    val alongTrackInnovationMps: Double?,
    val crossTrackInnovationMps: Double?,
    val accelerationEastMps2: Double,
    val accelerationNorthMps2: Double,
    val alongTrackAccelerationMps2: Double?,
    val crossTrackAccelerationMps2: Double?,
    val estimatedPitchRad: Double,
    val vehicleGradeRad: Double,
    val roadGradeRad: Double?,
    val pitchGradeResidualRad: Double?,
    val gravityLeakMps2: Double?,
    val positionDifferenceVelocityEastMps: Double?,
    val positionDifferenceVelocityNorthMps: Double?,
    val gnssVelocityConsistencyErrorMps: Double?,
    val gnssBearingConsistencyErrorRad: Double?,
    val windowSampleCount: Int,
    val alongTrackInnovationMedianMps: Double?,
    val crossTrackInnovationMedianMps: Double?,
    val gravityLeakMedianMps2: Double?,
    val cause: NavigationHorizontalErrorCause,
    val valid: Boolean,
    val rejectionReason: String,
    val headingTrackingState: NavigationHeadingTrackingState
)
