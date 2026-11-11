package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sqrt

/**
 * QuaternionのEuler pitch、前方軸傾斜、気圧道路勾配を同一GNSS時刻で比較する。
 * 入力: GNSS更新前状態、GNSS速度、気圧鉛直速度
 * 出力: pitch軸・符号・道路勾配との整合性診断
 */
class NavigationPitchReferenceAnalyzer(
    private val verticalObservations: List<NavigationVerticalObservation>,
    private val maximumVerticalAgeNs: Long = 200_000_000L,
    private val windowDurationNs: Long = 5_000_000_000L
) {
    private val window = ArrayDeque<PitchSample>()
    private var verticalIndex = 0

    fun analyze(
        state: NavigationState,
        gnss: NavigationGnssUpdate,
        trackingState: NavigationHeadingTrackingState
    ): NavigationPitchReferenceRecord {
        val eulerPitch = state.attitudeBodyToEnu.toEulerAngles().pitchRad
        val forwardEnu = state.attitudeBodyToEnu.rotate(BodyVector(1.0, 0.0, 0.0))
        val forwardGrade = atan2(
            forwardEnu.z,
            hypot(forwardEnu.x, forwardEnu.y)
        )
        // Euler pitchは車体前方軸の上向き傾斜と逆符号のため、
        // 道路勾配と同じ符号へ変換してから前方軸傾斜と比較する。
        val vehicleGradeFromEuler = -eulerPitch
        val eulerForwardResidual = wrap(vehicleGradeFromEuler - forwardGrade)
        val gnssSpeed = gnss.velocityEnuMps?.let {
            hypot(it.eastMps, it.northMps)
        }
        val vertical = nearestVertical(gnss.elapsedRealtimeNs)
        val roadGrade = if (
            trackingState == NavigationHeadingTrackingState.TRACKING &&
            gnssSpeed != null && gnssSpeed >= 5.0 &&
            vertical != null && abs(vertical.verticalSpeedMps) <= 5.0
        ) atan2(vertical.verticalSpeedMps, gnssSpeed) else null
        val pitchRoadResidual = roadGrade?.let { wrap(forwardGrade - it) }
        val rejection = when {
            trackingState != NavigationHeadingTrackingState.TRACKING -> "HEADING_NOT_TRACKING"
            gnssSpeed == null -> "GNSS_VELOCITY_UNAVAILABLE"
            gnssSpeed < 5.0 -> "SPEED"
            vertical == null -> "BAROMETER_UNAVAILABLE"
            abs(vertical.verticalSpeedMps) > 5.0 -> "BAROMETER_VERTICAL_SPEED"
            else -> ""
        }
        if (roadGrade != null) {
            window.addLast(PitchSample(gnss.elapsedRealtimeNs, eulerPitch, forwardGrade, roadGrade))
            while (window.isNotEmpty() &&
                gnss.elapsedRealtimeNs - window.first().elapsedRealtimeNs > windowDurationNs
            ) window.removeFirst()
        }
        val summary = summarize()
        return NavigationPitchReferenceRecord(
            elapsedRealtimeNs = gnss.elapsedRealtimeNs,
            eulerPitchRad = eulerPitch,
            vehicleGradeFromEulerRad = vehicleGradeFromEuler,
            vehicleGradeFromEulerResidualRad = eulerForwardResidual,
            forwardAxisEast = forwardEnu.x,
            forwardAxisNorth = forwardEnu.y,
            forwardAxisUp = forwardEnu.z,
            forwardAxisGradeRad = forwardGrade,
            eulerForwardAxisResidualRad = eulerForwardResidual,
            barometricVerticalSpeedMps = vertical?.verticalSpeedMps,
            gnssHorizontalSpeedMps = gnssSpeed,
            barometricRoadGradeRad = roadGrade,
            roadGradeAgeMs = vertical?.ageNs?.div(1_000_000.0),
            gravityPitchRad = null,
            gravityObservationValid = false,
            rotationVectorPitchRad = null,
            rotationVectorResidualRad = null,
            pitchRoadGradeResidualRad = pitchRoadResidual,
            pitchGravityResidualRad = null,
            windowSampleCount = summary.sampleCount,
            eulerPitchMedianRad = summary.eulerMedian,
            forwardAxisGradeMedianRad = summary.forwardMedian,
            roadGradeMedianRad = summary.roadMedian,
            gravityPitchMedianRad = null,
            axisConsistent = abs(eulerForwardResidual) < Math.toRadians(1.0),
            pitchDriftCandidate = rejection.isEmpty() &&
                summary.sampleCount >= 4 &&
                summary.forwardMedian != null &&
                summary.roadMedian != null &&
                abs(wrap(summary.forwardMedian - summary.roadMedian)) >= Math.toRadians(3.0),
            valid = rejection.isEmpty(),
            rejectionReason = rejection
        )
    }

    private fun nearestVertical(elapsedRealtimeNs: Long): VerticalMatch? {
        if (verticalObservations.isEmpty()) return null
        while (verticalIndex + 1 < verticalObservations.size &&
            abs(verticalObservations[verticalIndex + 1].elapsedRealtimeNs - elapsedRealtimeNs) <=
            abs(verticalObservations[verticalIndex].elapsedRealtimeNs - elapsedRealtimeNs)
        ) verticalIndex++
        val value = verticalObservations[verticalIndex]
        val age = abs(value.elapsedRealtimeNs - elapsedRealtimeNs)
        return if (age <= maximumVerticalAgeNs) VerticalMatch(value.verticalSpeedMps, age) else null
    }

    private fun summarize(): WindowSummary {
        if (window.isEmpty()) return WindowSummary(0, null, null, null)
        return WindowSummary(
            sampleCount = window.size,
            eulerMedian = window.map { it.eulerPitch }.median(),
            forwardMedian = window.map { it.forwardGrade }.median(),
            roadMedian = window.map { it.roadGrade }.median()
        )
    }

    private fun List<Double>.median(): Double {
        val sorted = sorted(); val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) * 0.5
        else sorted[middle]
    }
    private fun wrap(value: Double) = atan2(kotlin.math.sin(value), kotlin.math.cos(value))

    private data class VerticalMatch(val verticalSpeedMps: Double, val ageNs: Long)
    private data class PitchSample(
        val elapsedRealtimeNs: Long,
        val eulerPitch: Double,
        val forwardGrade: Double,
        val roadGrade: Double
    )
    private data class WindowSummary(
        val sampleCount: Int,
        val eulerMedian: Double?,
        val forwardMedian: Double?,
        val roadMedian: Double?
    )
}

data class NavigationPitchReferenceRecord(
    val elapsedRealtimeNs: Long,
    val eulerPitchRad: Double,
    val vehicleGradeFromEulerRad: Double,
    val vehicleGradeFromEulerResidualRad: Double,
    val forwardAxisEast: Double,
    val forwardAxisNorth: Double,
    val forwardAxisUp: Double,
    val forwardAxisGradeRad: Double,
    val eulerForwardAxisResidualRad: Double,
    val barometricVerticalSpeedMps: Double?,
    val gnssHorizontalSpeedMps: Double?,
    val barometricRoadGradeRad: Double?,
    val roadGradeAgeMs: Double?,
    val gravityPitchRad: Double?,
    val gravityObservationValid: Boolean,
    val rotationVectorPitchRad: Double?,
    val rotationVectorResidualRad: Double?,
    val pitchRoadGradeResidualRad: Double?,
    val pitchGravityResidualRad: Double?,
    val windowSampleCount: Int,
    val eulerPitchMedianRad: Double?,
    val forwardAxisGradeMedianRad: Double?,
    val roadGradeMedianRad: Double?,
    val gravityPitchMedianRad: Double?,
    val axisConsistent: Boolean,
    val pitchDriftCandidate: Boolean,
    val valid: Boolean,
    val rejectionReason: String
)
