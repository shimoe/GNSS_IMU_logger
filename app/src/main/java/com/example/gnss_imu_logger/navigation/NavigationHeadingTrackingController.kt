package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.ln
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 確定したHeading方向を維持したまま、長時間ずれたyawをGNSS進行方位へ段階的に戻す。
 * 入力: 推定姿勢、GNSS方位、方向状態、観測品質、通常Heading更新結果
 * 出力: 制限付きyaw補正後の姿勢と追従状態の診断
 */
class NavigationHeadingTrackingController(
    private val parameters: NavigationHeadingTrackingParameters =
        NavigationHeadingTrackingParameters.DEFAULT
) {
    private var state = NavigationHeadingTrackingState.UNAVAILABLE
    private var consecutiveRejectionCount = 0
    private val candidates = ArrayDeque<HeadingCandidate>()
    private var completionCount = 0

    fun update(
        elapsedRealtimeNs: Long,
        attitudeBodyToEnu: AttitudeQuaternion,
        directionState: NavigationHeadingDirectionState,
        bearingRad: Double?,
        bearingAccuracyRad: Double?,
        speedAccuracyMps: Double?,
        speedMps: Double?,
        yawRateRadps: Double?,
        headingUpdateAccepted: Boolean,
        headingUpdateAvailable: Boolean,
        directionConflict: Boolean
    ): NavigationHeadingTrackingResult {
        val stateBefore = state
        val predictedYawRad = attitudeBodyToEnu.toEulerAngles().yawRad
        val targetHeadingRad = targetHeading(directionState, bearingRad)
        val rawInnovationRad = targetHeadingRad?.let { wrapAngle(it - predictedYawRad) }
        val qualityReason = qualityRejectionReason(
            directionState,
            bearingAccuracyRad,
            speedAccuracyMps,
            speedMps,
            yawRateRadps,
            targetHeadingRad
        )
        val rejected = directionConflict || (headingUpdateAvailable && !headingUpdateAccepted)
        if (headingUpdateAccepted) {
            consecutiveRejectionCount = 0
            if (state != NavigationHeadingTrackingState.REACQUIRING) {
                state = NavigationHeadingTrackingState.TRACKING
            }
        } else if (rejected) {
            consecutiveRejectionCount++
        }

        var reacquisitionStarted = false
        var correctionApplied = false
        var limitedCorrectionRad = 0.0
        var correctedAttitude = attitudeBodyToEnu
        var rejectionReason = qualityReason

        if (qualityReason.isEmpty() && rawInnovationRad != null) {
            if (consecutiveRejectionCount >= parameters.degradedRejectionCount &&
                state != NavigationHeadingTrackingState.REACQUIRING) {
                state = NavigationHeadingTrackingState.DEGRADED
            }
            if (state == NavigationHeadingTrackingState.DEGRADED) {
                addCandidate(elapsedRealtimeNs, targetHeadingRad!!)
                if (candidateReady(elapsedRealtimeNs)) {
                    state = NavigationHeadingTrackingState.REACQUIRING
                    reacquisitionStarted = true
                    completionCount = 0
                }
            }
            if (state == NavigationHeadingTrackingState.REACQUIRING) {
                addCandidate(elapsedRealtimeNs, targetHeadingRad!!)
                limitedCorrectionRad = rawInnovationRad.coerceIn(
                    -parameters.maximumCorrectionPerUpdateRad,
                    parameters.maximumCorrectionPerUpdateRad
                )
                correctedAttitude = attitudeBodyToEnu.correctedEnuYaw(limitedCorrectionRad)
                correctionApplied = abs(limitedCorrectionRad) > 0.0
                if (abs(rawInnovationRad) <= parameters.completionInnovationRad) {
                    completionCount++
                    if (completionCount >= parameters.completionCount) {
                        state = NavigationHeadingTrackingState.TRACKING
                        consecutiveRejectionCount = 0
                        candidates.clear()
                        completionCount = 0
                    }
                } else {
                    completionCount = 0
                }
            }
        } else {
            if (state == NavigationHeadingTrackingState.REACQUIRING) {
                state = NavigationHeadingTrackingState.DEGRADED
            } else if (!directionState.isConfirmed()) {
                state = NavigationHeadingTrackingState.UNAVAILABLE
            }
            if (candidateGapExceeded(elapsedRealtimeNs)) candidates.clear()
        }

        val circularStdRad = circularStd(candidates.map { it.headingRad })
        if (rejectionReason.isEmpty() && directionConflict) rejectionReason = "DIRECTION_CONFLICT"
        if (rejectionReason.isEmpty() && headingUpdateAvailable && !headingUpdateAccepted) {
            rejectionReason = "HEADING_INNOVATION"
        }
        return NavigationHeadingTrackingResult(
            attitudeBodyToEnu = correctedAttitude,
            record = NavigationHeadingTrackingRecord(
                elapsedRealtimeNs = elapsedRealtimeNs,
                directionState = directionState,
                stateBefore = stateBefore,
                stateAfter = state,
                predictedYawRad = predictedYawRad,
                targetHeadingRad = targetHeadingRad,
                rawInnovationRad = rawInnovationRad,
                limitedCorrectionRad = limitedCorrectionRad,
                correctedYawRad = correctedAttitude.toEulerAngles().yawRad,
                bearingAccuracyRad = bearingAccuracyRad,
                speedAccuracyMps = speedAccuracyMps,
                speedMps = speedMps,
                yawRateRadps = yawRateRadps,
                consecutiveRejectionCount = consecutiveRejectionCount,
                candidateCount = candidates.size,
                candidateDurationSec = candidateDurationSec(elapsedRealtimeNs),
                bearingCircularStdRad = circularStdRad,
                reacquisitionStarted = reacquisitionStarted,
                correctionApplied = correctionApplied,
                headingUpdateAccepted = headingUpdateAccepted,
                rejectionReason = rejectionReason
            )
        )
    }

    private fun addCandidate(elapsedRealtimeNs: Long, headingRad: Double) {
        if (candidateGapExceeded(elapsedRealtimeNs)) candidates.clear()
        candidates.addLast(HeadingCandidate(elapsedRealtimeNs, headingRad))
        while (candidates.size > parameters.maximumCandidateCount) candidates.removeFirst()
    }

    private fun candidateReady(elapsedRealtimeNs: Long): Boolean =
        candidates.size >= parameters.requiredCandidateCount &&
            candidateDurationNs(elapsedRealtimeNs) >= parameters.requiredCandidateDurationNs &&
            circularStd(candidates.map { it.headingRad }) <= parameters.maximumCircularStdRad

    private fun candidateGapExceeded(elapsedRealtimeNs: Long): Boolean =
        candidates.lastOrNull()?.let {
            elapsedRealtimeNs - it.elapsedRealtimeNs > parameters.maximumCandidateGapNs
        } ?: false

    private fun candidateDurationNs(elapsedRealtimeNs: Long): Long =
        candidates.firstOrNull()?.let { elapsedRealtimeNs - it.elapsedRealtimeNs } ?: 0L

    private fun candidateDurationSec(elapsedRealtimeNs: Long): Double =
        candidateDurationNs(elapsedRealtimeNs).coerceAtLeast(0L) / NS_PER_SEC

    private fun qualityRejectionReason(
        directionState: NavigationHeadingDirectionState,
        bearingAccuracyRad: Double?,
        speedAccuracyMps: Double?,
        speedMps: Double?,
        yawRateRadps: Double?,
        targetHeadingRad: Double?
    ): String = when {
        !directionState.isConfirmed() -> "DIRECTION_UNRESOLVED"
        targetHeadingRad == null -> "BEARING_UNAVAILABLE"
        speedMps == null || speedMps < parameters.minimumSpeedMps -> "SPEED"
        bearingAccuracyRad == null || bearingAccuracyRad > parameters.maximumBearingAccuracyRad ->
            "BEARING_ACCURACY"
        speedAccuracyMps == null || speedAccuracyMps > parameters.maximumSpeedAccuracyMps ->
            "SPEED_ACCURACY"
        yawRateRadps == null || abs(yawRateRadps) > parameters.maximumYawRateRadps -> "YAW_RATE"
        else -> ""
    }

    private fun targetHeading(
        directionState: NavigationHeadingDirectionState,
        bearingRad: Double?
    ): Double? = bearingRad?.let {
        when (directionState) {
            NavigationHeadingDirectionState.FORWARD -> it
            NavigationHeadingDirectionState.REVERSE -> wrapAngle(it + Math.PI)
            else -> null
        }
    }

    private fun circularStd(values: List<Double>): Double {
        if (values.isEmpty()) return Double.POSITIVE_INFINITY
        val meanCos = values.sumOf(::cos) / values.size
        val meanSin = values.sumOf(::sin) / values.size
        val resultant = sqrt(meanCos * meanCos + meanSin * meanSin).coerceIn(1e-12, 1.0)
        return sqrt(-2.0 * ln(resultant))
    }

    private fun wrapAngle(angleRad: Double): Double = atan2(sin(angleRad), cos(angleRad))
    private fun NavigationHeadingDirectionState.isConfirmed() =
        this == NavigationHeadingDirectionState.FORWARD || this == NavigationHeadingDirectionState.REVERSE

    private data class HeadingCandidate(val elapsedRealtimeNs: Long, val headingRad: Double)
    companion object { private const val NS_PER_SEC = 1_000_000_000.0 }
}

enum class NavigationHeadingTrackingState { UNAVAILABLE, TRACKING, DEGRADED, REACQUIRING }

data class NavigationHeadingTrackingParameters(
    val degradedRejectionCount: Int,
    val requiredCandidateCount: Int,
    val requiredCandidateDurationNs: Long,
    val maximumCandidateGapNs: Long,
    val maximumCandidateCount: Int,
    val maximumCircularStdRad: Double,
    val maximumCorrectionPerUpdateRad: Double,
    val completionInnovationRad: Double,
    val completionCount: Int,
    val minimumSpeedMps: Double,
    val maximumBearingAccuracyRad: Double,
    val maximumSpeedAccuracyMps: Double,
    val maximumYawRateRadps: Double
) {
    companion object {
        val DEFAULT = NavigationHeadingTrackingParameters(
            degradedRejectionCount = 5,
            requiredCandidateCount = 5,
            requiredCandidateDurationNs = 4_000_000_000L,
            maximumCandidateGapNs = 1_500_000_000L,
            maximumCandidateCount = 10,
            maximumCircularStdRad = Math.toRadians(10.0),
            maximumCorrectionPerUpdateRad = Math.toRadians(5.0),
            completionInnovationRad = Math.toRadians(15.0),
            completionCount = 3,
            minimumSpeedMps = 5.0,
            maximumBearingAccuracyRad = Math.toRadians(15.0),
            maximumSpeedAccuracyMps = 2.0,
            maximumYawRateRadps = 0.5
        )
    }
}

data class NavigationHeadingTrackingResult(
    val attitudeBodyToEnu: AttitudeQuaternion,
    val record: NavigationHeadingTrackingRecord
)

data class NavigationHeadingTrackingRecord(
    val elapsedRealtimeNs: Long,
    val directionState: NavigationHeadingDirectionState,
    val stateBefore: NavigationHeadingTrackingState,
    val stateAfter: NavigationHeadingTrackingState,
    val predictedYawRad: Double,
    val targetHeadingRad: Double?,
    val rawInnovationRad: Double?,
    val limitedCorrectionRad: Double,
    val correctedYawRad: Double,
    val bearingAccuracyRad: Double?,
    val speedAccuracyMps: Double?,
    val speedMps: Double?,
    val yawRateRadps: Double?,
    val consecutiveRejectionCount: Int,
    val candidateCount: Int,
    val candidateDurationSec: Double,
    val bearingCircularStdRad: Double,
    val reacquisitionStarted: Boolean,
    val correctionApplied: Boolean,
    val headingUpdateAccepted: Boolean,
    val rejectionReason: String
)
