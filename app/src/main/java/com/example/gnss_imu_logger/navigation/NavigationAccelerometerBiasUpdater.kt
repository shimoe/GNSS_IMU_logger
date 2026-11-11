package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * 車体軸速度innovationの区間回帰を管理する。
 * 初期設定では推定のみを行い、NavigationStateのバイアスは変更しない。
 */
class NavigationAccelerometerBiasUpdater(
    private val parameters: NavigationAccelerometerBiasWindowParameters =
        NavigationAccelerometerBiasWindowParameters.DEFAULT
) {
    private val window = NavigationAccelerometerBiasWindow(
        parameters.maximumDurationNs,
        parameters.maximumValidSampleGapNs,
        parameters.maximumHoldNs
    )
    private val estimator = NavigationAccelerometerBiasEstimator(parameters)

    fun update(
        elapsedRealtimeNs: Long,
        stateBeforeGnss: NavigationState,
        gnssVelocity: EnuVelocity?,
        gnssIntervalSec: Double?,
        gnssSpeedStdMps: Double?,
        velocityAccepted: Boolean,
        directionState: NavigationHeadingDirectionState,
        headingTrackingState: NavigationHeadingTrackingState,
        yawRateRadps: Double?,
        accelerationBodyMps2: BodyVector?
    ): NavigationAccelerometerBiasUpdateResult {
        val gate = classifyGate(
            stateBeforeGnss,
            gnssVelocity,
            gnssIntervalSec,
            gnssSpeedStdMps,
            velocityAccepted,
            directionState,
            headingTrackingState,
            yawRateRadps,
            accelerationBodyMps2
        )
        var completed: NavigationBiasWindowSnapshot? = null
        var innovationEast: Double? = null
        var innovationNorth: Double? = null
        when (gate.action) {
            NavigationBiasWindowAction.RESET -> {
                completed = window.finish(elapsedRealtimeNs, gate.reason)
            }
            NavigationBiasWindowAction.HOLD -> {
                completed = window.hold(elapsedRealtimeNs, gate.reason)
            }
            NavigationBiasWindowAction.ADD -> {
                innovationEast = stateBeforeGnss.velocityEnuMps.eastMps - gnssVelocity!!.eastMps
                innovationNorth = stateBeforeGnss.velocityEnuMps.northMps - gnssVelocity.northMps
                val bodyInnovation = stateBeforeGnss.attitudeBodyToEnu.normalized().conjugate().rotate(
                    BodyVector(innovationEast, innovationNorth, 0.0)
                )
                completed = window.add(
                    NavigationAccelerometerBiasSample(
                        elapsedRealtimeNs = elapsedRealtimeNs,
                        velocityInnovationEastMps = innovationEast,
                        velocityInnovationNorthMps = innovationNorth,
                        velocityInnovationBodyForwardMps = bodyInnovation.x,
                        velocityInnovationBodyLeftMps = bodyInnovation.y,
                        yawRad = stateBeforeGnss.attitudeBodyToEnu.toEulerAngles().yawRad,
                        speedMps = sqrt(
                            gnssVelocity.eastMps * gnssVelocity.eastMps +
                                gnssVelocity.northMps * gnssVelocity.northMps
                        ),
                        yawRateRadps = yawRateRadps!!
                    )
                )
            }
        }
        val ready = window.snapshotWhenReady(
            parameters.minimumUpdateSampleCount,
            parameters.minimumUpdateDurationNs
        )
        val snapshot = completed ?: ready
        val estimate = snapshot?.let(estimator::estimate)
        // 終了済みの旧窓を評価した場合、現在収集中の新しい窓の状態は変更しない。
        if (estimate != null && ready != null) window.markEstimated()
        val before = stateBeforeGnss.accelerometerBiasMps2
        val forwardReason = axisRejectionReason(estimate, true)
        val leftReason = axisRejectionReason(estimate, false)
        val wouldAcceptForward = forwardReason.isEmpty()
        val wouldAcceptLeft = leftReason.isEmpty()
        val proposedForward = proposedCorrection(
            estimate?.bodyForwardSlopeMps2,
            estimate?.bodyForwardResidualMadMps,
            wouldAcceptForward
        )
        val proposedLeft = proposedCorrection(
            estimate?.bodyLeftSlopeMps2,
            estimate?.bodyLeftResidualMadMps,
            wouldAcceptLeft
        )
        val applyForward = parameters.applyBiasUpdate && wouldAcceptForward
        val applyLeft = parameters.applyBiasUpdate && wouldAcceptLeft
        val after = BodyVector(
            x = (before.x + if (applyForward) proposedForward else 0.0).coerceIn(
                -parameters.maximumAbsoluteBiasMps2, parameters.maximumAbsoluteBiasMps2
            ),
            y = (before.y + if (applyLeft) proposedLeft else 0.0).coerceIn(
                -parameters.maximumAbsoluteBiasMps2, parameters.maximumAbsoluteBiasMps2
            ),
            z = before.z
        )
        val record = NavigationAccelerometerBiasUpdateRecord(
            elapsedRealtimeNs = elapsedRealtimeNs,
            gnssIntervalSec = gnssIntervalSec,
            velocityInnovationEastMps = innovationEast,
            velocityInnovationNorthMps = innovationNorth,
            biasForwardBeforeMps2 = before.x,
            biasLeftBeforeMps2 = before.y,
            biasForwardCorrectionMps2 = after.x - before.x,
            biasLeftCorrectionMps2 = after.y - before.y,
            biasForwardAfterMps2 = after.x,
            biasLeftAfterMps2 = after.y,
            windowSequence = estimate?.windowSequence ?: window.sequence,
            windowStartedNs = estimate?.windowStartedElapsedRealtimeNs ?: window.startedNs(),
            windowLastSampleNs = estimate?.windowLastSampleElapsedRealtimeNs ?: window.lastSampleNs(),
            windowCompletedNs = estimate?.windowCompletedElapsedRealtimeNs,
            windowState = if (estimate != null) NavigationBiasWindowState.ESTIMATED else window.state,
            holdReason = window.holdReason,
            windowEndReason = estimate?.windowEndReason ?: window.windowEndReason,
            holdDurationSec = window.holdDurationNs(elapsedRealtimeNs) / NS_PER_SEC,
            validSampleGapSec = window.lastValidSampleGapNs()?.div(NS_PER_SEC),
            windowSampleCount = estimate?.sampleCount ?: window.size(),
            validDurationSec = estimate?.durationSec ?: window.validDurationNs() / NS_PER_SEC,
            windowYawChangeRad = estimate?.yawChangeRad,
            eastSlopeMps2 = estimate?.eastSlopeMps2,
            northSlopeMps2 = estimate?.northSlopeMps2,
            eastResidualMadMps = estimate?.eastResidualMadMps,
            northResidualMadMps = estimate?.northResidualMadMps,
            bodyForwardInnovationFirstMps = estimate?.bodyForwardInnovationFirstMps,
            bodyForwardInnovationLastMps = estimate?.bodyForwardInnovationLastMps,
            bodyLeftInnovationFirstMps = estimate?.bodyLeftInnovationFirstMps,
            bodyLeftInnovationLastMps = estimate?.bodyLeftInnovationLastMps,
            bodyForwardSlopeMps2 = estimate?.bodyForwardSlopeMps2,
            bodyLeftSlopeMps2 = estimate?.bodyLeftSlopeMps2,
            bodyForwardResidualMadMps = estimate?.bodyForwardResidualMadMps,
            bodyLeftResidualMadMps = estimate?.bodyLeftResidualMadMps,
            forwardSignRatio = estimate?.forwardSignRatio,
            leftSignRatio = estimate?.leftSignRatio,
            estimateAvailable = estimate != null,
            updateEnabled = parameters.applyBiasUpdate,
            wouldAcceptForward = wouldAcceptForward,
            wouldAcceptLeft = wouldAcceptLeft,
            proposedForwardCorrectionMps2 = proposedForward,
            proposedLeftCorrectionMps2 = proposedLeft,
            forwardAccepted = applyForward,
            leftAccepted = applyLeft,
            forwardRejectionReason = forwardReason,
            leftRejectionReason = leftReason,
            rejectionReason = gate.reason
        )
        if (ready != null) window.finish(elapsedRealtimeNs, "ESTIMATED_AT_THRESHOLD")
        return NavigationAccelerometerBiasUpdateResult(after, record)
    }

    private fun axisRejectionReason(
        estimate: NavigationAccelerometerBiasEstimate?,
        forward: Boolean
    ): String {
        if (estimate == null) return "ESTIMATE_UNAVAILABLE"
        if (estimate.sampleCount < parameters.minimumUpdateSampleCount) return "LOW_SAMPLE"
        if (estimate.durationSec < parameters.minimumUpdateDurationNs / NS_PER_SEC) return "SHORT_DURATION"
        val yawLimit = if (forward) parameters.maximumForwardYawChangeRad
        else parameters.maximumLeftYawChangeRad
        if (abs(estimate.yawChangeRad) > yawLimit) return "YAW_CHANGE"
        val residual = if (forward) estimate.bodyForwardResidualMadMps else estimate.bodyLeftResidualMadMps
        val residualLimit = if (forward) parameters.maximumForwardResidualMadMps
        else parameters.maximumLeftResidualMadMps
        if (residual > residualLimit) return "RESIDUAL_MAD"
        val signRatio = if (forward) estimate.forwardSignRatio else estimate.leftSignRatio
        val signLimit = if (forward) parameters.minimumForwardSignRatio else parameters.minimumLeftSignRatio
        if (signRatio < signLimit) return "SIGN_RATIO"
        val slope = if (forward) estimate.bodyForwardSlopeMps2 else estimate.bodyLeftSlopeMps2
        val minimumSlope = if (forward) parameters.minimumForwardSlopeMps2 else parameters.minimumLeftSlopeMps2
        val maximumSlope = if (forward) parameters.maximumForwardSlopeMps2 else parameters.maximumLeftSlopeMps2
        if (abs(slope) < minimumSlope) return "SLOPE_SMALL"
        if (abs(slope) > maximumSlope) return "SLOPE_LARGE"
        return ""
    }

    private fun proposedCorrection(slope: Double?, residualMad: Double?, accepted: Boolean): Double {
        if (!accepted || slope == null || residualMad == null) return 0.0
        val gain = if (residualMad <= 0.3) 0.02 else 0.01
        return (gain * slope).coerceIn(
            -parameters.maximumCorrectionPerUpdateMps2,
            parameters.maximumCorrectionPerUpdateMps2
        )
    }

    private fun classifyGate(
        state: NavigationState,
        gnssVelocity: EnuVelocity?,
        intervalSec: Double?,
        speedStdMps: Double?,
        velocityAccepted: Boolean,
        direction: NavigationHeadingDirectionState,
        tracking: NavigationHeadingTrackingState,
        yawRateRadps: Double?,
        accelerationBodyMps2: BodyVector?
    ): GateResult {
        if (direction != NavigationHeadingDirectionState.FORWARD &&
            direction != NavigationHeadingDirectionState.REVERSE
        ) return GateResult(NavigationBiasWindowAction.RESET, "DIRECTION_UNRESOLVED")
        if (tracking == NavigationHeadingTrackingState.REACQUIRING) {
            return GateResult(NavigationBiasWindowAction.RESET, "HEADING_REACQUIRING")
        }
        if (intervalSec == null || intervalSec <= 0.0 || intervalSec > 4.0) {
            return GateResult(NavigationBiasWindowAction.RESET, "GNSS_INTERVAL")
        }
        if (gnssVelocity == null) return GateResult(NavigationBiasWindowAction.RESET, "VELOCITY_UNAVAILABLE")
        val speed = sqrt(gnssVelocity.eastMps * gnssVelocity.eastMps +
            gnssVelocity.northMps * gnssVelocity.northMps)
        if (speed < 3.0) return GateResult(NavigationBiasWindowAction.RESET, "SPEED_RESET")
        if (!velocityAccepted) return GateResult(NavigationBiasWindowAction.HOLD, "VELOCITY_UPDATE")
        if (tracking != NavigationHeadingTrackingState.TRACKING) {
            return GateResult(NavigationBiasWindowAction.HOLD, "HEADING_NOT_TRACKING")
        }
        if (speedStdMps == null || speedStdMps > 2.0) {
            return GateResult(NavigationBiasWindowAction.HOLD, "SPEED_ACCURACY")
        }
        if (yawRateRadps == null || abs(yawRateRadps) > 0.25) {
            return GateResult(NavigationBiasWindowAction.HOLD, "YAW_RATE")
        }
        if (accelerationBodyMps2 == null) {
            return GateResult(NavigationBiasWindowAction.HOLD, "ACCELERATION_UNAVAILABLE")
        }
        val dynamicBody = state.attitudeBodyToEnu.normalized().conjugate().rotate(state.accelerationEnuMps2)
        if (abs(dynamicBody.x) > 2.5) {
            return GateResult(NavigationBiasWindowAction.HOLD, "FORWARD_ACCELERATION")
        }
        val horizontalAcceleration = sqrt(
            state.accelerationEnuMps2.x * state.accelerationEnuMps2.x +
                state.accelerationEnuMps2.y * state.accelerationEnuMps2.y
        )
        if (horizontalAcceleration > 4.0) {
            return GateResult(NavigationBiasWindowAction.HOLD, "HORIZONTAL_ACCELERATION")
        }
        if (speed < 5.0) return GateResult(NavigationBiasWindowAction.HOLD, "SPEED")
        return GateResult(NavigationBiasWindowAction.ADD, "")
    }

    private data class GateResult(val action: NavigationBiasWindowAction, val reason: String)
    private enum class NavigationBiasWindowAction { ADD, HOLD, RESET }
    companion object { private const val NS_PER_SEC = 1_000_000_000.0 }
}

data class NavigationAccelerometerBiasUpdateResult(
    val accelerometerBiasMps2: BodyVector,
    val record: NavigationAccelerometerBiasUpdateRecord
)

data class NavigationAccelerometerBiasUpdateRecord(
    val elapsedRealtimeNs: Long,
    val gnssIntervalSec: Double?,
    val velocityInnovationEastMps: Double?,
    val velocityInnovationNorthMps: Double?,
    val biasForwardBeforeMps2: Double,
    val biasLeftBeforeMps2: Double,
    val biasForwardCorrectionMps2: Double,
    val biasLeftCorrectionMps2: Double,
    val biasForwardAfterMps2: Double,
    val biasLeftAfterMps2: Double,
    val windowSequence: Long,
    val windowStartedNs: Long?,
    val windowLastSampleNs: Long?,
    val windowCompletedNs: Long?,
    val windowState: NavigationBiasWindowState,
    val holdReason: String,
    val windowEndReason: String,
    val holdDurationSec: Double,
    val validSampleGapSec: Double?,
    val windowSampleCount: Int,
    val validDurationSec: Double,
    val windowYawChangeRad: Double?,
    val eastSlopeMps2: Double?,
    val northSlopeMps2: Double?,
    val eastResidualMadMps: Double?,
    val northResidualMadMps: Double?,
    val bodyForwardInnovationFirstMps: Double?,
    val bodyForwardInnovationLastMps: Double?,
    val bodyLeftInnovationFirstMps: Double?,
    val bodyLeftInnovationLastMps: Double?,
    val bodyForwardSlopeMps2: Double?,
    val bodyLeftSlopeMps2: Double?,
    val bodyForwardResidualMadMps: Double?,
    val bodyLeftResidualMadMps: Double?,
    val forwardSignRatio: Double?,
    val leftSignRatio: Double?,
    val estimateAvailable: Boolean,
    val updateEnabled: Boolean,
    val wouldAcceptForward: Boolean,
    val wouldAcceptLeft: Boolean,
    val proposedForwardCorrectionMps2: Double,
    val proposedLeftCorrectionMps2: Double,
    val forwardAccepted: Boolean,
    val leftAccepted: Boolean,
    val forwardRejectionReason: String,
    val leftRejectionReason: String,
    val rejectionReason: String
)
