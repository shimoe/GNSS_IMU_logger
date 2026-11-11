package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * GNSS水平速度innovationから車体水平軸の加速度バイアスを低ゲインで更新する。
 * 入力: GNSS更新前状態、GNSS速度、GNSS間隔[s]、姿勢、走行状態
 * 出力: 更新後バイアスと診断結果
 */
class NavigationAccelerometerBiasUpdater(
    private val parameters: NavigationAccelerometerBiasParameters =
        NavigationAccelerometerBiasParameters.DEFAULT
) {
    private val forwardCandidate = AxisCandidate()
    private val leftCandidate = AxisCandidate()

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
        val reason = rejectionReason(
            stateBeforeGnss = stateBeforeGnss,
            gnssVelocity = gnssVelocity,
            gnssIntervalSec = gnssIntervalSec,
            gnssSpeedStdMps = gnssSpeedStdMps,
            velocityAccepted = velocityAccepted,
            directionState = directionState,
            headingTrackingState = headingTrackingState,
            yawRateRadps = yawRateRadps,
            accelerationBodyMps2 = accelerationBodyMps2
        )
        if (reason.isNotEmpty()) {
            forwardCandidate.reset()
            leftCandidate.reset()
            return rejected(elapsedRealtimeNs, stateBeforeGnss, reason, gnssIntervalSec)
        }

        val velocityInnovationEnu = BodyVector(
            x = stateBeforeGnss.velocityEnuMps.eastMps - gnssVelocity!!.eastMps,
            y = stateBeforeGnss.velocityEnuMps.northMps - gnssVelocity.northMps,
            z = 0.0
        )
        val accelerationErrorEnu = BodyVector(
            x = velocityInnovationEnu.x / gnssIntervalSec!!,
            y = velocityInnovationEnu.y / gnssIntervalSec,
            z = 0.0
        )
        val accelerationErrorBody = stateBeforeGnss.attitudeBodyToEnu
            .rotateEnuToBody(accelerationErrorEnu)
        val forwardReady = forwardCandidate.add(
            elapsedRealtimeNs,
            accelerationErrorBody.x,
            parameters
        )
        val leftReady = leftCandidate.add(
            elapsedRealtimeNs,
            accelerationErrorBody.y,
            parameters
        )
        val forwardCandidateCount = forwardCandidate.count
        val leftCandidateCount = leftCandidate.count
        val forwardCandidateDurationSec = forwardCandidate.durationSec(elapsedRealtimeNs)
        val leftCandidateDurationSec = leftCandidate.durationSec(elapsedRealtimeNs)
        val biasBefore = stateBeforeGnss.accelerometerBiasMps2
        val forwardCorrection = correction(accelerationErrorBody.x, forwardReady)
        val leftCorrection = correction(accelerationErrorBody.y, leftReady)
        val biasAfter = BodyVector(
            x = (biasBefore.x + forwardCorrection).coerceIn(
                -parameters.maximumAbsoluteBiasMps2,
                parameters.maximumAbsoluteBiasMps2
            ),
            y = (biasBefore.y + leftCorrection).coerceIn(
                -parameters.maximumAbsoluteBiasMps2,
                parameters.maximumAbsoluteBiasMps2
            ),
            z = biasBefore.z
        )
        if (forwardReady) forwardCandidate.reset()
        if (leftReady) leftCandidate.reset()
        return NavigationAccelerometerBiasUpdateResult(
            accelerometerBiasMps2 = biasAfter,
            record = NavigationAccelerometerBiasUpdateRecord(
                elapsedRealtimeNs = elapsedRealtimeNs,
                gnssIntervalSec = gnssIntervalSec,
                velocityInnovationEastMps = velocityInnovationEnu.x,
                velocityInnovationNorthMps = velocityInnovationEnu.y,
                accelerationErrorEnuEastMps2 = accelerationErrorEnu.x,
                accelerationErrorEnuNorthMps2 = accelerationErrorEnu.y,
                accelerationErrorBodyForwardMps2 = accelerationErrorBody.x,
                accelerationErrorBodyLeftMps2 = accelerationErrorBody.y,
                biasForwardBeforeMps2 = biasBefore.x,
                biasLeftBeforeMps2 = biasBefore.y,
                biasForwardCorrectionMps2 = biasAfter.x - biasBefore.x,
                biasLeftCorrectionMps2 = biasAfter.y - biasBefore.y,
                biasForwardAfterMps2 = biasAfter.x,
                biasLeftAfterMps2 = biasAfter.y,
                forwardCandidateCount = forwardCandidateCount,
                leftCandidateCount = leftCandidateCount,
                forwardCandidateDurationSec = forwardCandidateDurationSec,
                leftCandidateDurationSec = leftCandidateDurationSec,
                forwardAccepted = forwardReady,
                leftAccepted = leftReady,
                rejectionReason = ""
            )
        )
    }

    private fun rejectionReason(
        stateBeforeGnss: NavigationState,
        gnssVelocity: EnuVelocity?,
        gnssIntervalSec: Double?,
        gnssSpeedStdMps: Double?,
        velocityAccepted: Boolean,
        directionState: NavigationHeadingDirectionState,
        headingTrackingState: NavigationHeadingTrackingState,
        yawRateRadps: Double?,
        accelerationBodyMps2: BodyVector?
    ): String {
        if (!velocityAccepted || gnssVelocity == null) return "VELOCITY_UPDATE"
        if (gnssIntervalSec == null ||
            gnssIntervalSec !in parameters.minimumGnssIntervalSec..parameters.maximumGnssIntervalSec
        ) return "GNSS_INTERVAL"
        if (gnssSpeedStdMps == null || gnssSpeedStdMps > parameters.maximumSpeedStdMps) {
            return "SPEED_ACCURACY"
        }
        if (directionState != NavigationHeadingDirectionState.FORWARD &&
            directionState != NavigationHeadingDirectionState.REVERSE
        ) return "DIRECTION_UNRESOLVED"
        if (headingTrackingState != NavigationHeadingTrackingState.TRACKING) {
            return "HEADING_NOT_TRACKING"
        }
        if (yawRateRadps == null || abs(yawRateRadps) > parameters.maximumYawRateRadps) {
            return "YAW_RATE"
        }
        if (accelerationBodyMps2 == null) return "ACCELERATION_UNAVAILABLE"
        // 生の車体加速度には路面勾配に応じた重力成分が含まれるため、
        // 重力除去後ENU加速度を車体へ戻して走行加速度だけを判定する。
        val dynamicAccelerationBody = stateBeforeGnss.attitudeBodyToEnu
            .rotateEnuToBody(stateBeforeGnss.accelerationEnuMps2)
        if (abs(dynamicAccelerationBody.x) > parameters.maximumForwardAccelerationMps2) {
            return "FORWARD_ACCELERATION"
        }
        val horizontalAccelerationMps2 = sqrt(
            stateBeforeGnss.accelerationEnuMps2.x * stateBeforeGnss.accelerationEnuMps2.x +
                stateBeforeGnss.accelerationEnuMps2.y * stateBeforeGnss.accelerationEnuMps2.y
        )
        if (horizontalAccelerationMps2 > parameters.maximumHorizontalAccelerationMps2) {
            return "HORIZONTAL_ACCELERATION"
        }
        val horizontalSpeedMps = sqrt(
            gnssVelocity.eastMps * gnssVelocity.eastMps +
                gnssVelocity.northMps * gnssVelocity.northMps
        )
        return if (horizontalSpeedMps < parameters.minimumSpeedMps) "SPEED" else ""
    }

    private fun correction(errorMps2: Double, accepted: Boolean): Double =
        if (!accepted) 0.0 else (parameters.updateGain * errorMps2).coerceIn(
            -parameters.maximumCorrectionPerUpdateMps2,
            parameters.maximumCorrectionPerUpdateMps2
        )

    private fun rejected(
        elapsedRealtimeNs: Long,
        state: NavigationState,
        reason: String,
        gnssIntervalSec: Double?
    ) = NavigationAccelerometerBiasUpdateResult(
        accelerometerBiasMps2 = state.accelerometerBiasMps2,
        record = NavigationAccelerometerBiasUpdateRecord.rejected(
            elapsedRealtimeNs,
            gnssIntervalSec,
            state.accelerometerBiasMps2,
            reason
        )
    )

    private class AxisCandidate {
        var count: Int = 0
            private set
        private var signValue: Double = 0.0
        private var startedNs: Long? = null
        private var lastNs: Long? = null

        fun add(
            elapsedRealtimeNs: Long,
            value: Double,
            parameters: NavigationAccelerometerBiasParameters
        ): Boolean {
            if (!value.isFinite() || abs(value) < parameters.minimumCandidateErrorMps2) {
                reset()
                return false
            }
            if (lastNs?.let { elapsedRealtimeNs - it > parameters.maximumCandidateGapNs } == true ||
                sign(value) != signValue
            ) {
                reset()
            }
            if (count == 0) {
                signValue = sign(value)
                startedNs = elapsedRealtimeNs
            }
            count++
            lastNs = elapsedRealtimeNs
            return count >= parameters.requiredCandidateCount &&
                durationNs(elapsedRealtimeNs) >= parameters.requiredCandidateDurationNs
        }

        fun durationSec(nowNs: Long): Double = durationNs(nowNs) / NS_PER_SEC
        private fun durationNs(nowNs: Long): Long = startedNs?.let { nowNs - it } ?: 0L
        fun reset() {
            count = 0
            signValue = 0.0
            startedNs = null
            lastNs = null
        }
    }

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
    }
}

data class NavigationAccelerometerBiasParameters(
    val updateGain: Double,
    val maximumCorrectionPerUpdateMps2: Double,
    val maximumAbsoluteBiasMps2: Double,
    val minimumCandidateErrorMps2: Double,
    val requiredCandidateCount: Int,
    val requiredCandidateDurationNs: Long,
    val maximumCandidateGapNs: Long,
    val minimumGnssIntervalSec: Double,
    val maximumGnssIntervalSec: Double,
    val maximumSpeedStdMps: Double,
    val maximumYawRateRadps: Double,
    val maximumForwardAccelerationMps2: Double,
    val maximumHorizontalAccelerationMps2: Double,
    val minimumSpeedMps: Double
) {
    companion object {
        val DEFAULT = NavigationAccelerometerBiasParameters(
            updateGain = 0.02,
            maximumCorrectionPerUpdateMps2 = 0.02,
            maximumAbsoluteBiasMps2 = 0.5,
            minimumCandidateErrorMps2 = 0.05,
            requiredCandidateCount = 3,
            requiredCandidateDurationNs = 2_000_000_000L,
            maximumCandidateGapNs = 2_000_000_000L,
            minimumGnssIntervalSec = 0.5,
            maximumGnssIntervalSec = 2.0,
            maximumSpeedStdMps = 2.0,
            maximumYawRateRadps = 0.35,
            maximumForwardAccelerationMps2 = 2.0,
            maximumHorizontalAccelerationMps2 = 3.0,
            minimumSpeedMps = 5.0
        )
    }
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
    val accelerationErrorEnuEastMps2: Double?,
    val accelerationErrorEnuNorthMps2: Double?,
    val accelerationErrorBodyForwardMps2: Double?,
    val accelerationErrorBodyLeftMps2: Double?,
    val biasForwardBeforeMps2: Double,
    val biasLeftBeforeMps2: Double,
    val biasForwardCorrectionMps2: Double,
    val biasLeftCorrectionMps2: Double,
    val biasForwardAfterMps2: Double,
    val biasLeftAfterMps2: Double,
    val forwardCandidateCount: Int,
    val leftCandidateCount: Int,
    val forwardCandidateDurationSec: Double,
    val leftCandidateDurationSec: Double,
    val forwardAccepted: Boolean,
    val leftAccepted: Boolean,
    val rejectionReason: String
) {
    companion object {
        fun rejected(
            elapsedRealtimeNs: Long,
            gnssIntervalSec: Double?,
            bias: BodyVector,
            reason: String
        ) = NavigationAccelerometerBiasUpdateRecord(
            elapsedRealtimeNs, gnssIntervalSec,
            null, null, null, null, null, null,
            bias.x, bias.y, 0.0, 0.0, bias.x, bias.y,
            0, 0, 0.0, 0.0, false, false, reason
        )
    }
}
/**
 * ENU座標のベクトルを車体座標へ回転する。
 * 入力: ENU座標ベクトル
 * 出力: 車体座標ベクトル
 * ナビゲーション専用処理とし、計測画面が使用するQuaternion APIは変更しない。
 */
private fun AttitudeQuaternion.rotateEnuToBody(vector: BodyVector): BodyVector {
    val inverse = normalized().conjugate()
    return inverse.rotate(vector)
}
