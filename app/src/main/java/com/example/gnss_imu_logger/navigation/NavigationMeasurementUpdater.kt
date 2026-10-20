package com.example.gnss_imu_logger.navigation

import kotlin.math.max
import kotlin.math.sqrt

/**
 * GNSS位置・速度でナビゲーション状態を軸ごとに補正する。
 * 入力: 予測状態、各軸分散、GNSS更新
 * 出力: 補正後状態、補正後分散、位置・速度それぞれの採用結果
 *
 * 位置と速度は独立して判定する。速度異常だけを理由に有効な位置更新を捨てない。
 * 完全な15状態ESKFへ進む前に、GNSS閉ループで位置・速度発散を拘束できるか確認する実装である。
 */
class NavigationMeasurementUpdater(
    private val parameters: NavigationMeasurementParameters = NavigationMeasurementParameters.DEFAULT
) {
    fun update(
        state: NavigationState,
        variance: NavigationAxisVariance,
        measurement: NavigationGnssUpdate
    ): NavigationMeasurementUpdateResult {
        val positionInnovation = EnuPosition(
            eastM = measurement.positionEnuM.eastM - state.positionEnuM.eastM,
            northM = measurement.positionEnuM.northM - state.positionEnuM.northM,
            upM = measurement.positionEnuM.upM - state.positionEnuM.upM
        )
        val horizontalInnovationM = sqrt(
            positionInnovation.eastM * positionInnovation.eastM +
                positionInnovation.northM * positionInnovation.northM
        )
        val horizontalPositionGateM = max(
            parameters.minimumHorizontalPositionGateM,
            parameters.innovationSigmaGate * sqrt(
                max(variance.positionEastM2, variance.positionNorthM2) +
                    measurement.positionStdM * measurement.positionStdM
            )
        )
        val verticalPositionGateM = max(
            parameters.minimumVerticalPositionGateM,
            parameters.innovationSigmaGate * sqrt(
                variance.positionUpM2 + measurement.verticalStdM * measurement.verticalStdM
            )
        )
        val positionAccepted =
            horizontalInnovationM <= horizontalPositionGateM &&
                kotlin.math.abs(positionInnovation.upM) <= verticalPositionGateM

        val positionResult = if (positionAccepted) {
            updatePosition(state, variance, measurement, positionInnovation)
        } else PositionUpdate(state.positionEnuM, variance, null)

        val velocityMeasurement = measurement.velocityEnuMps
        val velocityResult = if (velocityMeasurement == null) {
            VelocityUpdate(state.velocityEnuMps, positionResult.variance, null, false, null)
        } else {
            updateVelocity(
                predictedVelocity = state.velocityEnuMps,
                variance = positionResult.variance,
                measurement = velocityMeasurement,
                speedStdMps = measurement.speedStdMps
            )
        }

        return NavigationMeasurementUpdateResult(
            state = state.copy(
                positionEnuM = positionResult.position,
                velocityEnuMps = velocityResult.velocity
            ),
            variance = velocityResult.variance,
            positionAccepted = positionAccepted,
            velocityAccepted = velocityResult.accepted,
            horizontalPositionInnovationM = horizontalInnovationM,
            velocityInnovationMps = velocityResult.innovationMps,
            positionGain = positionResult.gain,
            velocityGain = velocityResult.gain,
            horizontalPositionGateM = horizontalPositionGateM,
            verticalPositionGateM = verticalPositionGateM,
            velocityGateMps = velocityResult.gateMps,
            updateMode = if (positionAccepted) {
                NavigationPositionUpdateMode.NORMAL_UPDATE
            } else {
                NavigationPositionUpdateMode.REJECTED
            },
            rejectionReason = rejectionReason(
                positionAccepted = positionAccepted,
                velocityAvailable = velocityMeasurement != null,
                velocityAccepted = velocityResult.accepted
            )
        )
    }

    /**
     * 高品質GNSSを使って位置・速度状態を再捕捉する。
     * 入力: 予測状態、GNSS更新
     * 出力: GNSS位置と利用可能な速度で再初期化した状態・分散
     */
    fun reacquire(
        state: NavigationState,
        measurement: NavigationGnssUpdate
    ): NavigationMeasurementUpdateResult {
        val velocity = measurement.velocityEnuMps ?: EnuVelocity(
            eastMps = state.velocityEnuMps.eastMps,
            northMps = state.velocityEnuMps.northMps,
            upMps = 0.0
        )
        val horizontalVarianceM2 = max(
            parameters.minimumVariance,
            measurement.positionStdM * measurement.positionStdM
        )
        val verticalVarianceM2 = max(
            parameters.minimumVariance,
            measurement.verticalStdM * measurement.verticalStdM
        )
        val speedVarianceMps2 = max(
            measurement.speedStdMps,
            parameters.minimumSpeedStdMps
        ).let { it * it }
        val east = measurement.positionEnuM.eastM - state.positionEnuM.eastM
        val north = measurement.positionEnuM.northM - state.positionEnuM.northM
        return NavigationMeasurementUpdateResult(
            state = state.copy(
                positionEnuM = measurement.positionEnuM,
                velocityEnuMps = velocity
            ),
            variance = NavigationAxisVariance(
                positionEastM2 = horizontalVarianceM2,
                positionNorthM2 = horizontalVarianceM2,
                positionUpM2 = verticalVarianceM2,
                velocityEastMps2 = speedVarianceMps2,
                velocityNorthMps2 = speedVarianceMps2,
                velocityUpMps2 = speedVarianceMps2
            ),
            positionAccepted = true,
            velocityAccepted = measurement.velocityEnuMps != null,
            horizontalPositionInnovationM = sqrt(east * east + north * north),
            velocityInnovationMps = null,
            positionGain = null,
            velocityGain = null,
            horizontalPositionGateM = Double.NaN,
            verticalPositionGateM = Double.NaN,
            velocityGateMps = null,
            updateMode = NavigationPositionUpdateMode.REACQUIRED,
            rejectionReason = null
        )
    }

    private fun updatePosition(
        state: NavigationState,
        variance: NavigationAxisVariance,
        measurement: NavigationGnssUpdate,
        innovation: EnuPosition
    ): PositionUpdate {
        val horizontalNoiseM2 = measurement.positionStdM * measurement.positionStdM
        val verticalNoiseM2 = measurement.verticalStdM * measurement.verticalStdM
        val gain = AxisGain(
            east = gain(variance.positionEastM2, horizontalNoiseM2),
            north = gain(variance.positionNorthM2, horizontalNoiseM2),
            up = gain(variance.positionUpM2, verticalNoiseM2)
        )
        return PositionUpdate(
            position = EnuPosition(
                eastM = state.positionEnuM.eastM + gain.east * innovation.eastM,
                northM = state.positionEnuM.northM + gain.north * innovation.northM,
                upM = state.positionEnuM.upM + gain.up * innovation.upM
            ),
            variance = variance.copy(
                positionEastM2 = reducedVariance(variance.positionEastM2, gain.east),
                positionNorthM2 = reducedVariance(variance.positionNorthM2, gain.north),
                positionUpM2 = reducedVariance(variance.positionUpM2, gain.up)
            ),
            gain = gain
        )
    }

    private fun updateVelocity(
        predictedVelocity: EnuVelocity,
        variance: NavigationAxisVariance,
        measurement: EnuVelocity,
        speedStdMps: Double
    ): VelocityUpdate {
        val innovation = EnuVelocity(
            eastMps = measurement.eastMps - predictedVelocity.eastMps,
            northMps = measurement.northMps - predictedVelocity.northMps,
            upMps = measurement.upMps - predictedVelocity.upMps
        )
        val innovationMps = sqrt(
            innovation.eastMps * innovation.eastMps +
                innovation.northMps * innovation.northMps +
                innovation.upMps * innovation.upMps
        )
        val observationNoiseMps2 = max(
            speedStdMps,
            parameters.minimumSpeedStdMps
        ).let { it * it }
        val velocityGateMps = max(
            parameters.minimumVelocityGateMps,
            parameters.innovationSigmaGate * sqrt(
                maxOf(
                    variance.velocityEastMps2,
                    variance.velocityNorthMps2,
                    variance.velocityUpMps2
                ) + observationNoiseMps2
            )
        )
        if (innovationMps > velocityGateMps) {
            return VelocityUpdate(predictedVelocity, variance, innovationMps, false, velocityGateMps)
        }
        val gain = AxisGain(
            east = gain(variance.velocityEastMps2, observationNoiseMps2),
            north = gain(variance.velocityNorthMps2, observationNoiseMps2),
            up = gain(variance.velocityUpMps2, observationNoiseMps2)
        )
        return VelocityUpdate(
            velocity = EnuVelocity(
                eastMps = predictedVelocity.eastMps + gain.east * innovation.eastMps,
                northMps = predictedVelocity.northMps + gain.north * innovation.northMps,
                upMps = predictedVelocity.upMps + gain.up * innovation.upMps
            ),
            variance = variance.copy(
                velocityEastMps2 = reducedVariance(variance.velocityEastMps2, gain.east),
                velocityNorthMps2 = reducedVariance(variance.velocityNorthMps2, gain.north),
                velocityUpMps2 = reducedVariance(variance.velocityUpMps2, gain.up)
            ),
            innovationMps = innovationMps,
            accepted = true,
            gateMps = velocityGateMps,
            gain = gain
        )
    }

    private fun gain(predictedVariance: Double, observationVariance: Double): Double =
        predictedVariance / (predictedVariance + observationVariance)

    private fun reducedVariance(predictedVariance: Double, gain: Double): Double =
        max(parameters.minimumVariance, (1.0 - gain) * predictedVariance)

    private fun rejectionReason(
        positionAccepted: Boolean,
        velocityAvailable: Boolean,
        velocityAccepted: Boolean
    ): NavigationMeasurementRejectionReason? = when {
        !positionAccepted && velocityAvailable && !velocityAccepted ->
            NavigationMeasurementRejectionReason.POSITION_AND_VELOCITY_INNOVATION
        !positionAccepted -> NavigationMeasurementRejectionReason.POSITION_INNOVATION
        velocityAvailable && !velocityAccepted ->
            NavigationMeasurementRejectionReason.VELOCITY_INNOVATION
        else -> null
    }

    private data class PositionUpdate(
        val position: EnuPosition,
        val variance: NavigationAxisVariance,
        val gain: AxisGain?
    )

    private data class VelocityUpdate(
        val velocity: EnuVelocity,
        val variance: NavigationAxisVariance,
        val innovationMps: Double?,
        val accepted: Boolean,
        val gateMps: Double?,
        val gain: AxisGain? = null
    )
}

data class NavigationAxisVariance(
    val positionEastM2: Double,
    val positionNorthM2: Double,
    val positionUpM2: Double,
    val velocityEastMps2: Double,
    val velocityNorthMps2: Double,
    val velocityUpMps2: Double
) {
    fun predict(
        dtSec: Double,
        parameters: NavigationMeasurementParameters
    ): NavigationAxisVariance = copy(
        positionEastM2 = positionEastM2 + velocityEastMps2 * dtSec * dtSec +
            parameters.positionProcessNoiseM2PerSec * dtSec,
        positionNorthM2 = positionNorthM2 + velocityNorthMps2 * dtSec * dtSec +
            parameters.positionProcessNoiseM2PerSec * dtSec,
        positionUpM2 = positionUpM2 + velocityUpMps2 * dtSec * dtSec +
            parameters.verticalPositionProcessNoiseM2PerSec * dtSec,
        velocityEastMps2 = velocityEastMps2 + parameters.velocityProcessNoiseMps2PerSec * dtSec,
        velocityNorthMps2 = velocityNorthMps2 + parameters.velocityProcessNoiseMps2PerSec * dtSec,
        velocityUpMps2 = velocityUpMps2 + parameters.verticalVelocityProcessNoiseMps2PerSec * dtSec
    )

    companion object {
        val DEFAULT = NavigationAxisVariance(
            positionEastM2 = 100.0,
            positionNorthM2 = 100.0,
            positionUpM2 = 225.0,
            velocityEastMps2 = 25.0,
            velocityNorthMps2 = 25.0,
            velocityUpMps2 = 25.0
        )
    }
}

data class NavigationMeasurementUpdateResult(
    val state: NavigationState,
    val variance: NavigationAxisVariance,
    val positionAccepted: Boolean,
    val velocityAccepted: Boolean,
    val horizontalPositionInnovationM: Double,
    val velocityInnovationMps: Double?,
    val positionGain: AxisGain?,
    val velocityGain: AxisGain?,
    val horizontalPositionGateM: Double,
    val verticalPositionGateM: Double,
    val velocityGateMps: Double?,
    val updateMode: NavigationPositionUpdateMode,
    val rejectionReason: NavigationMeasurementRejectionReason?
)

data class AxisGain(
    val east: Double,
    val north: Double,
    val up: Double
)

enum class NavigationPositionUpdateMode {
    NORMAL_UPDATE,
    REJECTED,
    REACQUIRED
}

enum class NavigationMeasurementRejectionReason {
    POSITION_INNOVATION,
    VELOCITY_INNOVATION,
    POSITION_AND_VELOCITY_INNOVATION
}

data class NavigationMeasurementParameters(
    val innovationSigmaGate: Double,
    val minimumHorizontalPositionGateM: Double,
    val minimumVerticalPositionGateM: Double,
    val minimumVelocityGateMps: Double,
    val minimumSpeedStdMps: Double,
    val minimumVariance: Double,
    val reacquisitionRejectionCount: Int,
    val reacquisitionMaximumHorizontalAccuracyM: Double,
    val reacquisitionMinimumPositionAllowanceM: Double,
    val reacquisitionPositionAllowanceM: Double,
    val positionProcessNoiseM2PerSec: Double,
    val verticalPositionProcessNoiseM2PerSec: Double,
    val velocityProcessNoiseMps2PerSec: Double,
    val verticalVelocityProcessNoiseMps2PerSec: Double
) {
    companion object {
        val DEFAULT = NavigationMeasurementParameters(
            innovationSigmaGate = 6.0,
            minimumHorizontalPositionGateM = 100.0,
            minimumVerticalPositionGateM = 100.0,
            minimumVelocityGateMps = 50.0,
            minimumSpeedStdMps = 1.0,
            minimumVariance = 1e-6,
            reacquisitionRejectionCount = 5,
            reacquisitionMaximumHorizontalAccuracyM = 10.0,
            reacquisitionMinimumPositionAllowanceM = 30.0,
            reacquisitionPositionAllowanceM = 20.0,
            positionProcessNoiseM2PerSec = 4.0,
            verticalPositionProcessNoiseM2PerSec = 16.0,
            velocityProcessNoiseMps2PerSec = 2.25,
            verticalVelocityProcessNoiseMps2PerSec = 9.0
        )
    }
}
