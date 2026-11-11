package com.example.gnss_imu_logger.navigation

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/** 気圧高度と気圧鉛直速度でUp位置・速度だけを補正する。 */
class NavigationVerticalUpdater(
    private val parameters: NavigationVerticalParameters = NavigationVerticalParameters.DEFAULT
) {
    fun update(
        state: NavigationState,
        variance: NavigationAxisVariance,
        observation: NavigationVerticalObservation
    ): NavigationVerticalUpdateResult {
        val positionInnovation = observation.relativeUpM - state.positionEnuM.upM
        val speedInnovation = observation.verticalSpeedMps - state.velocityEnuMps.upMps
        val positionGate = max(
            parameters.minimumPositionGateM,
            parameters.innovationSigmaGate * sqrt(
                variance.positionUpM2 + observation.positionStdM * observation.positionStdM
            )
        )
        val speedGate = max(
            parameters.minimumSpeedGateMps,
            parameters.innovationSigmaGate * sqrt(
                variance.velocityUpMps2 + observation.speedStdMps * observation.speedStdMps
            )
        )
        val positionAccepted = observation.valid && abs(positionInnovation) <= positionGate
        val speedAccepted = observation.valid && abs(speedInnovation) <= speedGate
        val positionGain = if (positionAccepted) gain(
            variance.positionUpM2,
            observation.positionStdM * observation.positionStdM
        ) else 0.0
        val speedGain = if (speedAccepted) gain(
            variance.velocityUpMps2,
            observation.speedStdMps * observation.speedStdMps
        ) else 0.0
        val updatedState = state.copy(
            positionEnuM = state.positionEnuM.copy(
                upM = state.positionEnuM.upM + positionGain * positionInnovation
            ),
            velocityEnuMps = state.velocityEnuMps.copy(
                upMps = state.velocityEnuMps.upMps + speedGain * speedInnovation
            )
        )
        val updatedVariance = variance.copy(
            positionUpM2 = if (positionAccepted) {
                reducedVariance(variance.positionUpM2, positionGain)
            } else variance.positionUpM2,
            velocityUpMps2 = if (speedAccepted) {
                reducedVariance(variance.velocityUpMps2, speedGain)
            } else variance.velocityUpMps2
        )
        return NavigationVerticalUpdateResult(
            state = updatedState,
            variance = updatedVariance,
            positionInnovationM = positionInnovation,
            speedInnovationMps = speedInnovation,
            positionGateM = positionGate,
            speedGateMps = speedGate,
            positionGain = positionGain,
            speedGain = speedGain,
            positionAccepted = positionAccepted,
            speedAccepted = speedAccepted,
            rejectionReason = when {
                !observation.valid -> observation.rejectionReason
                !positionAccepted && !speedAccepted -> "POSITION_AND_SPEED_INNOVATION"
                !positionAccepted -> "POSITION_INNOVATION"
                !speedAccepted -> "SPEED_INNOVATION"
                else -> ""
            }
        )
    }

    private fun gain(predictedVariance: Double, observationVariance: Double): Double =
        predictedVariance / (predictedVariance + observationVariance)

    private fun reducedVariance(predictedVariance: Double, gain: Double): Double =
        max(parameters.minimumVariance, (1.0 - gain) * predictedVariance)
}

data class NavigationVerticalParameters(
    val minimumPositionGateM: Double,
    val minimumSpeedGateMps: Double,
    val innovationSigmaGate: Double,
    val minimumVariance: Double
) {
    companion object {
        val DEFAULT = NavigationVerticalParameters(
            minimumPositionGateM = 15.0,
            minimumSpeedGateMps = 3.0,
            innovationSigmaGate = 5.0,
            minimumVariance = 1e-6
        )
    }
}

data class NavigationVerticalObservation(
    val elapsedRealtimeNs: Long,
    val relativeUpM: Double,
    val verticalSpeedMps: Double,
    val positionStdM: Double,
    val speedStdMps: Double,
    val altitudeSource: NavigationAltitudeSource,
    val valid: Boolean,
    val rejectionReason: String
)

enum class NavigationAltitudeSource {
    BAROMETER_TERRAIN_ALIGNED,
    BAROMETER_RELATIVE
}

data class NavigationVerticalUpdateResult(
    val state: NavigationState,
    val variance: NavigationAxisVariance,
    val positionInnovationM: Double,
    val speedInnovationMps: Double,
    val positionGateM: Double,
    val speedGateMps: Double,
    val positionGain: Double,
    val speedGain: Double,
    val positionAccepted: Boolean,
    val speedAccepted: Boolean,
    val rejectionReason: String
)
