package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 前処理済みイベントへ純粋なIMU予測を適用する。
 * 入力: PreparedNavigationInput
 * 出力: 一定周期のNavigationPredictionSampleとGNSS時点の予測誤差
 */
class OfflineNavigationPredictor(
    private val outputIntervalNs: Long = DEFAULT_OUTPUT_INTERVAL_NS
) {
    fun run(input: PreparedNavigationInput): OfflineNavigationPredictionResult {
        require(input.events.isNotEmpty()) { "推定入力イベントがありません。" }
        val initialAcceleration = input.events.asSequence()
            .takeWhile { it.elapsedRealtimeNs <= input.initialElapsedRealtimeNs }
            .filterIsInstance<PreparedNavigationEvent.Imu>()
            .mapNotNull { it.sample.accelerationBodyMps2 }
            .lastOrNull()
        val yawRad = atan2(
            input.initialVelocityEnuMps.eastMps,
            input.initialVelocityEnuMps.northMps
        ).takeIf { input.initialVelocityEnuMps.horizontalSpeedMps() >= MINIMUM_INITIAL_YAW_SPEED_MPS }
            ?: 0.0
        val initialAttitude = initialAcceleration?.let { initialAttitude(it, yawRad) }
            ?: AttitudeQuaternion.fromEulerAngles(0.0, 0.0, yawRad)
        val initialTimestampNs = input.initialElapsedRealtimeNs
        val predictor = NavigationStatePredictor(
            NavigationState(
                elapsedRealtimeNs = initialTimestampNs,
                positionEnuM = input.initialPositionEnuM,
                velocityEnuMps = input.initialVelocityEnuMps,
                attitudeBodyToEnu = initialAttitude,
                accelerometerBiasMps2 = ZERO_BODY_VECTOR,
                gyroscopeBiasRadps = ZERO_BODY_VECTOR,
                accelerationEnuMps2 = ZERO_BODY_VECTOR,
                status = NavigationPredictionStatus.WAITING_FOR_IMU
            )
        )
        val samples = mutableListOf<NavigationPredictionSample>()
        val residuals = mutableListOf<NavigationGnssResidual>()
        var nextOutputNs = initialTimestampNs
        input.events.asSequence()
            .filter { it.elapsedRealtimeNs >= initialTimestampNs }
            .forEach { event ->
            when (event) {
                is PreparedNavigationEvent.Imu -> {
                    val state = predictor.update(event.sample)
                    if (state.elapsedRealtimeNs >= nextOutputNs) {
                        samples += state.toSample()
                        nextOutputNs = state.elapsedRealtimeNs + outputIntervalNs
                    }
                }
                is PreparedNavigationEvent.Gnss -> {
                    val state = predictor.currentState()
                    residuals += NavigationGnssResidual(
                        elapsedRealtimeNs = event.update.elapsedRealtimeNs,
                        eastErrorM = state.positionEnuM.eastM - event.update.positionEnuM.eastM,
                        northErrorM = state.positionEnuM.northM - event.update.positionEnuM.northM,
                        upErrorM = state.positionEnuM.upM - event.update.positionEnuM.upM,
                        horizontalErrorM = horizontalDistance(state.positionEnuM, event.update.positionEnuM),
                        velocityErrorMps = event.update.velocityEnuMps?.let {
                            velocityDistance(state.velocityEnuMps, it)
                        }
                    )
                }
            }
        }
        return OfflineNavigationPredictionResult(samples, residuals)
    }

    private fun initialAttitude(gravityBodyMps2: BodyVector, yawRad: Double): AttitudeQuaternion {
        val rollRad = atan2(gravityBodyMps2.y, gravityBodyMps2.z)
        val pitchRad = atan2(
            -gravityBodyMps2.x,
            sqrt(
                gravityBodyMps2.y * gravityBodyMps2.y +
                    gravityBodyMps2.z * gravityBodyMps2.z
            )
        )
        return AttitudeQuaternion.fromEulerAngles(rollRad, pitchRad, yawRad)
    }

    private fun NavigationState.toSample(): NavigationPredictionSample = NavigationPredictionSample(
        elapsedRealtimeNs = elapsedRealtimeNs,
        positionEnuM = positionEnuM,
        velocityEnuMps = velocityEnuMps,
        attitudeBodyToEnu = attitudeBodyToEnu,
        accelerationEnuMps2 = accelerationEnuMps2,
        status = status
    )

    private fun horizontalDistance(first: EnuPosition, second: EnuPosition): Double {
        val east = first.eastM - second.eastM
        val north = first.northM - second.northM
        return sqrt(east * east + north * north)
    }

    private fun velocityDistance(first: EnuVelocity, second: EnuVelocity): Double {
        val east = first.eastMps - second.eastMps
        val north = first.northMps - second.northMps
        val up = first.upMps - second.upMps
        return sqrt(east * east + north * north + up * up)
    }

    companion object {
        private const val DEFAULT_OUTPUT_INTERVAL_NS = 10_000_000L
        private const val MINIMUM_INITIAL_YAW_SPEED_MPS = 2.0
        private val ZERO_BODY_VECTOR = BodyVector(0.0, 0.0, 0.0)
    }
}

data class NavigationPredictionSample(
    val elapsedRealtimeNs: Long,
    val positionEnuM: EnuPosition,
    val velocityEnuMps: EnuVelocity,
    val attitudeBodyToEnu: AttitudeQuaternion,
    val accelerationEnuMps2: BodyVector,
    val status: NavigationPredictionStatus
)

data class NavigationGnssResidual(
    val elapsedRealtimeNs: Long,
    val eastErrorM: Double,
    val northErrorM: Double,
    val upErrorM: Double,
    val horizontalErrorM: Double,
    val velocityErrorMps: Double?
)

data class OfflineNavigationPredictionResult(
    val samples: List<NavigationPredictionSample>,
    val gnssResiduals: List<NavigationGnssResidual>
)

private fun EnuVelocity.horizontalSpeedMps(): Double =
    sqrt(eastMps * eastMps + northMps * northMps)
