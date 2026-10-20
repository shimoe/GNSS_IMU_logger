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
    val status: NavigationPredictionStatus,
    val uncertainty: NavigationPredictionUncertainty? = null
)

/**
 * ナビゲーション推定値の1 sigma標準偏差。
 * 入力: 位置・速度の軸別分散[m^2, (m/s)^2]、方位分散[rad^2]
 * 出力: 位置[m]、速度[m/s]、方位[rad]の標準偏差
 *
 * 現段階は15状態ESKF移行前の軸別分散を出力する。
 * 15状態共分散へ置き換えた後もCSVの列定義を維持する。
 */
data class NavigationPredictionUncertainty(
    val positionEastStdM: Double,
    val positionNorthStdM: Double,
    val positionUpStdM: Double,
    val horizontalPositionStdM: Double,
    val velocityEastStdMps: Double,
    val velocityNorthStdMps: Double,
    val velocityUpStdMps: Double,
    val headingStdRad: Double
) {
    companion object {
        fun from(
            variance: NavigationAxisVariance,
            headingVarianceRad2: Double
        ) = NavigationPredictionUncertainty(
            positionEastStdM = kotlin.math.sqrt(variance.positionEastM2.coerceAtLeast(0.0)),
            positionNorthStdM = kotlin.math.sqrt(variance.positionNorthM2.coerceAtLeast(0.0)),
            positionUpStdM = kotlin.math.sqrt(variance.positionUpM2.coerceAtLeast(0.0)),
            horizontalPositionStdM = kotlin.math.sqrt(
                (variance.positionEastM2 + variance.positionNorthM2).coerceAtLeast(0.0)
            ),
            velocityEastStdMps = kotlin.math.sqrt(variance.velocityEastMps2.coerceAtLeast(0.0)),
            velocityNorthStdMps = kotlin.math.sqrt(variance.velocityNorthMps2.coerceAtLeast(0.0)),
            velocityUpStdMps = kotlin.math.sqrt(variance.velocityUpMps2.coerceAtLeast(0.0)),
            headingStdRad = kotlin.math.sqrt(headingVarianceRad2.coerceAtLeast(0.0))
        )
    }
}

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
