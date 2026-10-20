package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector

/**
 * IMU入力からENU位置・速度・姿勢を予測する。
 * 入力: 車体座標加速度[m/s^2]、車体座標角速度[rad/s]、時刻[ns]
 * 出力: ENU位置[m]、ENU速度[m/s]、車体座標からENU座標へのQuaternion
 */
class NavigationStatePredictor(
    initialState: NavigationState,
    private val parameters: NavigationPredictionParameters = NavigationPredictionParameters.DEFAULT
) {
    private var state = initialState
    private var accelerationBodyMps2: BodyVector? = null
    private var angularRateBodyRadps: BodyVector? = null

    /**
     * 新しいIMU測定値まで状態を進める。
     * 入力: NavigationImuSample
     * 出力: 更新後のNavigationState。積分できない場合は現在状態
     */
    fun update(sample: NavigationImuSample): NavigationState {
        require(sample.elapsedRealtimeNs >= state.elapsedRealtimeNs) {
            "IMU時刻が現在の状態時刻より前です。"
        }
        val dtNs = sample.elapsedRealtimeNs - state.elapsedRealtimeNs
        if (dtNs > 0L && dtNs <= parameters.maximumIntegrationIntervalNs) {
            predict(sample.elapsedRealtimeNs, dtNs / NS_PER_SEC)
        } else if (dtNs > parameters.maximumIntegrationIntervalNs) {
            state = state.copy(
                elapsedRealtimeNs = sample.elapsedRealtimeNs,
                status = NavigationPredictionStatus.IMU_GAP
            )
        }
        sample.accelerationBodyMps2?.let { accelerationBodyMps2 = it }
        sample.angularRateBodyRadps?.let { angularRateBodyRadps = it }
        return state
    }

    fun currentState(): NavigationState = state

    private fun predict(timestampNs: Long, dtSec: Double) {
        val previous = state
        val rate = angularRateBodyRadps?.minus(previous.gyroscopeBiasRadps)
            ?: ZERO_BODY_VECTOR
        val nextAttitude = (
            previous.attitudeBodyToEnu * AttitudeQuaternion.fromAngularVelocity(rate, dtSec)
            ).normalized()
        val specificForceBody = accelerationBodyMps2?.minus(previous.accelerometerBiasMps2)
        val accelerationEnu = specificForceBody?.let {
            nextAttitude.rotate(it) - GRAVITY_ENU_MPS2
        } ?: ZERO_BODY_VECTOR
        val nextVelocity = EnuVelocity(
            eastMps = previous.velocityEnuMps.eastMps + accelerationEnu.x * dtSec,
            northMps = previous.velocityEnuMps.northMps + accelerationEnu.y * dtSec,
            upMps = previous.velocityEnuMps.upMps + accelerationEnu.z * dtSec
        )
        val nextPosition = EnuPosition(
            eastM = previous.positionEnuM.eastM +
                0.5 * (previous.velocityEnuMps.eastMps + nextVelocity.eastMps) * dtSec,
            northM = previous.positionEnuM.northM +
                0.5 * (previous.velocityEnuMps.northMps + nextVelocity.northMps) * dtSec,
            upM = previous.positionEnuM.upM +
                0.5 * (previous.velocityEnuMps.upMps + nextVelocity.upMps) * dtSec
        )
        state = previous.copy(
            elapsedRealtimeNs = timestampNs,
            positionEnuM = nextPosition,
            velocityEnuMps = nextVelocity,
            attitudeBodyToEnu = nextAttitude,
            accelerationEnuMps2 = accelerationEnu,
            status = if (specificForceBody == null || angularRateBodyRadps == null) {
                NavigationPredictionStatus.WAITING_FOR_IMU
            } else {
                NavigationPredictionStatus.PREDICTING
            }
        )
    }

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
        private val GRAVITY_ENU_MPS2 = BodyVector(0.0, 0.0, 9.80665)
    }
}

/** ナビゲーション予測状態。 */
data class NavigationState(
    val elapsedRealtimeNs: Long,
    val positionEnuM: EnuPosition,
    val velocityEnuMps: EnuVelocity,
    val attitudeBodyToEnu: AttitudeQuaternion,
    val accelerometerBiasMps2: BodyVector,
    val gyroscopeBiasRadps: BodyVector,
    val accelerationEnuMps2: BodyVector,
    val status: NavigationPredictionStatus
)

enum class NavigationPredictionStatus {
    WAITING_FOR_IMU,
    PREDICTING,
    IMU_GAP
}

data class NavigationPredictionParameters(
    val maximumIntegrationIntervalNs: Long
) {
    companion object {
        val DEFAULT = NavigationPredictionParameters(
            maximumIntegrationIntervalNs = 100_000_000L
        )
    }
}

private operator fun BodyVector.minus(other: BodyVector) = BodyVector(
    x = x - other.x,
    y = y - other.y,
    z = z - other.z
)

private val ZERO_BODY_VECTOR = BodyVector(0.0, 0.0, 0.0)
