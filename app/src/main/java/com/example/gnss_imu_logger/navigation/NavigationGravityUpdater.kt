package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.sqrt

/**
 * 低動作時の加速度方向を使ってロール・ピッチを補正する。
 * 入力: 車体からENUへの姿勢、車体加速度[m/s^2]、車体角速度[rad/s]、時間差[s]
 * 出力: ヨー角を変えずに重力方向へ補正した姿勢と診断値
 *
 * 走行中の加減速や旋回を重力と誤認しないよう、加速度ノルム、角速度、
 * 重力方向残差の全条件を満たす場合だけ小さく補正する。
 */
class NavigationGravityUpdater(
    private val parameters: NavigationGravityParameters = NavigationGravityParameters.DEFAULT
) {
    fun update(
        attitudeBodyToEnu: AttitudeQuaternion,
        accelerationBodyMps2: BodyVector?,
        angularRateBodyRadps: BodyVector?,
        dtSec: Double
    ): NavigationGravityUpdateResult {
        if (accelerationBodyMps2 == null || angularRateBodyRadps == null || dtSec <= 0.0) {
            return unavailable(attitudeBodyToEnu)
        }
        val accelerationNormMps2 = accelerationBodyMps2.norm()
        val normErrorMps2 = abs(accelerationNormMps2 - STANDARD_GRAVITY_MPS2)
        val angularRateNormRadps = angularRateBodyRadps.norm()
        if (
            !accelerationNormMps2.isFinite() ||
            normErrorMps2 > parameters.maximumGravityNormErrorMps2 ||
            angularRateNormRadps > parameters.maximumAngularRateRadps
        ) {
            return rejected(
                attitudeBodyToEnu,
                accelerationNormMps2,
                angularRateNormRadps,
                null
            )
        }
        val measuredUpBody = accelerationBodyMps2.normalizedOrNull()
            ?: return unavailable(attitudeBodyToEnu)
        val predictedUpBody = attitudeBodyToEnu.conjugate()
            .rotate(WORLD_UP)
            .normalizedOrNull()
            ?: return unavailable(attitudeBodyToEnu)
        val residualRad = acos(
            predictedUpBody.dot(measuredUpBody).coerceIn(-1.0, 1.0)
        )
        if (residualRad > parameters.maximumResidualRad) {
            return rejected(
                attitudeBodyToEnu,
                accelerationNormMps2,
                angularRateNormRadps,
                residualRad
            )
        }
        val correctionAxisBody = predictedUpBody.cross(measuredUpBody)
        val axisNorm = correctionAxisBody.norm()
        if (axisNorm < MINIMUM_VECTOR_NORM) {
            return NavigationGravityUpdateResult(
                attitudeBodyToEnu = attitudeBodyToEnu,
                available = true,
                accepted = true,
                accelerationNormMps2 = accelerationNormMps2,
                angularRateNormRadps = angularRateNormRadps,
                residualRad = residualRad,
                correctionRad = 0.0
            )
        }
        val correctionRad = minOf(
            residualRad * parameters.correctionGain,
            parameters.maximumCorrectionRateRadps * dtSec
        )
        val correctionRateBody = correctionAxisBody *
            (correctionRad / axisNorm / dtSec)
        val correctedAttitude = (
            attitudeBodyToEnu *
                AttitudeQuaternion.fromAngularVelocity(correctionRateBody, dtSec)
            ).normalized()
        return NavigationGravityUpdateResult(
            attitudeBodyToEnu = correctedAttitude,
            available = true,
            accepted = true,
            accelerationNormMps2 = accelerationNormMps2,
            angularRateNormRadps = angularRateNormRadps,
            residualRad = residualRad,
            correctionRad = correctionRad
        )
    }

    private fun unavailable(attitude: AttitudeQuaternion) = NavigationGravityUpdateResult(
        attitudeBodyToEnu = attitude,
        available = false,
        accepted = false,
        accelerationNormMps2 = null,
        angularRateNormRadps = null,
        residualRad = null,
        correctionRad = null
    )

    private fun rejected(
        attitude: AttitudeQuaternion,
        accelerationNormMps2: Double,
        angularRateNormRadps: Double,
        residualRad: Double?
    ) = NavigationGravityUpdateResult(
        attitudeBodyToEnu = attitude,
        available = true,
        accepted = false,
        accelerationNormMps2 = accelerationNormMps2,
        angularRateNormRadps = angularRateNormRadps,
        residualRad = residualRad,
        correctionRad = null
    )

    private fun BodyVector.norm(): Double = sqrt(x * x + y * y + z * z)

    private fun BodyVector.normalizedOrNull(): BodyVector? {
        val norm = norm()
        if (!norm.isFinite() || norm < MINIMUM_VECTOR_NORM) return null
        return BodyVector(x / norm, y / norm, z / norm)
    }

    private fun BodyVector.dot(other: BodyVector): Double =
        x * other.x + y * other.y + z * other.z

    private fun BodyVector.cross(other: BodyVector): BodyVector = BodyVector(
        x = y * other.z - z * other.y,
        y = z * other.x - x * other.z,
        z = x * other.y - y * other.x
    )

    private operator fun BodyVector.times(scale: Double) = BodyVector(
        x = x * scale,
        y = y * scale,
        z = z * scale
    )

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val MINIMUM_VECTOR_NORM = 1e-9
        private val WORLD_UP = BodyVector(0.0, 0.0, 1.0)
    }
}

data class NavigationGravityUpdateResult(
    val attitudeBodyToEnu: AttitudeQuaternion,
    val available: Boolean,
    val accepted: Boolean,
    val accelerationNormMps2: Double?,
    val angularRateNormRadps: Double?,
    val residualRad: Double?,
    val correctionRad: Double?
)

data class NavigationGravityParameters(
    val maximumGravityNormErrorMps2: Double,
    val maximumAngularRateRadps: Double,
    val maximumResidualRad: Double,
    val correctionGain: Double,
    val maximumCorrectionRateRadps: Double
) {
    companion object {
        val DEFAULT = NavigationGravityParameters(
            maximumGravityNormErrorMps2 = 0.8,
            maximumAngularRateRadps = Math.toRadians(20.0),
            maximumResidualRad = Math.toRadians(15.0),
            correctionGain = 0.02,
            maximumCorrectionRateRadps = Math.toRadians(2.0)
        )
    }
}
