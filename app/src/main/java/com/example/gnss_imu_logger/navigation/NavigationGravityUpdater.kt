package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.sqrt

/**
 * 低動作時の加速度方向を使ってロール・ピッチを補正し、補正方向の局所効果を診断する。
 * 入力: 車体からENUへの姿勢、車体加速度[m/s^2]、車体角速度[rad/s]、時間差[s]
 * 出力: 現行方向を適用した姿勢と、現行方向・逆方向の補正後重力残差
 *
 * applyPreferredGravityDirection=falseでは従来の補正方向を維持する。
 * 診断結果だけで本系の姿勢を変更しない。
 */
class NavigationGravityUpdater(
    private val parameters: NavigationGravityParameters = NavigationGravityParameters.DEFAULT,
    private val applyPreferredGravityDirection: Boolean = false
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
        if (!accelerationNormMps2.isFinite() || normErrorMps2 > parameters.maximumGravityNormErrorMps2) {
            return rejected(attitudeBodyToEnu, accelerationNormMps2, angularRateNormRadps, null,
                NavigationGravityRejectionReason.ACCELERATION_NORM)
        }
        if (!angularRateNormRadps.isFinite() || angularRateNormRadps > parameters.maximumAngularRateRadps) {
            return rejected(attitudeBodyToEnu, accelerationNormMps2, angularRateNormRadps, null,
                NavigationGravityRejectionReason.ANGULAR_RATE)
        }
        val measuredUpBody = accelerationBodyMps2.normalizedOrNull()
            ?: return unavailable(attitudeBodyToEnu)
        val residualBeforeRad = gravityResidualRad(attitudeBodyToEnu, measuredUpBody)
            ?: return unavailable(attitudeBodyToEnu)
        if (residualBeforeRad > parameters.maximumResidualRad) {
            return rejected(attitudeBodyToEnu, accelerationNormMps2, angularRateNormRadps,
                residualBeforeRad, NavigationGravityRejectionReason.RESIDUAL_GATE)
        }
        val predictedUpBody = predictedUpBody(attitudeBodyToEnu) ?: return unavailable(attitudeBodyToEnu)
        val correctionAxisBody = predictedUpBody.cross(measuredUpBody)
        val axisNorm = correctionAxisBody.norm()
        if (axisNorm < MINIMUM_VECTOR_NORM) {
            return acceptedWithoutCorrection(attitudeBodyToEnu, accelerationNormMps2,
                angularRateNormRadps, residualBeforeRad)
        }
        val correctionRad = minOf(
            residualBeforeRad * parameters.correctionGain,
            parameters.maximumCorrectionRateRadps * dtSec
        )
        val currentCandidate = correctedAttitude(
            attitudeBodyToEnu, correctionAxisBody, axisNorm, correctionRad, dtSec, 1.0
        )
        val oppositeCandidate = correctedAttitude(
            attitudeBodyToEnu, correctionAxisBody, axisNorm, correctionRad, dtSec, -1.0
        )
        val currentResidualAfterRad = gravityResidualRad(currentCandidate, measuredUpBody)
        val oppositeResidualAfterRad = gravityResidualRad(oppositeCandidate, measuredUpBody)
        val preferredDirection = preferredDirection(currentResidualAfterRad, oppositeResidualAfterRad)

        // 初回は診断専用。falseでは従来と同じcurrentCandidateを必ず本系へ適用する。
        val appliedAttitude = if (applyPreferredGravityDirection &&
            preferredDirection == NavigationGravityCorrectionDirection.OPPOSITE
        ) oppositeCandidate else currentCandidate
        val residualAfterRad = gravityResidualRad(appliedAttitude, measuredUpBody)
        val reductionRad = residualAfterRad?.let { residualBeforeRad - abs(it) }
        return NavigationGravityUpdateResult(
            attitudeBodyToEnu = appliedAttitude,
            available = true,
            accepted = true,
            accelerationNormMps2 = accelerationNormMps2,
            angularRateNormRadps = angularRateNormRadps,
            residualRad = residualBeforeRad,
            correctionRad = correctionRad,
            vehicleGradeBeforeRad = vehicleGrade(attitudeBodyToEnu),
            vehicleGradeAfterRad = vehicleGrade(appliedAttitude),
            gravityResidualBeforeRad = residualBeforeRad,
            gravityResidualAfterRad = residualAfterRad,
            residualReductionRad = reductionRad,
            correctionImprovedResidual = reductionRad?.let { it > 0.0 },
            currentDirectionResidualAfterRad = currentResidualAfterRad,
            oppositeDirectionResidualAfterRad = oppositeResidualAfterRad,
            preferredCorrectionDirection = preferredDirection,
            rejectionReason = NavigationGravityRejectionReason.ACCEPTED
        )
    }

    private fun correctedAttitude(attitude: AttitudeQuaternion, axis: BodyVector, axisNorm: Double,
        correctionRad: Double, dtSec: Double, direction: Double): AttitudeQuaternion {
        val rate = axis * (direction * correctionRad / axisNorm / dtSec)
        val corrected = (attitude * AttitudeQuaternion.fromAngularVelocity(rate, dtSec)).normalized()
        val beforeYaw = attitude.toEulerAngles().yawRad
        val euler = corrected.toEulerAngles()
        return AttitudeQuaternion.fromEulerAngles(euler.rollRad, euler.pitchRad, beforeYaw)
    }

    private fun gravityResidualRad(attitude: AttitudeQuaternion, measuredUpBody: BodyVector): Double? {
        val predicted = predictedUpBody(attitude) ?: return null
        return acos(predicted.dot(measuredUpBody).coerceIn(-1.0, 1.0))
    }

    private fun predictedUpBody(attitude: AttitudeQuaternion): BodyVector? =
        attitude.conjugate().rotate(WORLD_UP).normalizedOrNull()

    private fun vehicleGrade(attitude: AttitudeQuaternion): Double =
        -attitude.toEulerAngles().pitchRad

    private fun preferredDirection(current: Double?, opposite: Double?): NavigationGravityCorrectionDirection {
        if (current == null || opposite == null) return NavigationGravityCorrectionDirection.INCONCLUSIVE
        val difference = abs(current) - abs(opposite)
        return when {
            difference < -DIRECTION_DIFFERENCE_EPSILON_RAD -> NavigationGravityCorrectionDirection.CURRENT
            difference > DIRECTION_DIFFERENCE_EPSILON_RAD -> NavigationGravityCorrectionDirection.OPPOSITE
            else -> NavigationGravityCorrectionDirection.INCONCLUSIVE
        }
    }

    private fun acceptedWithoutCorrection(attitude: AttitudeQuaternion, accelerationNorm: Double,
        angularRateNorm: Double, residual: Double) = NavigationGravityUpdateResult(
        attitude, true, true, accelerationNorm, angularRateNorm, residual, 0.0,
        vehicleGrade(attitude), vehicleGrade(attitude), residual, residual, 0.0, false,
        residual, residual, NavigationGravityCorrectionDirection.INCONCLUSIVE,
        NavigationGravityRejectionReason.ACCEPTED
    )

    private fun unavailable(attitude: AttitudeQuaternion) = NavigationGravityUpdateResult(
        attitude, false, false, null, null, null, null,
        vehicleGrade(attitude), vehicleGrade(attitude), null, null, null, null,
        null, null, NavigationGravityCorrectionDirection.INCONCLUSIVE,
        NavigationGravityRejectionReason.OBSERVATION_UNAVAILABLE
    )

    private fun rejected(attitude: AttitudeQuaternion, accelerationNorm: Double?, angularRateNorm: Double?,
        residual: Double?, reason: NavigationGravityRejectionReason) = NavigationGravityUpdateResult(
        attitude, true, false, accelerationNorm, angularRateNorm, residual, null,
        vehicleGrade(attitude), vehicleGrade(attitude), residual, residual, null, null,
        null, null, NavigationGravityCorrectionDirection.INCONCLUSIVE, reason
    )

    private fun BodyVector.norm() = sqrt(x * x + y * y + z * z)
    private fun BodyVector.normalizedOrNull(): BodyVector? {
        val value = norm()
        return if (!value.isFinite() || value < MINIMUM_VECTOR_NORM) null
        else BodyVector(x / value, y / value, z / value)
    }
    private fun BodyVector.dot(other: BodyVector) = x * other.x + y * other.y + z * other.z
    private fun BodyVector.cross(other: BodyVector) = BodyVector(
        y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x
    )
    private operator fun BodyVector.times(scale: Double) = BodyVector(x * scale, y * scale, z * scale)

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val MINIMUM_VECTOR_NORM = 1e-9
        private const val DIRECTION_DIFFERENCE_EPSILON_RAD = 1e-12
        private val WORLD_UP = BodyVector(0.0, 0.0, 1.0)
    }
}

enum class NavigationGravityCorrectionDirection { CURRENT, OPPOSITE, INCONCLUSIVE }
enum class NavigationGravityRejectionReason {
    ACCELERATION_NORM, ANGULAR_RATE, RESIDUAL_GATE, OBSERVATION_UNAVAILABLE, ACCEPTED
}

data class NavigationGravityUpdateResult(
    val attitudeBodyToEnu: AttitudeQuaternion,
    val available: Boolean,
    val accepted: Boolean,
    val accelerationNormMps2: Double?,
    val angularRateNormRadps: Double?,
    val residualRad: Double?,
    val correctionRad: Double?,
    val vehicleGradeBeforeRad: Double,
    val vehicleGradeAfterRad: Double,
    val gravityResidualBeforeRad: Double?,
    val gravityResidualAfterRad: Double?,
    val residualReductionRad: Double?,
    val correctionImprovedResidual: Boolean?,
    val currentDirectionResidualAfterRad: Double?,
    val oppositeDirectionResidualAfterRad: Double?,
    val preferredCorrectionDirection: NavigationGravityCorrectionDirection,
    val rejectionReason: NavigationGravityRejectionReason
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
