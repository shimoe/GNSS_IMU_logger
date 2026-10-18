package com.example.gnss_imu_logger.attitude

import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 6軸IMUから車体姿勢を推定する。
 * 入力: 端末座標の加速度[m/s^2]、バイアス補正後角速度[rad/s]、elapsedRealtimeNs[ns]、静止判定
 * 出力: Quaternion、ロール・ピッチ・ヨー、角速度、重力方向残差、補正状態、信頼度
 */
class VehicleAttitudeEstimator(
    initialGravityDeviceMps2: BodyVector,
    private val mountTransform: DeviceMountTransform
) {
    private var attitude = initialAttitude(
        mountTransform.deviceToBody(initialGravityDeviceMps2)
    )
    private var latestAcceleration: BodyVector? = null
    private var accelerationChangeMps3 = 0.0
    private var lastAccelerationNs: Long? = null
    private var lastGyroscopeNs: Long? = null
    private var stationary = false

    /**
     * 加速度を車体座標へ変換し、加速度変化量を更新する。
     * 入力: elapsedRealtimeNs[ns]、端末座標加速度[m/s^2]
     * 出力: なし
     */
    @Synchronized
    fun updateAcceleration(timestampNs: Long, deviceValues: FloatArray) {
        val acceleration = mountTransform.deviceToBody(deviceValues)
        val previous = latestAcceleration
        val previousNs = lastAccelerationNs
        latestAcceleration = acceleration
        lastAccelerationNs = timestampNs

        if (previous == null || previousNs == null || timestampNs <= previousNs) return
        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec !in MINIMUM_DT_SEC..MAXIMUM_ACCELERATION_DT_SEC) return

        val currentChangeMps3 = (acceleration - previous).norm() / dtSec
        accelerationChangeMps3 += ACCELERATION_CHANGE_FILTER_ALPHA *
            (currentChangeMps3 - accelerationChangeMps3)
    }

    /**
     * 静止判定を重力補正の適用条件へ反映する。
     * 入力: 静止判定
     * 出力: なし
     */
    @Synchronized
    fun updateStationary(stationary: Boolean) {
        this.stationary = stationary
    }

    @Synchronized
    fun updateGyroscope(
        timestampNs: Long,
        correctedDeviceValuesRadps: FloatArray
    ): VehicleAttitudeSnapshot? {
        val bodyRate = mountTransform.deviceToBody(correctedDeviceValuesRadps)
        val previousNs = lastGyroscopeNs
        lastGyroscopeNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) return null

        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec !in MINIMUM_DT_SEC..MAXIMUM_DT_SEC) return null

        // 角速度は車体座標で受信しているため、Quaternionの右側へ微小回転を合成する。
        attitude = (attitude * AttitudeQuaternion.fromAngularVelocity(bodyRate, dtSec)).normalized()

        val acceleration = latestAcceleration
        val normConfidence = acceleration?.let(::accelerationNormConfidence) ?: 0.0
        val residualBeforeRad = acceleration?.let(::gravityResidualRad)
        val correction = gravityCorrectionDecision(
            bodyRate = bodyRate,
            normConfidence = normConfidence,
            residualRad = residualBeforeRad
        )
        if (acceleration != null && correction.weight > 0.0) {
            applyGravityCorrection(
                acceleration = acceleration,
                correctionWeight = correction.weight,
                correctionRateRadps = correction.maximumRateRadps,
                dtSec = dtSec
            )
        }

        val residualAfterRad = acceleration?.let(::gravityResidualRad)
        val directionConfidence = residualAfterRad?.let(::gravityDirectionConfidence) ?: 0.0
        val confidence = normConfidence * directionConfidence
        val euler = attitude.toEulerAngles()

        return VehicleAttitudeSnapshot(
            elapsedRealtimeNs = timestampNs,
            quaternion = attitude,
            rollRad = euler.rollRad,
            pitchRad = euler.pitchRad,
            yawRad = euler.yawRad,
            rollRateRadps = bodyRate.x,
            pitchRateRadps = bodyRate.y,
            yawRateRadps = bodyRate.z,
            gravityResidualRad = residualAfterRad ?: residualBeforeRad,
            accelerationNormConfidence = normConfidence,
            gravityDirectionConfidence = directionConfidence,
            gravityCorrectionWeight = correction.weight,
            gravityCorrectionMode = correction.mode,
            accelerationChangeMps3 = accelerationChangeMps3,
            attitudeValid = confidence >= VALID_CONFIDENCE,
            attitudeConfidence = confidence
        )
    }

    @Synchronized
    fun reset(initialGravityDeviceMps2: BodyVector) {
        attitude = initialAttitude(mountTransform.deviceToBody(initialGravityDeviceMps2))
        latestAcceleration = null
        accelerationChangeMps3 = 0.0
        lastAccelerationNs = null
        lastGyroscopeNs = null
        stationary = false
    }

    /** 強い並進加速度中は重力補正を弱める。 */
    private fun accelerationNormConfidence(acceleration: BodyVector): Double {
        val error = abs(acceleration.norm() - STANDARD_GRAVITY_MPS2)
        return (1.0 - error / MAXIMUM_GRAVITY_ERROR_MPS2).coerceIn(0.0, 1.0)
    }

    /** 推定重力と測定重力の角度残差から信頼度を求める。 */
    private fun gravityDirectionConfidence(residualRad: Double): Double =
        (1.0 - residualRad / MAXIMUM_VALID_GRAVITY_RESIDUAL_RAD).coerceIn(0.0, 1.0)

    /**
     * 静止中は重力方向へ再収束し、走行中は低動作かつ残差が小さい場合だけ補正する。
     * 入力: 車体角速度[rad/s]、加速度ノルム信頼度、重力方向残差[rad]
     * 出力: 補正状態、補正重み、最大補正速度[rad/s]
     */
    private fun gravityCorrectionDecision(
        bodyRate: BodyVector,
        normConfidence: Double,
        residualRad: Double?
    ): GravityCorrectionDecision {
        if (residualRad == null || normConfidence <= 0.0) {
            return GravityCorrectionDecision.propagationOnly()
        }
        if (stationary) {
            val mode = if (residualRad > RECOVERING_RESIDUAL_RAD) {
                GravityCorrectionMode.RECOVERING
            } else {
                GravityCorrectionMode.STATIONARY_GRAVITY
            }
            val rateRadps = if (mode == GravityCorrectionMode.RECOVERING) {
                RECOVERING_CORRECTION_RATE_RADPS
            } else {
                STATIONARY_CORRECTION_RATE_RADPS
            }
            return GravityCorrectionDecision(mode, normConfidence, rateRadps)
        }

        val angularRateConfidence = linearConfidence(
            value = bodyRate.norm(),
            fullConfidenceAt = LOW_DYNAMICS_FULL_ANGULAR_RATE_RADPS,
            zeroConfidenceAt = LOW_DYNAMICS_MAXIMUM_ANGULAR_RATE_RADPS
        )
        val accelerationChangeConfidence = linearConfidence(
            value = accelerationChangeMps3,
            fullConfidenceAt = LOW_DYNAMICS_FULL_ACCELERATION_CHANGE_MPS3,
            zeroConfidenceAt = LOW_DYNAMICS_MAXIMUM_ACCELERATION_CHANGE_MPS3
        )
        val residualConfidence = linearConfidence(
            value = residualRad,
            fullConfidenceAt = LOW_DYNAMICS_FULL_RESIDUAL_RAD,
            zeroConfidenceAt = LOW_DYNAMICS_MAXIMUM_RESIDUAL_RAD
        )
        val weight = normConfidence * angularRateConfidence *
            accelerationChangeConfidence * residualConfidence
        return if (weight >= MINIMUM_CORRECTION_WEIGHT) {
            GravityCorrectionDecision(
                mode = GravityCorrectionMode.LOW_DYNAMICS_GRAVITY,
                weight = weight,
                maximumRateRadps = LOW_DYNAMICS_CORRECTION_RATE_RADPS
            )
        } else {
            GravityCorrectionDecision.propagationOnly()
        }
    }

    private fun linearConfidence(
        value: Double,
        fullConfidenceAt: Double,
        zeroConfidenceAt: Double
    ): Double {
        if (!value.isFinite() || zeroConfidenceAt <= fullConfidenceAt) return 0.0
        return ((zeroConfidenceAt - value) /
            (zeroConfidenceAt - fullConfidenceAt)).coerceIn(0.0, 1.0)
    }

    /**
     * 重力方向誤差をQuaternionの微小回転として補正する。
     * Euler角を介さないため、ピッチ±90度付近でもロール・ヨーの結合を避ける。
     */
    private fun applyGravityCorrection(
        acceleration: BodyVector,
        correctionWeight: Double,
        correctionRateRadps: Double,
        dtSec: Double
    ) {
        val measuredUpBody = acceleration.normalizedOrNull() ?: return
        val predictedUpBody = attitude.conjugate()
            .rotate(WORLD_UP)
            .normalizedOrNull() ?: return

        // 現在姿勢へ右側から車体座標の補正回転を合成するため、
        // 測定上方向から予測上方向への外積を使用する。
        val correctionAxisBody = measuredUpBody.cross(predictedUpBody)
        val axisNorm = correctionAxisBody.norm()
        if (!axisNorm.isFinite() || axisNorm < MINIMUM_VECTOR_NORM) return

        val residualRad = acos(
            predictedUpBody.dot(measuredUpBody).coerceIn(-1.0, 1.0)
        )
        if (!residualRad.isFinite()) return

        val maximumCorrectionRad = correctionRateRadps * correctionWeight * dtSec
        val correctionRad = residualRad.coerceAtMost(maximumCorrectionRad)
        val correctionRate = correctionAxisBody * (correctionRad / axisNorm / dtSec)
        attitude = (
            attitude * AttitudeQuaternion.fromAngularVelocity(correctionRate, dtSec)
        ).normalized()
    }

    private fun gravityResidualRad(acceleration: BodyVector): Double? {
        val measuredUpBody = acceleration.normalizedOrNull() ?: return null
        val predictedUpBody = attitude.conjugate()
            .rotate(WORLD_UP)
            .normalizedOrNull() ?: return null
        return acos(predictedUpBody.dot(measuredUpBody).coerceIn(-1.0, 1.0))
    }

    private fun initialAttitude(gravityBodyMps2: BodyVector): AttitudeQuaternion {
        val rollRad = atan2(gravityBodyMps2.y, gravityBodyMps2.z)
        val pitchRad = atan2(
            -gravityBodyMps2.x,
            sqrt(
                gravityBodyMps2.y * gravityBodyMps2.y +
                    gravityBodyMps2.z * gravityBodyMps2.z
            )
        )
        return AttitudeQuaternion.fromEulerAngles(rollRad, pitchRad, 0.0)
    }

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

    private operator fun BodyVector.minus(other: BodyVector): BodyVector =
        BodyVector(x - other.x, y - other.y, z - other.z)

    private operator fun BodyVector.times(scale: Double): BodyVector =
        BodyVector(x * scale, y * scale, z * scale)

    private data class GravityCorrectionDecision(
        val mode: GravityCorrectionMode,
        val weight: Double,
        val maximumRateRadps: Double
    ) {
        companion object {
            fun propagationOnly() = GravityCorrectionDecision(
                mode = GravityCorrectionMode.PROPAGATION_ONLY,
                weight = 0.0,
                maximumRateRadps = 0.0
            )
        }
    }

    companion object {
        private val WORLD_UP = BodyVector(0.0, 0.0, 1.0)
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val MAXIMUM_GRAVITY_ERROR_MPS2 = 3.0
        private const val STATIONARY_CORRECTION_RATE_RADPS = 1.5
        private const val RECOVERING_CORRECTION_RATE_RADPS = 0.35
        private const val LOW_DYNAMICS_CORRECTION_RATE_RADPS = 0.30
        private val RECOVERING_RESIDUAL_RAD = Math.toRadians(5.0)
        private val LOW_DYNAMICS_FULL_RESIDUAL_RAD = Math.toRadians(3.0)
        private val LOW_DYNAMICS_MAXIMUM_RESIDUAL_RAD = Math.toRadians(15.0)
        private val MAXIMUM_VALID_GRAVITY_RESIDUAL_RAD = Math.toRadians(30.0)
        private val LOW_DYNAMICS_FULL_ANGULAR_RATE_RADPS = Math.toRadians(5.0)
        private val LOW_DYNAMICS_MAXIMUM_ANGULAR_RATE_RADPS = Math.toRadians(35.0)
        private const val LOW_DYNAMICS_FULL_ACCELERATION_CHANGE_MPS3 = 1.0
        private const val LOW_DYNAMICS_MAXIMUM_ACCELERATION_CHANGE_MPS3 = 8.0
        private const val ACCELERATION_CHANGE_FILTER_ALPHA = 0.05
        private const val MINIMUM_CORRECTION_WEIGHT = 0.05
        private const val VALID_CONFIDENCE = 0.35
        private const val MINIMUM_DT_SEC = 0.0005
        private const val MAXIMUM_DT_SEC = 0.1
        private const val MAXIMUM_ACCELERATION_DT_SEC = 0.1
        private const val MINIMUM_VECTOR_NORM = 1e-12
    }
}

enum class GravityCorrectionMode {
    STATIONARY_GRAVITY,
    LOW_DYNAMICS_GRAVITY,
    PROPAGATION_ONLY,
    RECOVERING
}

data class VehicleAttitudeSnapshot(
    val elapsedRealtimeNs: Long,
    val quaternion: AttitudeQuaternion,
    val rollRad: Double,
    val pitchRad: Double,
    val yawRad: Double,
    val rollRateRadps: Double,
    val pitchRateRadps: Double,
    val yawRateRadps: Double,
    val gravityResidualRad: Double?,
    val accelerationNormConfidence: Double,
    val gravityDirectionConfidence: Double,
    val gravityCorrectionWeight: Double,
    val gravityCorrectionMode: GravityCorrectionMode,
    val accelerationChangeMps3: Double,
    val attitudeValid: Boolean,
    val attitudeConfidence: Double
)
