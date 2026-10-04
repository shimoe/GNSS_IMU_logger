package com.example.gnss_imu_logger.attitude

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 6軸IMUから車体姿勢を推定する。
 * 入力: 車体座標の加速度[m/s^2]、バイアス補正後角速度[rad/s]、elapsedRealtimeNs[ns]
 * 出力: Quaternion、ロール・ピッチ・ヨー、角速度、信頼度
 */
class VehicleAttitudeEstimator(
    initialRollRad: Double,
    initialPitchRad: Double
) {
    private var attitude = AttitudeQuaternion.fromEulerAngles(
        initialRollRad,
        initialPitchRad,
        0.0
    )
    private var latestAcceleration: BodyVector? = null
    private var lastGyroscopeNs: Long? = null

    @Synchronized
    fun updateAcceleration(deviceValues: FloatArray) {
        latestAcceleration = BodyFrameTransform.deviceToBody(deviceValues)
    }

    @Synchronized
    fun updateGyroscope(
        timestampNs: Long,
        correctedDeviceValuesRadps: FloatArray
    ): VehicleAttitudeSnapshot? {
        val bodyRate = BodyFrameTransform.deviceToBody(correctedDeviceValuesRadps)
        val previousNs = lastGyroscopeNs
        lastGyroscopeNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) return null

        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec !in MINIMUM_DT_SEC..MAXIMUM_DT_SEC) return null

        // ジャイロ積分で姿勢を予測する。
        attitude = (attitude * AttitudeQuaternion.fromAngularVelocity(bodyRate, dtSec)).normalized()

        val acceleration = latestAcceleration
        val confidence = acceleration?.let(::accelerationConfidence) ?: 0.0
        if (acceleration != null && confidence > 0.0) {
            applyGravityCorrection(acceleration, confidence, dtSec)
        }

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
            attitudeValid = confidence >= VALID_CONFIDENCE,
            attitudeConfidence = confidence
        )
    }

    @Synchronized
    fun reset(initialRollRad: Double, initialPitchRad: Double) {
        attitude = AttitudeQuaternion.fromEulerAngles(initialRollRad, initialPitchRad, 0.0)
        latestAcceleration = null
        lastGyroscopeNs = null
    }

    /** 強い並進加速度中は重力補正を弱める。 */
    private fun accelerationConfidence(acceleration: BodyVector): Double {
        val norm = sqrt(
            acceleration.x * acceleration.x +
                acceleration.y * acceleration.y +
                acceleration.z * acceleration.z
        )
        val error = abs(norm - STANDARD_GRAVITY_MPS2)
        return (1.0 - error / MAXIMUM_GRAVITY_ERROR_MPS2).coerceIn(0.0, 1.0)
    }

    /** 加速度から得たロール・ピッチへ小さく補正し、ヨーはジャイロ積分を維持する。 */
    private fun applyGravityCorrection(
        acceleration: BodyVector,
        confidence: Double,
        dtSec: Double
    ) {
        val measuredRoll = atan2(acceleration.y, acceleration.z)
        val measuredPitch = atan2(
            -acceleration.x,
            sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z)
        )
        val current = attitude.toEulerAngles()
        val weight = (GRAVITY_CORRECTION_RATE_PER_SEC * confidence * dtSec).coerceIn(0.0, 1.0)
        val correctedRoll = current.rollRad + normalizeAngle(measuredRoll - current.rollRad) * weight
        val correctedPitch = current.pitchRad + normalizeAngle(measuredPitch - current.pitchRad) * weight
        attitude = AttitudeQuaternion.fromEulerAngles(correctedRoll, correctedPitch, current.yawRad)
    }

    private fun normalizeAngle(angleRad: Double): Double {
        var value = angleRad
        while (value > Math.PI) value -= 2.0 * Math.PI
        while (value < -Math.PI) value += 2.0 * Math.PI
        return value
    }

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val MAXIMUM_GRAVITY_ERROR_MPS2 = 3.0
        private const val GRAVITY_CORRECTION_RATE_PER_SEC = 1.5
        private const val VALID_CONFIDENCE = 0.35
        private const val MINIMUM_DT_SEC = 0.0005
        private const val MAXIMUM_DT_SEC = 0.1
    }
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
    val attitudeValid: Boolean,
    val attitudeConfidence: Double
)
