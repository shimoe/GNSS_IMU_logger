package com.example.gnss_imu_logger.attitude

import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 車体座標から基準座標への姿勢を表す単位Quaternion。
 * 入力: w, x, y, z
 * 出力: 正規化、積、回転、ロール・ピッチ・ヨー
 */
data class AttitudeQuaternion(
    val w: Double,
    val x: Double,
    val y: Double,
    val z: Double
) {
    fun normalized(): AttitudeQuaternion {
        val norm = sqrt(w * w + x * x + y * y + z * z)
        if (!norm.isFinite() || norm < MINIMUM_NORM) return IDENTITY
        return AttitudeQuaternion(w / norm, x / norm, y / norm, z / norm)
    }

    operator fun times(other: AttitudeQuaternion): AttitudeQuaternion = AttitudeQuaternion(
        w = w * other.w - x * other.x - y * other.y - z * other.z,
        x = w * other.x + x * other.w + y * other.z - z * other.y,
        y = w * other.y - x * other.z + y * other.w + z * other.x,
        z = w * other.z + x * other.y - y * other.x + z * other.w
    )

    fun conjugate(): AttitudeQuaternion = AttitudeQuaternion(w, -x, -y, -z)

    /** 車体座標のベクトルを基準座標へ回転する。 */
    fun rotate(vector: BodyVector): BodyVector {
        val value = this * AttitudeQuaternion(0.0, vector.x, vector.y, vector.z) * conjugate()
        return BodyVector(value.x, value.y, value.z)
    }

    fun toEulerAngles(): EulerAngles {
        val q = normalized()
        val roll = atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        )
        val pitch = asin((2.0 * (q.w * q.y - q.z * q.x)).coerceIn(-1.0, 1.0))
        val yaw = atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return EulerAngles(roll, pitch, yaw)
    }

    companion object {
        val IDENTITY = AttitudeQuaternion(1.0, 0.0, 0.0, 0.0)

        fun fromEulerAngles(rollRad: Double, pitchRad: Double, yawRad: Double): AttitudeQuaternion {
            val cr = cos(rollRad * 0.5)
            val sr = sin(rollRad * 0.5)
            val cp = cos(pitchRad * 0.5)
            val sp = sin(pitchRad * 0.5)
            val cy = cos(yawRad * 0.5)
            val sy = sin(yawRad * 0.5)
            return AttitudeQuaternion(
                w = cr * cp * cy + sr * sp * sy,
                x = sr * cp * cy - cr * sp * sy,
                y = cr * sp * cy + sr * cp * sy,
                z = cr * cp * sy - sr * sp * cy
            ).normalized()
        }

        /** 角速度[rad/s]と時間差[s]から微小回転を作る。 */
        fun fromAngularVelocity(rateRadps: BodyVector, dtSec: Double): AttitudeQuaternion {
            val angle = sqrt(
                rateRadps.x * rateRadps.x +
                    rateRadps.y * rateRadps.y +
                    rateRadps.z * rateRadps.z
            ) * dtSec
            if (!angle.isFinite() || angle < MINIMUM_ANGLE_RAD) return IDENTITY
            val rateNorm = angle / dtSec
            val scale = sin(angle * 0.5) / rateNorm
            return AttitudeQuaternion(
                w = cos(angle * 0.5),
                x = rateRadps.x * scale,
                y = rateRadps.y * scale,
                z = rateRadps.z * scale
            ).normalized()
        }

        private const val MINIMUM_NORM = 1e-12
        private const val MINIMUM_ANGLE_RAD = 1e-12
    }
}

data class EulerAngles(
    val rollRad: Double,
    val pitchRad: Double,
    val yawRad: Double
)
