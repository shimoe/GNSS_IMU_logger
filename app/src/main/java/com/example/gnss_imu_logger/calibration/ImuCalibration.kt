package com.example.gnss_imu_logger.calibration

import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 初期校正結果を後続の姿勢推定へ渡す。
 * 入力: InitialCalibrationが算出した校正結果、未校正ジャイロ[rad/s]
 * 出力: バイアス補正済みジャイロ、初期ロール・ピッチ、重力単位ベクトル
 */
class ImuCalibration private constructor(
    val gravityMps2: Vector3,
    val gyroscopeBiasRadps: Vector3,
    val initialRollRad: Double,
    val initialPitchRad: Double,
    val gravityUnitVector: Vector3
) {
    fun correctGyroscope(values: FloatArray): FloatArray = floatArrayOf(
        values.getOrElse(0) { 0f } - gyroscopeBiasRadps.x.toFloat(),
        values.getOrElse(1) { 0f } - gyroscopeBiasRadps.y.toFloat(),
        values.getOrElse(2) { 0f } - gyroscopeBiasRadps.z.toFloat()
    )

    companion object {
        fun from(result: CalibrationResult): ImuCalibration {
            val gravity = result.gravityMps2
            val norm = sqrt(
                gravity.x * gravity.x +
                    gravity.y * gravity.y +
                    gravity.z * gravity.z
            )
            require(norm > MINIMUM_GRAVITY_NORM_MPS2) {
                "初期重力ベクトルの大きさが不足しています"
            }
            val unit = Vector3(
                gravity.x / norm,
                gravity.y / norm,
                gravity.z / norm
            )
            val roll = atan2(unit.y, unit.z)
            val pitch = asin((-unit.x).coerceIn(-1.0, 1.0))
            return ImuCalibration(
                gravityMps2 = gravity,
                gyroscopeBiasRadps = result.gyroscopeBiasRadps,
                initialRollRad = roll,
                initialPitchRad = pitch,
                gravityUnitVector = unit
            )
        }

        private const val MINIMUM_GRAVITY_NORM_MPS2 = 1.0
    }
}
