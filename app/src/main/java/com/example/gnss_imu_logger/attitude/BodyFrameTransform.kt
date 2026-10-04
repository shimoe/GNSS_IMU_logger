package com.example.gnss_imu_logger.attitude

/**
 * Android端末座標を車体座標へ変換する。
 * 入力: 端末座標X/Y/Z
 * 出力: 車体座標X前方/Y左/Z上方
 *
 * 端末画面はライダー側、端末上端は車体上方、端末右側面は車体右側とする。
 * 正確な取り付け角の補正は、後続の走行中バイアス推定で扱う。
 */
object BodyFrameTransform {
    fun deviceToBody(values: FloatArray): BodyVector = BodyVector(
        x = -values.getOrElse(2) { 0f }.toDouble(),
        y = -values.getOrElse(0) { 0f }.toDouble(),
        z = values.getOrElse(1) { 0f }.toDouble()
    )
}

data class BodyVector(
    val x: Double,
    val y: Double,
    val z: Double
)
