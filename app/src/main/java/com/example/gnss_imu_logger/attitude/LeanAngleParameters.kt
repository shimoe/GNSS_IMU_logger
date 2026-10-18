package com.example.gnss_imu_logger.attitude

import kotlin.math.PI

/**
 * 単車ロール推定の調整値。
 * 入力: 時間[s]、速度[m/s]、加速度[m/s^2]、角速度[rad/s]、角加速度[rad/s^2]
 * 出力: LeanAngleEstimatorが使用する不変なパラメーター集合
 */
data class LeanAngleParameters(
    val parameterSetId: String,
    val lowPassCutoffHz: Double,
    val minimumCentripetalSpeedMps: Double,
    val minimumReferenceConfidence: Double,
    val fullSpeedAccelerationMps2: Double,
    val maximumSpeedAccelerationMps2: Double,
    val fullYawAccelerationRadps2: Double,
    val maximumYawAccelerationRadps2: Double,
    val fullRollRateRadps: Double,
    val maximumRollRateRadps: Double,
    val minimumWindowSec: Double,
    val windowRetentionSec: Double,
    val maximumSpeedAgeSec: Double,
    val minimumSpeedIntervalSec: Double,
    val maximumSpeedIntervalSec: Double,
    val maximumAccelerationIntervalSec: Double,
    val maximumGyroscopeIntervalSec: Double,
    val correctionFilterAlpha: Double,
    val dynamicsFilterAlpha: Double,
    val holdCorrectionDecaySec: Double,
    val maximumGravityErrorMps2: Double
) {
    init {
        require(parameterSetId.isNotBlank()) { "パラメーターセットIDは空にできません。" }
        require(lowPassCutoffHz > 0.0) { "ローパスカットオフ周波数は0 Hzより大きくしてください。" }
        require(minimumCentripetalSpeedMps >= 0.0) { "最低向心加速度参照速度は0 m/s以上にしてください。" }
        require(minimumReferenceConfidence in 0.0..1.0) { "最低参照信頼度は0から1の範囲にしてください。" }
        require(fullSpeedAccelerationMps2 < maximumSpeedAccelerationMps2) {
            "速度変化量の最大信頼度しきい値は無効化しきい値未満にしてください。"
        }
        require(fullYawAccelerationRadps2 < maximumYawAccelerationRadps2) {
            "ヨー角加速度の最大信頼度しきい値は無効化しきい値未満にしてください。"
        }
        require(fullRollRateRadps < maximumRollRateRadps) {
            "ロールレートの最大信頼度しきい値は無効化しきい値未満にしてください。"
        }
        require(minimumWindowSec > 0.0) { "時間窓は0秒より大きくしてください。" }
        require(windowRetentionSec >= minimumWindowSec) { "履歴保持時間は時間窓以上にしてください。" }
        require(maximumSpeedAgeSec > 0.0) { "GNSS速度の最大経過時間は0秒より大きくしてください。" }
        require(minimumSpeedIntervalSec < maximumSpeedIntervalSec) {
            "GNSS速度更新間隔の最小値は最大値未満にしてください。"
        }
        require(correctionFilterAlpha in 0.0..1.0) { "補正量フィルター係数は0から1の範囲にしてください。" }
        require(dynamicsFilterAlpha in 0.0..1.0) { "運動状態フィルター係数は0から1の範囲にしてください。" }
        require(holdCorrectionDecaySec > 0.0) { "HOLD時の補正減衰時間は0秒より大きくしてください。" }
        require(maximumGravityErrorMps2 > 0.0) { "重力ノルム許容誤差は0 m/s^2より大きくしてください。" }
    }

    val windowRetentionNs: Long
        get() = (windowRetentionSec * NS_PER_SEC).toLong()

    val maximumSpeedAgeNs: Long
        get() = (maximumSpeedAgeSec * NS_PER_SEC).toLong()

    companion object {
        /** 現在の実走評価に使用する既定値。 */
        val DEFAULT = LeanAngleParameters(
            parameterSetId = "lean_default_v1",
            lowPassCutoffHz = 5.0,
            minimumCentripetalSpeedMps = 3.0,
            minimumReferenceConfidence = 0.2,
            fullSpeedAccelerationMps2 = 0.3,
            maximumSpeedAccelerationMps2 = 1.5,
            fullYawAccelerationRadps2 = 5.0 * PI / 180.0,
            maximumYawAccelerationRadps2 = 30.0 * PI / 180.0,
            fullRollRateRadps = 10.0 * PI / 180.0,
            maximumRollRateRadps = 60.0 * PI / 180.0,
            minimumWindowSec = 1.9,
            windowRetentionSec = 2.1,
            maximumSpeedAgeSec = 1.5,
            minimumSpeedIntervalSec = 0.2,
            maximumSpeedIntervalSec = 2.0,
            maximumAccelerationIntervalSec = 0.1,
            maximumGyroscopeIntervalSec = 0.1,
            correctionFilterAlpha = 0.05,
            dynamicsFilterAlpha = 0.05,
            holdCorrectionDecaySec = 2.0,
            maximumGravityErrorMps2 = 1.0
        )

        private const val NS_PER_SEC = 1_000_000_000.0
    }
}
