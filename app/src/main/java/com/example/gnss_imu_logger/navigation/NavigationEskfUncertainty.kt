package com.example.gnss_imu_logger.navigation

import kotlin.math.sqrt

/**
 * 15状態ESKF予測共分散から抽出した比較用診断値。
 * 入力: 15x15予測共分散
 * 出力: 位置・速度・姿勢・IMUバイアスの1 sigmaと行列診断値
 */
data class NavigationEskfUncertainty(
    val horizontalPositionStdM: Double,
    val velocityStdMps: Double,
    val attitudeStdRad: Double,
    val accelerometerBiasStdMps2: Double,
    val gyroscopeBiasStdRadps: Double,
    val maximumAsymmetry: Double,
    val minimumDiagonal: Double
) {
    companion object {
        fun from(covariance: NavigationEskfCovariance): NavigationEskfUncertainty {
            fun combinedStandardDeviation(start: Int): Double = sqrt(
                (0 until 3).sumOf { axis ->
                    val std = covariance.standardDeviation(start + axis)
                    std * std
                }.coerceAtLeast(0.0)
            )
            return NavigationEskfUncertainty(
                horizontalPositionStdM = sqrt(
                    covariance.standardDeviation(NavigationErrorStateIndex.POSITION).let { it * it } +
                        covariance.standardDeviation(NavigationErrorStateIndex.POSITION + 1).let { it * it }
                ),
                velocityStdMps = combinedStandardDeviation(NavigationErrorStateIndex.VELOCITY),
                attitudeStdRad = combinedStandardDeviation(NavigationErrorStateIndex.ATTITUDE),
                accelerometerBiasStdMps2 = combinedStandardDeviation(NavigationErrorStateIndex.ACCELEROMETER_BIAS),
                gyroscopeBiasStdRadps = combinedStandardDeviation(NavigationErrorStateIndex.GYROSCOPE_BIAS),
                maximumAsymmetry = covariance.maximumAsymmetry(),
                minimumDiagonal = covariance.minimumDiagonal()
            )
        }
    }
}
