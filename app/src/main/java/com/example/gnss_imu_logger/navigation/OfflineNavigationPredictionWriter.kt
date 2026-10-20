package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.BodyVector

import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.util.Locale
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * 純粋IMU予測とGNSS残差を解析用CSVへ保存する。
 * 入力: OfflineNavigationPredictionResult、OfflineNavigationPredictionSummary
 * 出力: navigation_prediction.csv、navigation_gnss_residual.csv、navigation_prediction_summary.csv
 */
class OfflineNavigationPredictionWriter(
    private val sessionDirectory: File
) {
    fun write(
        result: OfflineNavigationPredictionResult,
        summary: OfflineNavigationPredictionSummary
    ) {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        writePrediction(result.samples)
        writeResiduals(result.gnssResiduals)
        writeSummary(summary)
    }

    private fun writePrediction(samples: List<NavigationPredictionSample>) {
        CsvWriter(sessionDirectory.resolve(PREDICTION_FILE_NAME)).use { writer ->
            writer.write(
                listOf(
                    "elapsed_realtime_ns", "east_m", "north_m", "up_m",
                    "velocity_east_mps", "velocity_north_mps", "velocity_up_mps",
                    "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
                    "acceleration_east_mps2", "acceleration_north_mps2",
                    "acceleration_up_mps2",
                    "roll_rad", "pitch_rad", "yaw_rad",
                    "angular_rate_forward_radps", "angular_rate_left_radps",
                    "angular_rate_up_radps", "predicted_roll_rate_radps",
                    "predicted_pitch_rate_radps", "predicted_yaw_rate_radps", "status",
                    "position_east_std_m", "position_north_std_m", "position_up_std_m",
                    "horizontal_position_std_m",
                    "velocity_east_std_mps", "velocity_north_std_mps", "velocity_up_std_mps",
                    "heading_std_rad",
                    "eskf_horizontal_position_std_m", "eskf_velocity_std_mps",
                    "eskf_attitude_std_rad", "eskf_accelerometer_bias_std_mps2",
                    "eskf_gyroscope_bias_std_radps", "eskf_maximum_asymmetry",
                    "eskf_minimum_diagonal"
                )
            )
            samples.forEach { sample ->
                val euler = sample.attitudeBodyToEnu.toEulerAngles()
                val eulerRate = sample.angularRateBodyRadps?.let {
                    bodyRateToEulerRate(
                        rollRad = euler.rollRad,
                        pitchRad = euler.pitchRad,
                        angularRateBodyRadps = it
                    )
                }
                writer.write(
                    listOf(
                        sample.elapsedRealtimeNs,
                        sample.positionEnuM.eastM,
                        sample.positionEnuM.northM,
                        sample.positionEnuM.upM,
                        sample.velocityEnuMps.eastMps,
                        sample.velocityEnuMps.northMps,
                        sample.velocityEnuMps.upMps,
                        sample.attitudeBodyToEnu.w,
                        sample.attitudeBodyToEnu.x,
                        sample.attitudeBodyToEnu.y,
                        sample.attitudeBodyToEnu.z,
                        sample.accelerationEnuMps2.x,
                        sample.accelerationEnuMps2.y,
                        sample.accelerationEnuMps2.z,
                        euler.rollRad,
                        euler.pitchRad,
                        euler.yawRad,
                        sample.angularRateBodyRadps?.x,
                        sample.angularRateBodyRadps?.y,
                        sample.angularRateBodyRadps?.z,
                        eulerRate?.rollRateRadps,
                        eulerRate?.pitchRateRadps,
                        eulerRate?.yawRateRadps,
                        sample.status.name,
                        sample.uncertainty?.positionEastStdM,
                        sample.uncertainty?.positionNorthStdM,
                        sample.uncertainty?.positionUpStdM,
                        sample.uncertainty?.horizontalPositionStdM,
                        sample.uncertainty?.velocityEastStdMps,
                        sample.uncertainty?.velocityNorthStdMps,
                        sample.uncertainty?.velocityUpStdMps,
                        sample.uncertainty?.headingStdRad,
                        sample.eskfUncertainty?.horizontalPositionStdM,
                        sample.eskfUncertainty?.velocityStdMps,
                        sample.eskfUncertainty?.attitudeStdRad,
                        sample.eskfUncertainty?.accelerometerBiasStdMps2,
                        sample.eskfUncertainty?.gyroscopeBiasStdRadps,
                        sample.eskfUncertainty?.maximumAsymmetry,
                        sample.eskfUncertainty?.minimumDiagonal
                    )
                )
            }
        }
    }

    private fun writeResiduals(residuals: List<NavigationGnssResidual>) {
        CsvWriter(sessionDirectory.resolve(RESIDUAL_FILE_NAME)).use { writer ->
            writer.write(
                listOf(
                    "elapsed_realtime_ns", "east_error_m", "north_error_m", "up_error_m",
                    "horizontal_error_m", "velocity_error_mps",
                    "horizontal_position_std_m", "velocity_std_mps",
                    "normalized_horizontal_position_error",
                    "eskf_horizontal_position_std_m",
                    "eskf_normalized_horizontal_position_error"
                )
            )
            residuals.forEach { residual ->
                writer.write(
                    listOf(
                        residual.elapsedRealtimeNs,
                        residual.eastErrorM,
                        residual.northErrorM,
                        residual.upErrorM,
                        residual.horizontalErrorM,
                        residual.velocityErrorMps,
                        residual.horizontalPositionStdM,
                        residual.velocityStdMps,
                        residual.normalizedHorizontalPositionError,
                        residual.eskfHorizontalPositionStdM,
                        residual.eskfNormalizedHorizontalPositionError
                    )
                )
            }
        }
    }

    private fun writeSummary(summary: OfflineNavigationPredictionSummary) {
        CsvWriter(sessionDirectory.resolve(SUMMARY_FILE_NAME)).use { writer ->
            writer.write(listOf("metric", "value", "unit"))
            writer.write(listOf("assessment", summary.assessment.name, ""))
            writer.write(listOf("prediction_sample_count", summary.predictionSampleCount, "count"))
            writer.write(listOf("gnss_residual_count", summary.gnssResidualCount, "count"))
            writer.write(listOf("first_horizontal_error", summary.firstHorizontalErrorM, "m"))
            writer.write(listOf("median_horizontal_error", summary.medianHorizontalErrorM, "m"))
            writer.write(listOf("final_horizontal_error", summary.finalHorizontalErrorM, "m"))
            writer.write(listOf("maximum_horizontal_error", summary.maximumHorizontalErrorM, "m"))
            writer.write(listOf("final_velocity_error", summary.finalVelocityErrorMps, "m/s"))
            writer.write(listOf("maximum_velocity_error", summary.maximumVelocityErrorMps, "m/s"))
            writer.write(listOf("maximum_absolute_up_velocity", summary.maximumAbsoluteUpVelocityMps, "m/s"))
            writer.write(listOf("maximum_linear_acceleration", summary.maximumLinearAccelerationMps2, "m/s^2"))
            writer.write(listOf("imu_gap_sample_count", summary.imuGapSampleCount, "count"))
            writer.write(listOf("gnss_gap_count", summary.gnssGapEvaluations.size, "count"))
            summary.gnssGapEvaluations.forEachIndexed { index, gap ->
                val prefix = "gnss_gap_${index + 1}"
                writer.write(listOf("${prefix}_duration", gap.durationNs / NS_PER_SEC, "s"))
                writer.write(listOf("${prefix}_horizontal_error_increase", gap.horizontalErrorIncreaseM, "m"))
                writer.write(listOf("${prefix}_velocity_error_increase", gap.velocityErrorIncreaseMps, "m/s"))
            }
        }
    }

    /**
     * 車体座標角速度からEuler角速度を算出する。
     * 入力: ロール・ピッチ[rad]、車体X/Y/Z軸角速度[rad/s]
     * 出力: ロール・ピッチ・ヨー角速度[rad/s]。ピッチが特異点付近ならnull。
     */
    private fun bodyRateToEulerRate(
        rollRad: Double,
        pitchRad: Double,
        angularRateBodyRadps: BodyVector
    ): EulerRate? {
        val cosPitch = cos(pitchRad)
        if (abs(cosPitch) < MINIMUM_EULER_RATE_COS_PITCH) return null
        val forwardRate = angularRateBodyRadps.x
        val leftRate = angularRateBodyRadps.y
        val upRate = angularRateBodyRadps.z
        val sinRoll = sin(rollRad)
        val cosRoll = cos(rollRad)
        return EulerRate(
            rollRateRadps = forwardRate +
                leftRate * sinRoll * tan(pitchRad) +
                upRate * cosRoll * tan(pitchRad),
            pitchRateRadps = leftRate * cosRoll - upRate * sinRoll,
            yawRateRadps =
                (leftRate * sinRoll + upRate * cosRoll) / cosPitch
        )
    }

    private data class EulerRate(
        val rollRateRadps: Double,
        val pitchRateRadps: Double,
        val yawRateRadps: Double
    )

    private class CsvWriter(file: File) : Closeable {
        private val writer: BufferedWriter = BufferedWriter(
            OutputStreamWriter(FileOutputStream(file), Charsets.UTF_8)
        )

        fun write(values: List<Any?>) {
            writer.appendLine(values.joinToString(",") { format(it) })
        }

        private fun format(value: Any?): String = when (value) {
            null -> ""
            is Double -> if (value.isFinite()) String.format(Locale.US, "%.9f", value) else ""
            is Float -> if (value.isFinite()) String.format(Locale.US, "%.9f", value) else ""
            else -> value.toString().replace("\"", "\"\"").let { "\"$it\"" }
        }

        override fun close() = writer.close()
    }

    companion object {
        private const val PREDICTION_FILE_NAME = "navigation_prediction.csv"
        private const val MINIMUM_EULER_RATE_COS_PITCH = 1e-3
        private const val RESIDUAL_FILE_NAME = "navigation_gnss_residual.csv"
        private const val SUMMARY_FILE_NAME = "navigation_prediction_summary.csv"
        private const val NS_PER_SEC = 1_000_000_000.0
    }
}
