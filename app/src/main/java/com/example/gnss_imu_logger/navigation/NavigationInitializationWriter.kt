package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 初期姿勢の計算結果をCSVへ保存する。 */
class NavigationInitializationWriter(private val sessionDirectory: File) {
    fun write(value: NavigationInitializationDiagnostics) {
        sessionDirectory.resolve("navigation_initialization.csv").bufferedWriter().use { writer ->
            writer.appendLine(
                "initialization_start_ns,initialization_end_ns,duration_sec," +
                    "acceleration_sample_count,angular_rate_sample_count," +
                    "mean_acceleration_forward_mps2,mean_acceleration_left_mps2," +
                    "mean_acceleration_up_mps2,acceleration_norm_mean_mps2," +
                    "acceleration_norm_std_mps2,angular_rate_rms_radps," +
                    "initial_roll_rad,initial_pitch_rad,initial_yaw_rad," +
                    "gravity_removed_acceleration_norm_mps2,initialization_result," +
                    "initialization_source,angular_rate_source," +
                    "calibration_completed_ns,position_source_ns," +
                    "gyroscope_bias_forward_radps,gyroscope_bias_left_radps," +
                    "gyroscope_bias_up_radps"
            )
            writer.appendLine(listOf(
                value.startElapsedRealtimeNs,
                value.endElapsedRealtimeNs,
                ((value.endElapsedRealtimeNs-value.startElapsedRealtimeNs)/1e9).format(),
                value.accelerationSampleCount,
                value.angularRateSampleCount,
                value.meanAccelerationBodyMps2.x.format(),
                value.meanAccelerationBodyMps2.y.format(),
                value.meanAccelerationBodyMps2.z.format(),
                value.accelerationNormMeanMps2.format(),
                value.accelerationNormStdMps2.format(),
                value.angularRateRmsRadps.format(),
                value.initialRollRad?.format().orEmpty(),
                value.initialPitchRad?.format().orEmpty(),
                value.initialYawRad?.format().orEmpty(),
                value.gravityRemovedAccelerationNormMps2?.format().orEmpty(),
                value.result.name,
                value.source.name,
                value.angularRateSource.name,
                value.calibrationCompletedElapsedRealtimeNs ?: "",
                value.positionSourceElapsedRealtimeNs ?: "",
                value.gyroscopeBiasBodyRadps.x.format(),
                value.gyroscopeBiasBodyRadps.y.format(),
                value.gyroscopeBiasBodyRadps.z.format()
            ).joinToString(","))
        }
    }

    private fun Double.format() = if (isFinite()) String.format(Locale.US, "%.12f", this) else ""
}
