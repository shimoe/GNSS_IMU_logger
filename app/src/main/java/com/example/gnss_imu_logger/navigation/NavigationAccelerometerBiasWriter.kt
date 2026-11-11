package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 加速度バイアス候補と更新結果をCSVへ保存する。 */
class NavigationAccelerometerBiasWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationAccelerometerBiasUpdateRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,gnss_interval_sec,velocity_innovation_east_mps," +
                    "velocity_innovation_north_mps,acceleration_error_enu_east_mps2," +
                    "acceleration_error_enu_north_mps2,acceleration_error_body_forward_mps2," +
                    "acceleration_error_body_left_mps2,bias_forward_before_mps2," +
                    "bias_left_before_mps2,bias_forward_correction_mps2," +
                    "bias_left_correction_mps2,bias_forward_after_mps2,bias_left_after_mps2," +
                    "forward_candidate_count,left_candidate_count," +
                    "forward_candidate_duration_sec,left_candidate_duration_sec," +
                    "forward_accepted,left_accepted,rejection_reason"
            )
            records.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs,
                    value.gnssIntervalSec.format(),
                    value.velocityInnovationEastMps.format(),
                    value.velocityInnovationNorthMps.format(),
                    value.accelerationErrorEnuEastMps2.format(),
                    value.accelerationErrorEnuNorthMps2.format(),
                    value.accelerationErrorBodyForwardMps2.format(),
                    value.accelerationErrorBodyLeftMps2.format(),
                    value.biasForwardBeforeMps2.format(),
                    value.biasLeftBeforeMps2.format(),
                    value.biasForwardCorrectionMps2.format(),
                    value.biasLeftCorrectionMps2.format(),
                    value.biasForwardAfterMps2.format(),
                    value.biasLeftAfterMps2.format(),
                    value.forwardCandidateCount,
                    value.leftCandidateCount,
                    value.forwardCandidateDurationSec.format(),
                    value.leftCandidateDurationSec.format(),
                    value.forwardAccepted,
                    value.leftAccepted,
                    value.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_accelerometer_bias_update.csv"
    }
}
