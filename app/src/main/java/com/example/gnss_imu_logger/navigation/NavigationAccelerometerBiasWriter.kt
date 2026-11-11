package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 車体軸回帰の窓ID、終了理由、回帰品質、提案補正量をCSVへ保存する。 */
class NavigationAccelerometerBiasWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationAccelerometerBiasUpdateRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,gnss_interval_sec,velocity_innovation_east_mps," +
                    "velocity_innovation_north_mps,bias_forward_before_mps2," +
                    "bias_left_before_mps2,bias_forward_correction_mps2," +
                    "bias_left_correction_mps2,bias_forward_after_mps2," +
                    "bias_left_after_mps2,window_sequence,window_started_ns," +
                    "window_last_sample_ns,window_completed_ns,window_state," +
                    "hold_reason,window_end_reason,hold_duration_sec," +
                    "valid_sample_gap_sec,window_sample_count,valid_duration_sec," +
                    "window_yaw_change_rad,east_slope_mps2,north_slope_mps2," +
                    "east_residual_mad_mps,north_residual_mad_mps," +
                    "body_forward_innovation_first_mps,body_forward_innovation_last_mps," +
                    "body_left_innovation_first_mps,body_left_innovation_last_mps," +
                    "body_forward_slope_mps2,body_left_slope_mps2," +
                    "body_forward_residual_mad_mps,body_left_residual_mad_mps," +
                    "forward_sign_ratio,left_sign_ratio,estimate_available," +
                    "update_enabled,would_accept_forward,would_accept_left," +
                    "proposed_forward_correction_mps2,proposed_left_correction_mps2," +
                    "forward_accepted,left_accepted,forward_rejection_reason," +
                    "left_rejection_reason,rejection_reason"
            )
            records.forEach { v ->
                writer.appendLine(listOf(
                    v.elapsedRealtimeNs, v.gnssIntervalSec.f(),
                    v.velocityInnovationEastMps.f(), v.velocityInnovationNorthMps.f(),
                    v.biasForwardBeforeMps2.f(), v.biasLeftBeforeMps2.f(),
                    v.biasForwardCorrectionMps2.f(), v.biasLeftCorrectionMps2.f(),
                    v.biasForwardAfterMps2.f(), v.biasLeftAfterMps2.f(),
                    v.windowSequence, v.windowStartedNs ?: "", v.windowLastSampleNs ?: "",
                    v.windowCompletedNs ?: "", v.windowState.name, v.holdReason,
                    v.windowEndReason, v.holdDurationSec.f(), v.validSampleGapSec.f(),
                    v.windowSampleCount, v.validDurationSec.f(), v.windowYawChangeRad.f(),
                    v.eastSlopeMps2.f(), v.northSlopeMps2.f(),
                    v.eastResidualMadMps.f(), v.northResidualMadMps.f(),
                    v.bodyForwardInnovationFirstMps.f(), v.bodyForwardInnovationLastMps.f(),
                    v.bodyLeftInnovationFirstMps.f(), v.bodyLeftInnovationLastMps.f(),
                    v.bodyForwardSlopeMps2.f(), v.bodyLeftSlopeMps2.f(),
                    v.bodyForwardResidualMadMps.f(), v.bodyLeftResidualMadMps.f(),
                    v.forwardSignRatio.f(), v.leftSignRatio.f(), v.estimateAvailable,
                    v.updateEnabled, v.wouldAcceptForward, v.wouldAcceptLeft,
                    v.proposedForwardCorrectionMps2.f(), v.proposedLeftCorrectionMps2.f(),
                    v.forwardAccepted, v.leftAccepted, v.forwardRejectionReason,
                    v.leftRejectionReason, v.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.f(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_accelerometer_bias_update.csv" }
}
