package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** Heading追従・再捕捉の状態と制限付きyaw補正をCSVへ保存する。 */
class NavigationHeadingTrackingWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationHeadingTrackingRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,direction_state,state_before,state_after," +
                    "predicted_yaw_rad,target_heading_rad,raw_innovation_rad," +
                    "limited_correction_rad,corrected_yaw_rad,bearing_accuracy_rad," +
                    "speed_accuracy_mps,speed_mps,yaw_rate_radps," +
                    "consecutive_rejection_count,candidate_count,candidate_duration_sec," +
                    "bearing_circular_std_rad,reacquisition_started,correction_applied," +
                    "heading_update_accepted,rejection_reason"
            )
            records.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs,
                    value.directionState.name,
                    value.stateBefore.name,
                    value.stateAfter.name,
                    value.predictedYawRad.format(),
                    value.targetHeadingRad.format(),
                    value.rawInnovationRad.format(),
                    value.limitedCorrectionRad.format(),
                    value.correctedYawRad.format(),
                    value.bearingAccuracyRad.format(),
                    value.speedAccuracyMps.format(),
                    value.speedMps.format(),
                    value.yawRateRadps.format(),
                    value.consecutiveRejectionCount,
                    value.candidateCount,
                    value.candidateDurationSec.format(),
                    value.bearingCircularStdRad.format(),
                    value.reacquisitionStarted,
                    value.correctionApplied,
                    value.headingUpdateAccepted,
                    value.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_heading_tracking.csv" }
}
