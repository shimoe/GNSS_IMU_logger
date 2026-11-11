package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

class NavigationHeadingDirectionWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationHeadingDirectionRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter().use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,speed_mps,bearing_accuracy_rad,speed_accuracy_mps," +
                    "yaw_rate_radps,regular_innovation_rad,opposite_innovation_rad," +
                    "candidate_margin_rad,evidence,candidate,candidate_count," +
                    "candidate_duration_sec,candidate_started_ns,last_candidate_evidence_ns," +
                    "state_before,state_after,confirmed_elapsed_realtime_ns," +
                    "conflicting_evidence_count,direction_decision_available," +
                    "direction_decision_rejection_reason,heading_update_allowed," +
                    "heading_update_rejection_reason"
            )
            records.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs,
                    value.speedMps.format(),
                    value.bearingAccuracyRad.format(),
                    value.speedAccuracyMps.format(),
                    value.yawRateRadps.format(),
                    value.regularInnovationRad.format(),
                    value.oppositeInnovationRad.format(),
                    value.candidateMarginRad.format(),
                    value.evidence.name,
                    value.candidate.name,
                    value.candidateCount,
                    value.candidateDurationSec.format(),
                    value.candidateStartedNs ?: "",
                    value.lastCandidateEvidenceNs ?: "",
                    value.stateBefore.name,
                    value.stateAfter.name,
                    value.confirmedElapsedRealtimeNs ?: "",
                    value.conflictingEvidenceCount,
                    value.directionDecisionAvailable,
                    value.directionDecisionRejectionReason,
                    value.headingUpdateAllowed,
                    value.headingUpdateRejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_heading_direction.csv" }
}
