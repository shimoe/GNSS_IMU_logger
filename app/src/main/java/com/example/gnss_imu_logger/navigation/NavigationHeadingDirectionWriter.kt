package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

class NavigationHeadingDirectionWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationHeadingDirectionRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter().use { writer ->
            writer.appendLine("elapsed_realtime_ns,speed_mps,angular_rate_norm_radps,regular_innovation_rad,opposite_innovation_rad,evidence,candidate,candidate_duration_sec,state")
            records.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs,
                    value.speedMps.format(),
                    value.angularRateNormRadps.format(),
                    value.regularInnovationRad.format(),
                    value.oppositeInnovationRad.format(),
                    value.evidence.name,
                    value.candidate.name,
                    value.candidateDurationSec.format(),
                    value.state.name
                ).joinToString(","))
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_heading_direction.csv" }
}
