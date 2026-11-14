package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** ラップ結果と交差候補の受理・棄却理由をCSVへ保存する。 */
class LapResultWriter(private val sessionDirectory: File) {
    fun write(result: LapAnalysisResult, writeEvents: Boolean) {
        sessionDirectory.resolve(RESULT_FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "lap_index,lap_type,start_elapsed_realtime_ns,finish_elapsed_realtime_ns," +
                    "duration_sec,distance_m,average_speed_mps,crossing_speed_mps," +
                    "position_uncertainty_m,position_source,valid,invalid_reason"
            )
            result.records.forEach { value ->
                writer.appendLine(listOf(
                    value.lapIndex, value.lapType.name, value.startElapsedRealtimeNs,
                    value.finishElapsedRealtimeNs, value.durationSec.f(), value.distanceM.f(),
                    value.averageSpeedMps.f(), value.crossingSpeedMps.f(),
                    value.positionUncertaintyM.f(), value.positionSource.name,
                    value.valid, value.invalidReason
                ).joinToString(","))
            }
        }
        if (!writeEvents) return
        sessionDirectory.resolve(EVENT_FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,state_before,state_after,signed_distance_before_m," +
                    "signed_distance_after_m,line_ratio,crossing_speed_mps,normal_speed_mps," +
                    "position_uncertainty_m,accepted,rejection_reason"
            )
            result.events.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs, value.stateBefore.name, value.stateAfter.name,
                    value.signedDistanceBeforeM.f(), value.signedDistanceAfterM.f(),
                    value.lineRatio.f(), value.crossingSpeedMps.f(), value.normalSpeedMps.f(),
                    value.positionUncertaintyM.f(), value.accepted, value.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.f(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val RESULT_FILE_NAME = "lap_results.csv"
        private const val EVENT_FILE_NAME = "lap_events.csv"
    }
}
