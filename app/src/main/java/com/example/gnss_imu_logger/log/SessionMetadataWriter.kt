package com.example.gnss_imu_logger.log

import com.example.gnss_imu_logger.model.SessionMetadata
import com.example.gnss_imu_logger.model.SensorSummary
import java.io.File

/**
 * session.jsonを一時ファイル経由で安全に更新する。
 * 入力: SessionMetadata
 * 出力: session.json
 */
class SessionMetadataWriter(private val file: File) {
    fun write(metadata: SessionMetadata) {
        val temp = File(file.parentFile, "${file.name}.tmp")
        temp.writeText(toJson(metadata), Charsets.UTF_8)
        if (file.exists() && !file.delete()) {
            throw IllegalStateException("既存のセッション情報を更新できませんでした")
        }
        if (!temp.renameTo(file)) {
            throw IllegalStateException("セッション情報を確定できませんでした")
        }
    }

    private fun toJson(m: SessionMetadata): String = buildString {
        appendLine("{")
        appendLine("  \"format_version\": ${m.formatVersion},")
        appendLine("  \"session_id\": ${json(m.sessionId)},")
        appendLine("  \"status\": ${json(m.status.name.lowercase())},")
        appendLine("  \"mode\": ${json(m.mode.name.lowercase())},")
        appendLine("  \"device\": {")
        appendLine("    \"manufacturer\": ${json(m.manufacturer)},")
        appendLine("    \"model\": ${json(m.model)},")
        appendLine("    \"android_version\": ${json(m.androidVersion)},")
        appendLine("    \"api_level\": ${m.apiLevel}")
        appendLine("  },")
        appendLine("  \"time\": {")
        appendLine("    \"start_elapsed_ns\": ${m.startElapsedNs},")
        appendLine("    \"start_utc_ns\": ${m.startUtcNs},")
        appendLine("    \"initial_utc_offset_ns\": ${m.initialUtcOffsetNs},")
        appendLine("    \"end_elapsed_ns\": ${m.endElapsedNs ?: "null"},")
        appendLine("    \"end_utc_ns\": ${m.endUtcNs ?: "null"}")
        appendLine("  },")
        appendLine("  \"error_message\": ${nullableJson(m.errorMessage)},")
        appendSummary(m)
        appendLine("}")
    }

    private fun StringBuilder.appendSummary(m: SessionMetadata) {
        val summary = m.summary
        if (summary == null) {
            appendLine("  \"statistics\": null")
            return
        }
        appendLine("  \"statistics\": {")
        appendSensor("accelerometer", summary.accelerometer, true)
        appendSensor("gyroscope", summary.gyroscope, true)
        appendLine("    \"gnss\": {")
        appendLine("      \"count\": ${summary.gnss.count},")
        appendLine("      \"mean_rate_hz\": ${summary.gnss.meanRateHz},")
        appendLine("      \"maximum_interval_ms\": ${summary.gnss.maximumIntervalMs}")
        appendLine("    },")
        appendLine("    \"total_bytes\": ${summary.totalBytes},")
        appendLine("    \"free_bytes_at_end\": ${summary.freeBytesAtEnd},")
        appendLine("    \"measurement\": {")
        appendLine("      \"measurement_started_elapsed_ns\": ${summary.measurement.measurementStartedElapsedNs},")
        appendLine("      \"first_ready_elapsed_ns\": ${summary.measurement.firstReadyElapsedNs ?: "null"},")
        appendLine("      \"last_ready_elapsed_ns\": ${summary.measurement.lastReadyElapsedNs ?: "null"},")
        appendLine("      \"measurement_stopped_elapsed_ns\": ${summary.measurement.measurementStoppedElapsedNs ?: "null"},")
        appendLine("      \"time_to_first_ready_ms\": ${summary.measurement.timeToFirstReadyMs ?: "null"},")
        appendLine("      \"ready_entered_count\": ${summary.measurement.readyEnteredCount},")
        appendLine("      \"degraded_count\": ${summary.measurement.degradedCount},")
        appendLine("      \"gnss_lost_count\": ${summary.measurement.gnssLostCount},")
        appendLine("      \"total_ready_duration_ms\": ${summary.measurement.totalReadyDurationMs},")
        appendLine("      \"total_degraded_duration_ms\": ${summary.measurement.totalDegradedDurationMs}")
        appendLine("    }")
        appendLine("  }")
    }

    private fun StringBuilder.appendSensor(
        name: String,
        value: SensorSummary,
        trailingComma: Boolean
    ) {
        appendLine("    \"$name\": {")
        appendLine("      \"received_count\": ${value.receivedCount},")
        appendLine("      \"written_count\": ${value.writtenCount},")
        appendLine("      \"dropped_count\": ${value.droppedCount},")
        appendLine("      \"estimated_missing_count\": ${value.estimatedMissingCount},")
        appendLine("      \"mean_rate_hz\": ${value.meanRateHz},")
        appendLine("      \"median_interval_ms\": ${value.medianIntervalMs},")
        appendLine("      \"maximum_interval_ms\": ${value.maximumIntervalMs}")
        appendLine("    }${if (trailingComma) "," else ""}")
    }

    private fun nullableJson(value: String?): String = value?.let(::json) ?: "null"

    private fun json(value: String): String {
        val escaped = value
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t")
        return "\"$escaped\""
    }
}
