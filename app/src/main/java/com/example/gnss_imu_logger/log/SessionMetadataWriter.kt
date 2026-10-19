package com.example.gnss_imu_logger.log

import com.example.gnss_imu_logger.calibration.CalibrationResult
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
        appendLine("  \"device_mount\": ${json(m.deviceMount.lowercase())},")
        appendLeanAngleParameters(m)
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

    private fun StringBuilder.appendLeanAngleParameters(m: SessionMetadata) {
        val value = m.leanAngleParameters
        appendLine("  \"lean_angle_parameters\": {")
        appendLine("    \"parameter_set_id\": ${json(value.parameterSetId)},")
        appendLine("    \"low_pass_cutoff_hz\": ${value.lowPassCutoffHz},")
        appendLine("    \"minimum_centripetal_speed_mps\": ${value.minimumCentripetalSpeedMps},")
        appendLine("    \"minimum_reference_confidence\": ${value.minimumReferenceConfidence},")
        appendLine("    \"full_speed_acceleration_mps2\": ${value.fullSpeedAccelerationMps2},")
        appendLine("    \"maximum_speed_acceleration_mps2\": ${value.maximumSpeedAccelerationMps2},")
        appendLine("    \"full_yaw_acceleration_radps2\": ${value.fullYawAccelerationRadps2},")
        appendLine("    \"maximum_yaw_acceleration_radps2\": ${value.maximumYawAccelerationRadps2},")
        appendLine("    \"full_roll_rate_radps\": ${value.fullRollRateRadps},")
        appendLine("    \"maximum_roll_rate_radps\": ${value.maximumRollRateRadps},")
        appendLine("    \"minimum_window_sec\": ${value.minimumWindowSec},")
        appendLine("    \"window_retention_sec\": ${value.windowRetentionSec},")
        appendLine("    \"maximum_speed_age_sec\": ${value.maximumSpeedAgeSec},")
        appendLine("    \"minimum_speed_interval_sec\": ${value.minimumSpeedIntervalSec},")
        appendLine("    \"maximum_speed_interval_sec\": ${value.maximumSpeedIntervalSec},")
        appendLine("    \"maximum_acceleration_interval_sec\": ${value.maximumAccelerationIntervalSec},")
        appendLine("    \"maximum_gyroscope_interval_sec\": ${value.maximumGyroscopeIntervalSec},")
        appendLine("    \"correction_filter_alpha\": ${value.correctionFilterAlpha},")
        appendLine("    \"dynamics_filter_alpha\": ${value.dynamicsFilterAlpha},")
        appendLine("    \"hold_correction_decay_sec\": ${value.holdCorrectionDecaySec},")
        appendLine("    \"maximum_gravity_error_mps2\": ${value.maximumGravityErrorMps2}")
        appendLine("  },")
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
        appendLine("    },")
        appendCalibration(summary.calibration)
        appendLine(",")
        appendLeanAngleSummary(summary.leanAngle)
        appendLine("  }")
    }

    private fun StringBuilder.appendLeanAngleSummary(
        value: com.example.gnss_imu_logger.attitude.LeanAngleSummary?
    ) {
        if (value == null) {
            appendLine("    \"lean_angle\": null")
            return
        }
        appendLine("    \"lean_angle\": {")
        appendLine("      \"total_count\": ${value.totalCount},")
        appendLine("      \"valid_count\": ${value.validCount},")
        appendLine("      \"gravity_count\": ${value.gravityCount},")
        appendLine("      \"centripetal_count\": ${value.centripetalCount},")
        appendLine("      \"hold_count\": ${value.holdCount},")
        appendLine("      \"gravity_duration_ms\": ${value.gravityDurationMs},")
        appendLine("      \"centripetal_duration_ms\": ${value.centripetalDurationMs},")
        appendLine("      \"hold_duration_ms\": ${value.holdDurationMs},")
        appendLine("      \"reference_valid_ratio\": ${value.referenceValidRatio},")
        appendLine("      \"mean_reference_confidence\": ${value.meanReferenceConfidence},")
        appendLine("      \"maximum_absolute_drift_correction_radps\": ${value.maximumAbsoluteDriftCorrectionRadps},")
        appendLine("      \"drift_correction_rms_radps\": ${value.driftCorrectionRmsRadps}")
        appendLine("    }")
    }

    private fun StringBuilder.appendCalibration(value: CalibrationResult?) {
        if (value == null) {
            appendLine("    \"calibration\": null")
            return
        }
        appendLine("    \"calibration\": {")
        appendLine("      \"started_elapsed_ns\": ${value.startedElapsedNs},")
        appendLine("      \"completed_elapsed_ns\": ${value.completedElapsedNs},")
        appendLine("      \"accelerometer_samples\": ${value.accelerometerSamples},")
        appendLine("      \"gyroscope_samples\": ${value.gyroscopeSamples},")
        appendVector("gravity_mps2", value.gravityMps2, true)
        appendVector("gravity_std_mps2", value.gravityStdMps2, true)
        appendVector("gyroscope_bias_radps", value.gyroscopeBiasRadps, true)
        appendVector("gyroscope_std_radps", value.gyroscopeStdRadps, false)
        appendLine("    }")
    }

    private fun StringBuilder.appendVector(
        name: String,
        value: com.example.gnss_imu_logger.calibration.Vector3,
        trailingComma: Boolean
    ) {
        appendLine("      \"$name\": {")
        appendLine("        \"x\": ${value.x},")
        appendLine("        \"y\": ${value.y},")
        appendLine("        \"z\": ${value.z}")
        appendLine("      }${if (trailingComma) "," else ""}")
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
