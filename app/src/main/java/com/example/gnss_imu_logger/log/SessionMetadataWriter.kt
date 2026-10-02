package com.example.gnss_imu_logger.log

import com.example.gnss_imu_logger.model.SessionMetadata
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
            throw IllegalStateException(
                "既存のセッション情報を更新できませんでした"
            )
        }
        if (!temp.renameTo(file)) {
            throw IllegalStateException(
                "セッション情報を確定できませんでした"
            )
        }
    }

    private fun toJson(metadata: SessionMetadata): String = buildString {
        appendLine("{")
        appendLine("  \"format_version\": ${metadata.formatVersion},")
        appendLine("  \"session_id\": ${jsonString(metadata.sessionId)},")
        appendLine(
            "  \"status\": " +
                "${jsonString(metadata.status.name.lowercase())},"
        )
        appendLine(
            "  \"mode\": " +
                "${jsonString(metadata.mode.name.lowercase())},"
        )
        appendLine("  \"device\": {")
        appendLine(
            "    \"manufacturer\": " +
                "${jsonString(metadata.manufacturer)},"
        )
        appendLine("    \"model\": ${jsonString(metadata.model)},")
        appendLine(
            "    \"android_version\": " +
                "${jsonString(metadata.androidVersion)},"
        )
        appendLine("    \"api_level\": ${metadata.apiLevel}")
        appendLine("  },")
        appendLine("  \"time\": {")
        appendLine(
            "    \"start_elapsed_ns\": ${metadata.startElapsedNs},"
        )
        appendLine("    \"start_utc_ns\": ${metadata.startUtcNs},")
        appendLine(
            "    \"initial_utc_offset_ns\": " +
                "${metadata.initialUtcOffsetNs},"
        )
        appendLine(
            "    \"end_elapsed_ns\": " +
                "${metadata.endElapsedNs ?: "null"},"
        )
        appendLine(
            "    \"end_utc_ns\": ${metadata.endUtcNs ?: "null"}"
        )
        appendLine("  },")
        appendLine(
            "  \"error_message\": " +
                nullableJsonString(metadata.errorMessage)
        )
        appendLine("}")
    }

    private fun nullableJsonString(value: String?): String =
        value?.let(::jsonString) ?: "null"

    private fun jsonString(value: String): String {
        val escaped = value
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t")
        return "\"$escaped\""
    }
}
