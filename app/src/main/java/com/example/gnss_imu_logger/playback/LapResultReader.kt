package com.example.gnss_imu_logger.playback

import com.example.gnss_imu_logger.navigation.LapPositionSource
import com.example.gnss_imu_logger.navigation.LapRecord
import com.example.gnss_imu_logger.navigation.LapType
import java.io.File

/**
 * lap_results.csvをラップ選択再生用モデルへ読み込む。
 * 入力: セッションディレクトリ
 * 出力: 有効なOUT/TIMED/INラップ。ファイルがない場合は空一覧
 */
class LapResultReader {
    fun read(sessionDirectory: File): List<LapRecord> {
        val file = sessionDirectory.resolve(FILE_NAME)
        if (!file.isFile) return emptyList()
        val records = mutableListOf<LapRecord>()
        CsvRowReader(file).forEachRow { row ->
            runCatching { row.toLapRecord() }
                .getOrNull()
                ?.takeIf { it.valid && it.lapType != LapType.INVALID_LAP }
                ?.let(records::add)
        }
        return records.sortedBy { it.startElapsedRealtimeNs }
    }

    private fun Map<String, String>.toLapRecord() = LapRecord(
        lapIndex = requiredInt("lap_index"),
        lapType = LapType.valueOf(required("lap_type")),
        startElapsedRealtimeNs = requiredLong("start_elapsed_realtime_ns"),
        finishElapsedRealtimeNs = requiredLong("finish_elapsed_realtime_ns"),
        durationSec = requiredDouble("duration_sec"),
        distanceM = requiredDouble("distance_m"),
        averageSpeedMps = requiredDouble("average_speed_mps"),
        crossingSpeedMps = optionalDouble("crossing_speed_mps"),
        positionUncertaintyM = optionalDouble("position_uncertainty_m"),
        positionSource = LapPositionSource.valueOf(required("position_source")),
        valid = required("valid").toBooleanStrict(),
        invalidReason = get("invalid_reason").orEmpty(),
        durationOutlier = get("duration_outlier")?.toBooleanStrictOrNull() ?: false
    )

    private fun Map<String, String>.required(name: String): String =
        get(name)?.takeIf { it.isNotBlank() } ?: error("lap_results.csvの${name}が空です。")

    private fun Map<String, String>.requiredInt(name: String): Int =
        required(name).toIntOrNull() ?: error("lap_results.csvの${name}が不正です。")

    private fun Map<String, String>.requiredLong(name: String): Long =
        required(name).toLongOrNull() ?: error("lap_results.csvの${name}が不正です。")

    private fun Map<String, String>.requiredDouble(name: String): Double =
        optionalDouble(name) ?: error("lap_results.csvの${name}が不正です。")

    private fun Map<String, String>.optionalDouble(name: String): Double? =
        get(name)?.takeIf { it.isNotBlank() }?.toDoubleOrNull()?.takeIf { it.isFinite() }

    companion object { private const val FILE_NAME = "lap_results.csv" }
}
