package com.example.gnss_imu_logger.playback

import java.io.File
import kotlin.math.sqrt

/** 改善済みオフライン自己位置推定を再生画面用モデルへ読み込む。 */
class NavigationPlaybackReader {
    fun read(sessionDirectory: File): NavigationPlaybackResult {
        val predictionFile = sessionDirectory.resolve(PREDICTION_FILE_NAME)
        require(predictionFile.isFile) {
            "自己位置推定結果がありません: ${predictionFile.path}"
        }
        val points = mutableListOf<NavigationPlaybackPoint>()
        CsvRowReader(predictionFile).forEachRow { row ->
            val elapsedRealtimeNs = row.requiredLong("elapsed_realtime_ns")
            val eastM = row.requiredDouble("east_m")
            val northM = row.requiredDouble("north_m")
            val upM = row.requiredDouble("up_m")
            val eastMps = row.requiredDouble("velocity_east_mps")
            val northMps = row.requiredDouble("velocity_north_mps")
            points += NavigationPlaybackPoint(
                elapsedRealtimeNs = elapsedRealtimeNs,
                eastM = eastM,
                northM = northM,
                upM = upM,
                speedMps = sqrt(eastMps * eastMps + northMps * northMps),
                yawRad = row.optionalDouble("yaw_rad"),
                horizontalPositionStdM = row.optionalDouble("horizontal_position_std_m"),
                status = row["status"].orEmpty()
            )
        }
        require(points.isNotEmpty()) { "自己位置推定結果が空です。" }
        return NavigationPlaybackResult(
            points = points,
            assessment = readAssessment(sessionDirectory),
            source = NavigationPlaybackSource.OFFLINE_NAVIGATION
        )
    }

    private fun readAssessment(sessionDirectory: File): String? {
        val file = sessionDirectory.resolve(SUMMARY_FILE_NAME)
        if (!file.isFile) return null
        var assessment: String? = null
        CsvRowReader(file).forEachRow { row ->
            if (row["metric"] == "assessment") assessment = row["value"]
        }
        return assessment
    }

    private fun Map<String, String>.requiredLong(name: String): Long =
        get(name)?.toLongOrNull() ?: error("${name}が不正です。")

    private fun Map<String, String>.requiredDouble(name: String): Double =
        optionalDouble(name) ?: error("${name}が不正です。")

    private fun Map<String, String>.optionalDouble(name: String): Double? =
        get(name)?.takeIf { it.isNotBlank() }?.toDoubleOrNull()

    companion object {
        private const val PREDICTION_FILE_NAME = "navigation_prediction.csv"
        private const val SUMMARY_FILE_NAME = "navigation_prediction_summary.csv"
    }
}

data class NavigationPlaybackResult(
    val points: List<NavigationPlaybackPoint>,
    val assessment: String?,
    val source: NavigationPlaybackSource
) {
    fun nearest(elapsedRealtimeNs: Long, maximumDifferenceNs: Long = 20_000_000L): NavigationPlaybackPoint? {
        var low = 0
        var high = points.lastIndex
        while (low <= high) {
            val middle = (low + high) ushr 1
            if (points[middle].elapsedRealtimeNs < elapsedRealtimeNs) low = middle + 1
            else high = middle - 1
        }
        val candidates = listOfNotNull(points.getOrNull(low), points.getOrNull(low - 1))
        return candidates.minByOrNull { kotlin.math.abs(it.elapsedRealtimeNs - elapsedRealtimeNs) }
            ?.takeIf { kotlin.math.abs(it.elapsedRealtimeNs - elapsedRealtimeNs) <= maximumDifferenceNs }
    }
}

data class NavigationPlaybackPoint(
    val elapsedRealtimeNs: Long,
    val eastM: Double,
    val northM: Double,
    val upM: Double,
    val speedMps: Double,
    val yawRad: Double?,
    val horizontalPositionStdM: Double?,
    val status: String
)

enum class NavigationPlaybackSource {
    OFFLINE_NAVIGATION,
    GNSS_FALLBACK
}
