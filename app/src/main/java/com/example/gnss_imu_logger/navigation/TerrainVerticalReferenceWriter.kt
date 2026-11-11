package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 地理院DEMから推定した鉛直基準差を保存する。 */
class TerrainVerticalReferenceWriter(private val sessionDirectory: File) {
    fun write(value: TerrainVerticalReferenceResult) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine("metric,value,unit")
            writer.writeMetric("valid_sample_count", value.validSampleCount, "count")
            writer.writeMetric("total_sample_count", value.totalSampleCount, "count")
            writer.writeMetric("gnss_to_msl_offset", value.gnssToMslOffsetM, "m")
            writer.writeMetric("gnss_centered_rmse", value.gnssCenteredRmseM, "m")
            writer.writeMetric("gnss_inlier_count", value.gnssInlierCount, "count")
            writer.writeMetric("barometer_to_msl_offset", value.barometerToMslOffsetM, "m")
            writer.writeMetric("barometer_centered_rmse", value.barometerCenteredRmseM, "m")
            writer.writeMetric("barometer_inlier_count", value.barometerInlierCount, "count")
            writer.writeMetric("recommended_altitude_source", value.recommendedAltitudeSource, "")
            writer.writeMetric("altitude_datum", "MSL_TERRAIN_APPROXIMATION", "")
            writer.writeMetric("attribution", ATTRIBUTION, "")
        }
    }

    private fun java.io.BufferedWriter.writeMetric(name: String, value: Any, unit: String) {
        val formatted = if (value is Double) String.format(Locale.US, "%.9f", value) else value
        appendLine(listOf(name, formatted, unit).joinToString(","))
    }

    companion object {
        private const val FILE_NAME = "navigation_vertical_reference.csv"
        private const val ATTRIBUTION = "国土地理院 地理院タイル 標高タイルを加工して作成"
    }
}
