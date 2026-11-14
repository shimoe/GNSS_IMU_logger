package com.example.gnss_imu_logger.navigation

import org.json.JSONObject
import java.io.File

/** lap_config.jsonから方向付きスタートライン設定を読み込む。 */
class LapConfigReader {
    fun readOrNull(sessionDirectory: File): LapDetectionConfig? {
        return readOrNullFromFile(sessionDirectory.resolve(FILE_NAME))
    }

    fun readOrNullFromFile(file: File): LapDetectionConfig? {
        if (!file.isFile) return null
        val json = JSONObject(file.readText(Charsets.UTF_8))
        require(json.optInt("schema_version", 1) == 1) {
            "未対応のラップ設定形式です。"
        }
        fun required(name: String): Double = json.optDouble(name, Double.NaN).also {
            require(it.isFinite()) { "lap_config.jsonの${name}が不正です。" }
        }
        return LapDetectionConfig(
            lineStart = GeodeticPosition(
                required("line_start_latitude_deg"),
                required("line_start_longitude_deg"),
                0.0
            ),
            lineEnd = GeodeticPosition(
                required("line_end_latitude_deg"),
                required("line_end_longitude_deg"),
                0.0
            ),
            allowedDirection = runCatching {
                LapCrossingDirection.valueOf(json.getString("allowed_direction"))
            }.getOrElse { error("lap_config.jsonの通過方向が不正です。") },
            minimumSpeedMps = json.optDouble("minimum_speed_mps", 3.0),
            minimumLapDurationSec = json.optDouble("minimum_lap_duration_sec", 30.0),
            rearmDistanceM = json.optDouble("rearm_distance_m", 40.0),
            maximumPositionUncertaintyM =
                json.optDouble("maximum_position_uncertainty_m", 20.0),
            lineEndExtensionM = json.optDouble("line_end_extension_m", 10.0),
            maximumUncertaintyExtensionM =
                json.optDouble("maximum_uncertainty_extension_m", 15.0),
            minimumNormalSpeedMps = json.optDouble("minimum_normal_speed_mps", 1.0)
        )
    }

    companion object { private const val FILE_NAME = "lap_config.json" }
}
