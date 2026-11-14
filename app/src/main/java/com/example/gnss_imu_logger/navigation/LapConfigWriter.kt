package com.example.gnss_imu_logger.navigation

import org.json.JSONObject
import java.io.File

/** ラップ設定を一時ファイルへ書き、再読込確認後に正式ファイルへ置き換える。 */
class LapConfigWriter {
    fun write(sessionDirectory: File, config: LapDetectionConfig) {
        val temporary = sessionDirectory.resolve("lap_config.json.tmp")
        val destination = sessionDirectory.resolve(FILE_NAME)
        val json = JSONObject()
            .put("schema_version", 1)
            .put("line_start_latitude_deg", config.lineStart.latitudeDeg)
            .put("line_start_longitude_deg", config.lineStart.longitudeDeg)
            .put("line_end_latitude_deg", config.lineEnd.latitudeDeg)
            .put("line_end_longitude_deg", config.lineEnd.longitudeDeg)
            .put("allowed_direction", config.allowedDirection.name)
            .put("minimum_speed_mps", config.minimumSpeedMps)
            .put("minimum_lap_duration_sec", config.minimumLapDurationSec)
            .put("rearm_distance_m", config.rearmDistanceM)
            .put("maximum_position_uncertainty_m", config.maximumPositionUncertaintyM)
            .put("line_end_extension_m", config.lineEndExtensionM)
            .put("maximum_uncertainty_extension_m", config.maximumUncertaintyExtensionM)
            .put("minimum_normal_speed_mps", config.minimumNormalSpeedMps)
        temporary.writeText(json.toString(2) + "\n", Charsets.UTF_8)
        require(LapConfigReader().readOrNullFromFile(temporary) != null) {
            "保存したスタートライン設定を読み直せませんでした。"
        }
        if (destination.exists() && !destination.delete()) {
            error("以前のスタートライン設定を削除できません。")
        }
        require(temporary.renameTo(destination)) {
            "スタートライン設定を正式ファイルへ置き換えられませんでした。"
        }
    }

    fun delete(sessionDirectory: File) {
        listOf(FILE_NAME, "lap_results.csv", "lap_events.csv").forEach { name ->
            val file = sessionDirectory.resolve(name)
            if (file.exists() && !file.delete()) error("${name}を削除できませんでした。")
        }
    }

    companion object { private const val FILE_NAME = "lap_config.json" }
}
