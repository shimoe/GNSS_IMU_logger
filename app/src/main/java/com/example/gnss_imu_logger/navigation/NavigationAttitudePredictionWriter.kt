package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/**
 * IMU予測による姿勢変化を解析用CSVへ保存する。
 * 入力: 予測時刻、積分時間差[s]、バイアス補正後角速度[rad/s]、予測前後Quaternion
 * 出力: navigation_attitude_prediction.csv
 */
class NavigationAttitudePredictionWriter(
    private val sessionDirectory: File
) {
    fun write(records: List<NavigationAttitudePredictionRecord>) {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        sessionDirectory.resolve(FILE_NAME).bufferedWriter().use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,source,dt_sec," +
                    "angular_rate_forward_radps,angular_rate_left_radps,angular_rate_up_radps," +
                    "quaternion_before_w,quaternion_before_x,quaternion_before_y,quaternion_before_z," +
                    "quaternion_after_w,quaternion_after_x,quaternion_after_y,quaternion_after_z," +
                    "roll_delta_rad,pitch_delta_rad,yaw_delta_rad"
            )
            records.forEach { record ->
                writer.appendLine(
                    listOf(
                        record.elapsedRealtimeNs,
                        record.source,
                        record.dtSec.format(),
                        record.angularRateForwardRadps.format(),
                        record.angularRateLeftRadps.format(),
                        record.angularRateUpRadps.format(),
                        record.quaternionBefore.w.format(),
                        record.quaternionBefore.x.format(),
                        record.quaternionBefore.y.format(),
                        record.quaternionBefore.z.format(),
                        record.quaternionAfter.w.format(),
                        record.quaternionAfter.x.format(),
                        record.quaternionAfter.y.format(),
                        record.quaternionAfter.z.format(),
                        record.rollDeltaRad.format(),
                        record.pitchDeltaRad.format(),
                        record.yawDeltaRad.format()
                    ).joinToString(",")
                )
            }
        }
    }

    private fun Double.format(): String =
        if (isFinite()) String.format(Locale.US, "%.12f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_attitude_prediction.csv"
    }
}
