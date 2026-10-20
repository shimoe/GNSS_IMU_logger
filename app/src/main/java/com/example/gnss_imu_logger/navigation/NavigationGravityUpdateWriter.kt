package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/**
 * 重力方向によるロール・ピッチ補正結果をCSVへ保存する。
 * 入力: NavigationGravityUpdateRecord一覧
 * 出力: navigation_gravity_update.csv
 */
class NavigationGravityUpdateWriter(
    private val sessionDirectory: File
) {
    fun write(records: List<NavigationGravityUpdateRecord>) {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        sessionDirectory.resolve(FILE_NAME).bufferedWriter().use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,available,accepted,acceleration_norm_mps2," +
                    "angular_rate_norm_radps,residual_rad,correction_rad"
            )
            records.forEach { record ->
                writer.appendLine(
                    listOf(
                        record.elapsedRealtimeNs,
                        record.available,
                        record.accepted,
                        record.accelerationNormMps2.format(),
                        record.angularRateNormRadps.format(),
                        record.residualRad.format(),
                        record.correctionRad.format()
                    ).joinToString(",")
                )
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_gravity_update.csv"
    }
}
