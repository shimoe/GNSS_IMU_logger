package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/**
 * GNSS位置・速度・進行方位更新の採用結果をCSVへ保存する。
 * 入力: NavigationMeasurementUpdateRecord一覧
 * 出力: navigation_measurement_update.csv
 */
class NavigationMeasurementUpdateWriter(
    private val sessionDirectory: File
) {
    fun write(records: List<NavigationMeasurementUpdateRecord>) {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        sessionDirectory.resolve(FILE_NAME).bufferedWriter().use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,position_accepted,velocity_accepted," +
                    "horizontal_position_innovation_m,velocity_innovation_mps," +
                    "horizontal_position_gate_m,vertical_position_gate_m,velocity_gate_mps," +
                    "position_gain_east,position_gain_north,position_gain_up," +
                    "velocity_gain_east,velocity_gain_north,velocity_gain_up," +
                    "consecutive_position_rejection_count," +
                    "consecutive_velocity_rejection_count,position_update_mode," +
                    "heading_available,heading_accepted,heading_innovation_rad," +
                    "heading_gate_rad,heading_gain,heading_correction_rad," +
                    "heading_variance_rad2,rejection_reason"
            )
            records.forEach { record ->
                writer.appendLine(
                    listOf(
                        record.elapsedRealtimeNs,
                        record.positionAccepted,
                        record.velocityAccepted,
                        record.horizontalPositionInnovationM.format(),
                        record.velocityInnovationMps.format(),
                        record.horizontalPositionGateM.format(),
                        record.verticalPositionGateM.format(),
                        record.velocityGateMps.format(),
                        record.positionGain?.east.format(),
                        record.positionGain?.north.format(),
                        record.positionGain?.up.format(),
                        record.velocityGain?.east.format(),
                        record.velocityGain?.north.format(),
                        record.velocityGain?.up.format(),
                        record.consecutivePositionRejectionCount,
                        record.consecutiveVelocityRejectionCount,
                        record.updateMode.name,
                        record.headingAvailable,
                        record.headingAccepted,
                        record.headingInnovationRad.format(),
                        record.headingGateRad.format(),
                        record.headingGain.format(),
                        record.headingCorrectionRad.format(),
                        record.headingVarianceRad2.format(),
                        record.rejectionReason?.name ?: ""
                    ).joinToString(",")
                )
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_measurement_update.csv"
    }
}
