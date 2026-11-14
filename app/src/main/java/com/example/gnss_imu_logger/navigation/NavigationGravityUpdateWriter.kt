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
                    "angular_rate_norm_radps,residual_rad,correction_rad," +
                    "roll_before_rad,pitch_before_rad,yaw_before_rad," +
                    "roll_after_rad,pitch_after_rad,yaw_after_rad," +
                    "roll_delta_rad,pitch_delta_rad,yaw_delta_rad," +
                    "vehicle_grade_before_rad,vehicle_grade_after_rad," +
                    "gravity_residual_before_rad,gravity_residual_after_rad," +
                    "residual_reduction_rad,correction_improved_residual," +
                    "current_direction_residual_after_rad,opposite_direction_residual_after_rad," +
                    "preferred_correction_direction,applied_correction_direction," +
                    "cumulative_pitch_correction_rad,rejection_reason"
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
                        record.correctionRad.format(),
                        record.rollBeforeRad.format(),
                        record.pitchBeforeRad.format(),
                        record.yawBeforeRad.format(),
                        record.rollAfterRad.format(),
                        record.pitchAfterRad.format(),
                        record.yawAfterRad.format(),
                        record.rollDeltaRad.format(),
                        record.pitchDeltaRad.format(),
                        record.yawDeltaRad.format(),
                        record.vehicleGradeBeforeRad.format(),
                        record.vehicleGradeAfterRad.format(),
                        record.gravityResidualBeforeRad.format(),
                        record.gravityResidualAfterRad.format(),
                        record.residualReductionRad.format(),
                        record.correctionImprovedResidual,
                        record.currentDirectionResidualAfterRad.format(),
                        record.oppositeDirectionResidualAfterRad.format(),
                        record.preferredCorrectionDirection.name,
                        record.appliedCorrectionDirection.name,
                        record.cumulativePitchCorrectionRad.format(),
                        record.rejectionReason.name
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
