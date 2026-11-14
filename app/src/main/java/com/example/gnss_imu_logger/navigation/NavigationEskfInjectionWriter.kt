package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 水平GNSS観測から公称yawへ注入したESKF補正をCSVへ保存する。 */
class NavigationEskfInjectionWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationEskfInjectionRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,position_accepted,velocity_accepted," +
                    "raw_yaw_correction_rad,applied_yaw_correction_rad," +
                    "horizontal_speed_mps,yaw_rate_radps,turning_update," +
                    "yaw_limited,applied,fallback_reason," +
                    "covariance_minimum_diagonal,covariance_maximum_asymmetry"
            )
            records.forEach { value ->
                writer.appendLine(listOf(
                    value.elapsedRealtimeNs,
                    value.positionAccepted,
                    value.velocityAccepted,
                    value.rawYawCorrectionRad.format(),
                    value.appliedYawCorrectionRad.format(),
                    value.horizontalSpeedMps.format(),
                    value.yawRateRadps.format(),
                    value.turningUpdate,
                    value.yawLimited,
                    value.applied,
                    value.fallbackReason,
                    value.covarianceMinimumDiagonal.format(),
                    value.covarianceMaximumAsymmetry.format()
                ).joinToString(","))
            }
        }
    }

    private fun Double?.format(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_eskf_injection.csv" }
}

data class NavigationEskfInjectionRecord(
    val elapsedRealtimeNs: Long,
    val positionAccepted: Boolean,
    val velocityAccepted: Boolean,
    val rawYawCorrectionRad: Double?,
    val appliedYawCorrectionRad: Double,
    val horizontalSpeedMps: Double,
    val yawRateRadps: Double?,
    val turningUpdate: Boolean,
    val yawLimited: Boolean,
    val applied: Boolean,
    val fallbackReason: String,
    val covarianceMinimumDiagonal: Double,
    val covarianceMaximumAsymmetry: Double
)
