package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 気圧高度とGNSS高度の比較結果をCSVへ保存する。 */
class BarometricAltitudeWriter(private val sessionDirectory: File) {
    fun write(result: BarometricAltitudeResult) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,pressure_hpa,filtered_pressure_hpa," +
                    "relative_altitude_m,calibrated_altitude_m,vertical_speed_mps," +
                    "reference_pressure_hpa,reference_gnss_altitude_m," +
                    "gnss_altitude_m,gnss_vertical_accuracy_m,valid,rejection_reason"
            )
            result.records.forEach { value ->
                writer.appendLine(
                    listOf(
                        value.elapsedRealtimeNs,
                        value.pressureHpa.format(),
                        value.filteredPressureHpa.format(),
                        value.relativeAltitudeM.format(),
                        value.calibratedAltitudeM.format(),
                        value.verticalSpeedMps?.format().orEmpty(),
                        value.referencePressureHpa.format(),
                        value.referenceGnssAltitudeM.format(),
                        value.gnssAltitudeM?.format().orEmpty(),
                        value.gnssVerticalAccuracyM?.format().orEmpty(),
                        value.valid,
                        value.rejectionReason
                    ).joinToString(",")
                )
            }
        }
    }

    private fun Double.format(): String =
        if (isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_barometric_altitude.csv"
    }
}
