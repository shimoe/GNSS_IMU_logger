package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** 水平誤差の進行方向・横方向・pitch・GNSS整合性診断をCSVへ保存する。 */
class NavigationHorizontalErrorWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationHorizontalErrorRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,predicted_velocity_east_mps,predicted_velocity_north_mps," +
                    "gnss_velocity_east_mps,gnss_velocity_north_mps,innovation_east_mps," +
                    "innovation_north_mps,along_track_innovation_mps,cross_track_innovation_mps," +
                    "acceleration_east_mps2,acceleration_north_mps2," +
                    "along_track_acceleration_mps2,cross_track_acceleration_mps2," +
                    "estimated_pitch_rad,vehicle_grade_rad,road_grade_rad," +
                    "pitch_grade_residual_rad," +
                    "gravity_leak_mps2,position_difference_velocity_east_mps," +
                    "position_difference_velocity_north_mps,gnss_velocity_consistency_error_mps," +
                    "gnss_bearing_consistency_error_rad,window_sample_count," +
                    "along_track_innovation_median_mps,cross_track_innovation_median_mps," +
                    "gravity_leak_median_mps2,cause,heading_tracking_state,valid,rejection_reason"
            )
            records.forEach { v ->
                writer.appendLine(listOf(
                    v.elapsedRealtimeNs, v.predictedVelocityEastMps.f(),
                    v.predictedVelocityNorthMps.f(), v.gnssVelocityEastMps.f(),
                    v.gnssVelocityNorthMps.f(), v.innovationEastMps.f(),
                    v.innovationNorthMps.f(), v.alongTrackInnovationMps.f(),
                    v.crossTrackInnovationMps.f(), v.accelerationEastMps2.f(),
                    v.accelerationNorthMps2.f(), v.alongTrackAccelerationMps2.f(),
                    v.crossTrackAccelerationMps2.f(), v.estimatedPitchRad.f(),
                    v.vehicleGradeRad.f(), v.roadGradeRad.f(),
                    v.pitchGradeResidualRad.f(), v.gravityLeakMps2.f(),
                    v.positionDifferenceVelocityEastMps.f(),
                    v.positionDifferenceVelocityNorthMps.f(),
                    v.gnssVelocityConsistencyErrorMps.f(),
                    v.gnssBearingConsistencyErrorRad.f(), v.windowSampleCount,
                    v.alongTrackInnovationMedianMps.f(), v.crossTrackInnovationMedianMps.f(),
                    v.gravityLeakMedianMps2.f(), v.cause.name, v.headingTrackingState.name,
                    v.valid, v.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.f(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_horizontal_error_analysis.csv" }
}
