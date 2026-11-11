package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale

/** pitch基準候補と軸・符号の整合性をCSVへ保存する。 */
class NavigationPitchReferenceWriter(private val sessionDirectory: File) {
    fun write(records: List<NavigationPitchReferenceRecord>) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,euler_pitch_rad,vehicle_grade_from_euler_rad," +
                    "vehicle_grade_from_euler_residual_rad,forward_axis_east,forward_axis_north," +
                    "forward_axis_up,forward_axis_grade_rad,euler_forward_axis_residual_rad," +
                    "barometric_vertical_speed_mps,gnss_horizontal_speed_mps," +
                    "barometric_road_grade_rad,road_grade_age_ms,gravity_pitch_rad," +
                    "gravity_observation_valid,rotation_vector_pitch_rad," +
                    "rotation_vector_residual_rad,pitch_road_grade_residual_rad," +
                    "pitch_gravity_residual_rad,window_sample_count,euler_pitch_median_rad," +
                    "forward_axis_grade_median_rad,road_grade_median_rad," +
                    "gravity_pitch_median_rad,axis_consistent,pitch_drift_candidate," +
                    "valid,rejection_reason"
            )
            records.forEach { v ->
                writer.appendLine(listOf(
                    v.elapsedRealtimeNs, v.eulerPitchRad.f(),
                    v.vehicleGradeFromEulerRad.f(), v.vehicleGradeFromEulerResidualRad.f(),
                    v.forwardAxisEast.f(),
                    v.forwardAxisNorth.f(), v.forwardAxisUp.f(), v.forwardAxisGradeRad.f(),
                    v.eulerForwardAxisResidualRad.f(), v.barometricVerticalSpeedMps.f(),
                    v.gnssHorizontalSpeedMps.f(), v.barometricRoadGradeRad.f(),
                    v.roadGradeAgeMs.f(), v.gravityPitchRad.f(),
                    v.gravityObservationValid, v.rotationVectorPitchRad.f(),
                    v.rotationVectorResidualRad.f(), v.pitchRoadGradeResidualRad.f(),
                    v.pitchGravityResidualRad.f(), v.windowSampleCount,
                    v.eulerPitchMedianRad.f(), v.forwardAxisGradeMedianRad.f(),
                    v.roadGradeMedianRad.f(), v.gravityPitchMedianRad.f(),
                    v.axisConsistent, v.pitchDriftCandidate, v.valid, v.rejectionReason
                ).joinToString(","))
            }
        }
    }

    private fun Double?.f(): String =
        if (this != null && isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object { private const val FILE_NAME = "navigation_pitch_reference.csv" }
}
