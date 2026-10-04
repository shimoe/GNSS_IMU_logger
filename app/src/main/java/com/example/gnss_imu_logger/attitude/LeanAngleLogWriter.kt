package com.example.gnss_imu_logger.attitude

import com.example.gnss_imu_logger.log.CsvLogWriter
import java.io.Closeable
import java.io.File

/**
 * 診断モードでロール推定値と中間値を保存する。
 * 入力: LeanAngleSnapshot
 * 出力: lean_angle.csv
 */
class LeanAngleLogWriter(file: File) : Closeable {
    private val writer = CsvLogWriter(
        file,
        listOf(
            "elapsed_realtime_ns", "roll_rad", "roll_deg",
            "reference_roll_rad", "measured_roll_rate_radps",
            "corrected_roll_rate_radps", "drift_correction_radps",
            "yaw_rate_radps", "speed_mps", "centripetal_acceleration_mps2",
            "reference_source", "window_duration_ms", "estimate_valid"
        )
    )

    fun write(value: LeanAngleSnapshot) {
        writer.write(
            listOf(
                value.elapsedRealtimeNs,
                value.rollRad,
                Math.toDegrees(value.rollRad),
                value.referenceRollRad,
                value.measuredRollRateRadps,
                value.correctedRollRateRadps,
                value.driftCorrectionRadps,
                value.yawRateRadps,
                value.speedMps,
                value.centripetalAccelerationMps2,
                value.referenceSource.name,
                value.windowDurationMs,
                value.estimateValid
            )
        )
    }

    override fun close() = writer.close()
}
