package com.example.gnss_imu_logger.attitude

import com.example.gnss_imu_logger.log.CsvLogWriter
import java.io.Closeable
import java.io.File

/**
 * 診断モードで6軸姿勢推定結果を保存する。
 * 入力: VehicleAttitudeSnapshot
 * 出力: vehicle_attitude.csv
 */
class VehicleAttitudeLogWriter(file: File) : Closeable {
    private val writer = CsvLogWriter(
        file,
        listOf(
            "elapsed_realtime_ns",
            "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
            "roll_rad", "pitch_rad", "yaw_rad",
            "roll_rate_radps", "pitch_rate_radps", "yaw_rate_radps",
            "gravity_residual_rad", "attitude_valid", "attitude_confidence"
        )
    )

    fun write(value: VehicleAttitudeSnapshot) {
        writer.write(
            listOf(
                value.elapsedRealtimeNs,
                value.quaternion.w,
                value.quaternion.x,
                value.quaternion.y,
                value.quaternion.z,
                value.rollRad,
                value.pitchRad,
                value.yawRad,
                value.rollRateRadps,
                value.pitchRateRadps,
                value.yawRateRadps,
                value.gravityResidualRad,
                value.attitudeValid,
                value.attitudeConfidence
            )
        )
    }

    override fun close() = writer.close()
}
