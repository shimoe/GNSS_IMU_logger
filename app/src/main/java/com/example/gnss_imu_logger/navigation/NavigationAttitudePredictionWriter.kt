package com.example.gnss_imu_logger.navigation

import java.io.Closeable
import java.io.File
import java.util.Locale

/**
 * IMU予測による姿勢変化を解析用CSVへ保存する。
 * 入力: 予測時刻、積分時間差[s]、バイアス補正後角速度[rad/s]、予測前後Quaternion
 * 出力: navigation_attitude_prediction.csv
 */
class NavigationAttitudePredictionWriter(
    sessionDirectory: File
) : NavigationAttitudePredictionSink, Closeable {
    private val writer = openWriter(sessionDirectory).also {
        it.appendLine(
            "elapsed_realtime_ns,source,dt_sec," +
                "angular_rate_forward_radps,angular_rate_left_radps,angular_rate_up_radps," +
                "acceleration_body_forward_mps2,acceleration_body_left_mps2," +
                "acceleration_body_up_mps2,acceleration_enu_east_mps2," +
                "acceleration_enu_north_mps2,acceleration_enu_up_mps2," +
                "velocity_east_mps,velocity_north_mps,velocity_up_mps," +
                "roll_after_rad,pitch_after_rad,yaw_after_rad,abnormal_reason," +
                "quaternion_before_w,quaternion_before_x,quaternion_before_y,quaternion_before_z," +
                "quaternion_after_w,quaternion_after_x,quaternion_after_y,quaternion_after_z," +
                "roll_delta_rad,pitch_delta_rad,yaw_delta_rad"
        )
    }

    override fun write(record: NavigationAttitudePredictionRecord) {
        writer.appendLine(
            listOf(
                record.elapsedRealtimeNs,
                record.source,
                record.dtSec.format(),
                record.angularRateForwardRadps.format(),
                record.angularRateLeftRadps.format(),
                record.angularRateUpRadps.format(),
                record.accelerationBodyForwardMps2.format(),
                record.accelerationBodyLeftMps2.format(),
                record.accelerationBodyUpMps2.format(),
                record.accelerationEnuEastMps2.format(),
                record.accelerationEnuNorthMps2.format(),
                record.accelerationEnuUpMps2.format(),
                record.velocityEastMps.format(),
                record.velocityNorthMps.format(),
                record.velocityUpMps.format(),
                record.rollAfterRad.format(),
                record.pitchAfterRad.format(),
                record.yawAfterRad.format(),
                record.abnormalReason,
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

    override fun close() {
        writer.close()
    }

    private fun Double.format(): String =
        if (isFinite()) String.format(Locale.US, "%.12f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_attitude_prediction.csv"

        /**
         * 診断CSVを開く。
         * 入力: セッションディレクトリ
         * 出力: CSV用BufferedWriter
         */
        private fun openWriter(sessionDirectory: File) =
            sessionDirectory.also {
                require(it.isDirectory) {
                    "セッションディレクトリが存在しません: ${it.path}"
                }
            }.resolve(FILE_NAME).bufferedWriter()
    }
}

/**
 * 姿勢予測診断の出力先。
 * 入力: NavigationAttitudePredictionRecord
 * 出力: 実装先へ逐次保存
 */
fun interface NavigationAttitudePredictionSink {
    fun write(record: NavigationAttitudePredictionRecord)

    companion object {
        val NONE = NavigationAttitudePredictionSink { }
    }
}
