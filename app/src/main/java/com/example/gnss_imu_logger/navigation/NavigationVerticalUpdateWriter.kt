package com.example.gnss_imu_logger.navigation

import java.io.Closeable
import java.io.File
import java.util.Locale

/** 鉛直位置・速度観測の更新結果を逐次保存する。 */
class NavigationVerticalUpdateWriter(sessionDirectory: File) : Closeable {
    private val writer = sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).also {
        it.appendLine(
            "elapsed_realtime_ns,altitude_source,predicted_up_m,observed_up_m," +
                "position_innovation_m,position_gate_m,position_gain,position_accepted," +
                "predicted_vertical_speed_mps,observed_vertical_speed_mps," +
                "speed_innovation_mps,speed_gate_mps,speed_gain,speed_accepted," +
                "position_std_m,speed_std_mps,rejection_reason"
        )
    }

    fun write(
        observation: NavigationVerticalObservation,
        stateBefore: NavigationState,
        result: NavigationVerticalUpdateResult
    ) {
        writer.appendLine(listOf(
            observation.elapsedRealtimeNs,
            observation.altitudeSource.name,
            stateBefore.positionEnuM.upM.format(),
            observation.relativeUpM.format(),
            result.positionInnovationM.format(),
            result.positionGateM.format(),
            result.positionGain.format(),
            result.positionAccepted,
            stateBefore.velocityEnuMps.upMps.format(),
            observation.verticalSpeedMps.format(),
            result.speedInnovationMps.format(),
            result.speedGateMps.format(),
            result.speedGain.format(),
            result.speedAccepted,
            observation.positionStdM.format(),
            observation.speedStdMps.format(),
            result.rejectionReason
        ).joinToString(","))
    }

    override fun close() = writer.close()
    private fun Double.format() = if (isFinite()) String.format(Locale.US, "%.9f", this) else ""

    companion object {
        private const val FILE_NAME = "navigation_vertical_update.csv"
    }
}
