package com.example.gnss_imu_logger.log

import android.hardware.Sensor
import java.io.Closeable
import java.io.File

/**
 * 診断モードでバイアス補正後のジャイロを保存する。
 * 入力: elapsedRealtimeNs[ns]、補正後角速度X/Y/Z[rad/s]、精度
 * 出力: gyro_corrected.bin
 */
class CorrectedGyroscopeWriter(
    file: File,
    onError: (String, Throwable?) -> Unit
) : Closeable {
    private val writer = BinarySensorWriter(
        file,
        Sensor.TYPE_GYROSCOPE_UNCALIBRATED,
        onError
    )

    fun offer(
        timestampNs: Long,
        correctedValuesRadps: FloatArray,
        accuracy: Int
    ): Boolean {
        val values = floatArrayOf(
            correctedValuesRadps.getOrElse(0) { 0f },
            correctedValuesRadps.getOrElse(1) { 0f },
            correctedValuesRadps.getOrElse(2) { 0f },
            0f,
            0f,
            0f
        )
        return writer.offer(timestampNs, values, accuracy)
    }

    override fun close() = writer.close()
}
