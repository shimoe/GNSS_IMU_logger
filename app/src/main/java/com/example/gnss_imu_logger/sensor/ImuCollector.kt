package com.example.gnss_imu_logger.sensor

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.example.gnss_imu_logger.log.BinarySensorWriter
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import java.io.Closeable

/**
 * SOG05の未校正加速度と未校正ジャイロを取得する。
 * 入力: Android SensorManager、計測モード、セッション保存先
 * 出力: accel.bin、gyro.bin、各センサーの受信統計
 */
class ImuCollector(
    private val sensorManager: SensorManager,
    private val session: SessionContext,
    private val onFatalError: (String, Throwable?) -> Unit,
    private val onAccelerometerUpdated: (Long, FloatArray) -> Unit = { _, _ -> },
    private val onGyroscopeUpdated: (Long, FloatArray) -> Unit = { _, _ -> }
) : SensorEventListener, Closeable {
    private val accelStats = SensorStats()
    private val gyroStats = SensorStats()
    private var accelWriter: BinarySensorWriter? = null
    private var gyroWriter: BinarySensorWriter? = null
    private var started = false

    fun start() {
        if (started) return
        val accel = requireSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, "未校正加速度")
        val gyro = requireSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED, "未校正ジャイロ")

        accelWriter = BinarySensorWriter(
            session.file("accel.bin"),
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED,
            onFatalError
        )
        gyroWriter = BinarySensorWriter(
            session.file("gyro.bin"),
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED,
            onFatalError
        )

        register(accel, SENSOR_PERIOD_US)
        register(gyro, SENSOR_PERIOD_US)
        started = true
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED -> {
                accelStats.add(event.timestamp)
                onAccelerometerUpdated(event.timestamp, event.values.copyOf())
                if (accelWriter?.offer(event.timestamp, event.values, event.accuracy) == false) {
                    onFatalError("加速度ログの書き込み待ちデータが上限に達しました", null)
                }
            }
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED -> {
                gyroStats.add(event.timestamp)
                onGyroscopeUpdated(event.timestamp, event.values.copyOf())
                if (gyroWriter?.offer(event.timestamp, event.values, event.accuracy) == false) {
                    onFatalError("ジャイロログの書き込み待ちデータが上限に達しました", null)
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    fun snapshot(): ImuStats = ImuStats(
        accelerometer = accelStats.snapshot(
            accelWriter?.writtenCount() ?: 0L,
            accelWriter?.droppedCount() ?: 0L
        ),
        gyroscope = gyroStats.snapshot(
            gyroWriter?.writtenCount() ?: 0L,
            gyroWriter?.droppedCount() ?: 0L
        )
    )

    override fun close() {
        sensorManager.unregisterListener(this)
        accelWriter?.close()
        gyroWriter?.close()
        accelWriter = null
        gyroWriter = null
        started = false
    }

    private fun requireSensor(type: Int, displayName: String): Sensor =
        sensorManager.getDefaultSensor(type)
            ?: throw IllegalStateException("${displayName}センサーを利用できません")

    private fun register(sensor: Sensor, periodUs: Int) {
        val registered = sensorManager.registerListener(
            this,
            sensor,
            periodUs,
            SENSOR_LATENCY_US
        )
        if (!registered) {
            throw IllegalStateException("${sensor.name}を登録できませんでした")
        }
        session.event(
            EventLevel.INFO,
            EventType.SENSOR_REGISTERED,
            "${sensor.name}を登録しました"
        )
    }

    data class ImuStats(
        val accelerometer: SensorStats.Snapshot,
        val gyroscope: SensorStats.Snapshot
    )

    companion object {
        private const val SENSOR_PERIOD_US = 5_000
        private const val SENSOR_LATENCY_US = 100_000
    }
}
