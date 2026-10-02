package com.example.gnss_imu_logger.sensor

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.example.gnss_imu_logger.log.BinarySensorWriter
import com.example.gnss_imu_logger.log.CsvLogWriter
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import com.example.gnss_imu_logger.model.LogMode
import java.io.Closeable

/**
 * 気圧、地磁気および診断用センサーを取得する。
 * 入力: SensorManager、計測モード、セッション保存先
 * 出力: pressure.bin、magnetic.bin、診断モード用バイナリ、sensor_info.csv
 */
class EnvironmentCollector(
    private val sensorManager: SensorManager,
    private val session: SessionContext,
    private val onFatalError: (String, Throwable?) -> Unit
) : SensorEventListener, Closeable {
    private val writers = mutableMapOf<Int, BinarySensorWriter>()
    private var started = false

    fun start() {
        if (started) return
        writeSensorInfo()
        register(
            Sensor.TYPE_PRESSURE,
            PRESSURE_PERIOD_US,
            "pressure.bin",
            required = false
        )
        register(
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED,
            MAGNETIC_PERIOD_US,
            "magnetic.bin",
            required = false
        )

        if (session.mode == LogMode.DIAGNOSTIC) {
            register(
                Sensor.TYPE_ACCELEROMETER,
                IMU_PERIOD_US,
                "accel_calibrated.bin",
                required = false
            )
            register(
                Sensor.TYPE_GYROSCOPE,
                IMU_PERIOD_US,
                "gyro_calibrated.bin",
                required = false
            )
            register(
                Sensor.TYPE_GAME_ROTATION_VECTOR,
                IMU_PERIOD_US,
                "game_rotation_vector.bin",
                required = false
            )
        }
        started = true
    }

    override fun onSensorChanged(event: SensorEvent) {
        val writer = writers[event.sensor.type] ?: return
        if (!writer.offer(event.timestamp, event.values, event.accuracy)) {
            onFatalError(
                "${event.sensor.name}の書き込み待ちデータが上限に達しました",
                null
            )
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun close() {
        if (!started && writers.isEmpty()) return
        sensorManager.unregisterListener(this)
        writers.values.forEach { it.close() }
        writers.clear()
        started = false
    }

    private fun register(
        type: Int,
        periodUs: Int,
        fileName: String,
        required: Boolean
    ) {
        val sensor = sensorManager.getDefaultSensor(type)
        if (sensor == null) {
            if (required) throw IllegalStateException("センサー種別${type}を利用できません")
            session.event(
                EventLevel.WARNING,
                EventType.SENSOR_UNAVAILABLE,
                "センサー種別${type}は端末に搭載されていません"
            )
            return
        }
        val writer = BinarySensorWriter(session.file(fileName), type, onFatalError)
        val registered = sensorManager.registerListener(
            this,
            sensor,
            periodUs,
            SENSOR_LATENCY_US
        )
        if (!registered) {
            writer.close()
            throw IllegalStateException("${sensor.name}を登録できませんでした")
        }
        writers[type] = writer
        session.event(
            EventLevel.INFO,
            EventType.SENSOR_REGISTERED,
            "${sensor.name}を登録しました"
        )
    }

    private fun writeSensorInfo() {
        val writer = CsvLogWriter(
            session.file("sensor_info.csv"),
            listOf(
                "name", "vendor", "type", "min_delay_us", "max_delay_us",
                "maximum_range", "resolution", "fifo_max", "fifo_reserved",
                "wakeup"
            )
        )
        sensorManager.getSensorList(Sensor.TYPE_ALL).forEach { sensor ->
            writer.write(
                listOf(
                    sensor.name,
                    sensor.vendor,
                    sensor.type,
                    sensor.minDelay,
                    sensor.maxDelay,
                    sensor.maximumRange,
                    sensor.resolution,
                    sensor.fifoMaxEventCount,
                    sensor.fifoReservedEventCount,
                    sensor.isWakeUpSensor
                )
            )
        }
        writer.close()
    }

    companion object {
        private const val IMU_PERIOD_US = 5_000
        private const val PRESSURE_PERIOD_US = 40_000
        private const val MAGNETIC_PERIOD_US = 40_000
        private const val SENSOR_LATENCY_US = 100_000
    }
}
