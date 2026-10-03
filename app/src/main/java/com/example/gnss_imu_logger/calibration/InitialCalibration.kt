package com.example.gnss_imu_logger.calibration

import kotlin.math.sqrt

/**
 * 静止成立後の未校正IMUから初期校正値を集計する。
 * 入力: 加速度[m/s^2]、角速度[rad/s]、elapsedRealtimeNs[ns]、静止判定
 * 出力: ジャイロバイアス、重力ベクトル、各軸標準偏差、使用サンプル数
 */
class InitialCalibration {
    private var active = false
    private var completed = false
    private var startedNs: Long? = null
    private var result: CalibrationResult? = null
    private val accelerometer = VectorAccumulator()
    private val gyroscope = VectorAccumulator()
    private var latestTimestampNs = 0L

    @Synchronized
    fun updateStationary(stationary: Boolean, nowNs: Long): CalibrationEvent? {
        if (completed) return null
        if (stationary && !active) {
            active = true
            startedNs = nowNs
            accelerometer.reset()
            gyroscope.reset()
            return CalibrationEvent.STARTED
        }
        if (!stationary && active) {
            active = false
            startedNs = null
            accelerometer.reset()
            gyroscope.reset()
            return CalibrationEvent.INTERRUPTED
        }
        return null
    }

    @Synchronized
    fun addAccelerometer(timestampNs: Long, values: FloatArray): CalibrationEvent? {
        if (!active || completed) return null
        accelerometer.add(values)
        return evaluate(timestampNs)
    }

    @Synchronized
    fun addGyroscope(timestampNs: Long, values: FloatArray): CalibrationEvent? {
        if (!active || completed) return null
        gyroscope.add(values)
        return evaluate(timestampNs)
    }

    @Synchronized
    fun snapshot() = CalibrationProgress(
        active, completed,
        startedNs?.let { (latestTimestampNs - it).coerceAtLeast(0L) / NS_PER_MS } ?: 0L,
        accelerometer.count, gyroscope.count, result
    )

    private fun evaluate(nowNs: Long): CalibrationEvent? {
        latestTimestampNs = maxOf(latestTimestampNs, nowNs)
        val start = startedNs ?: return null
        if (nowNs - start < REQUIRED_DURATION_NS) return null
        if (accelerometer.count < MINIMUM_SAMPLES || gyroscope.count < MINIMUM_SAMPLES) return null
        result = CalibrationResult(
            start, nowNs, accelerometer.count, gyroscope.count,
            accelerometer.mean(), accelerometer.standardDeviation(),
            gyroscope.mean(), gyroscope.standardDeviation()
        )
        completed = true
        active = false
        return CalibrationEvent.COMPLETED
    }

    private class VectorAccumulator {
        var count = 0; private set
        private val sum = DoubleArray(3)
        private val squareSum = DoubleArray(3)
        fun add(values: FloatArray) { for (i in 0..2) { val v=values.getOrElse(i){0f}.toDouble(); sum[i]+=v; squareSum[i]+=v*v }; count++ }
        fun mean()=Vector3(sum[0]/count,sum[1]/count,sum[2]/count)
        fun standardDeviation():Vector3 { val m=mean(); return Vector3(dev(0,m.x),dev(1,m.y),dev(2,m.z)) }
        fun reset(){count=0;sum.fill(0.0);squareSum.fill(0.0)}
        private fun dev(i:Int,m:Double)=sqrt((squareSum[i]/count-m*m).coerceAtLeast(0.0))
    }
    companion object { private const val REQUIRED_DURATION_NS=5_000_000_000L; private const val MINIMUM_SAMPLES=800; private const val NS_PER_MS=1_000_000L }
}
enum class CalibrationEvent { STARTED, INTERRUPTED, COMPLETED }
data class Vector3(val x:Double,val y:Double,val z:Double)
data class CalibrationResult(val startedElapsedNs:Long,val completedElapsedNs:Long,val accelerometerSamples:Int,val gyroscopeSamples:Int,val gravityMps2:Vector3,val gravityStdMps2:Vector3,val gyroscopeBiasRadps:Vector3,val gyroscopeStdRadps:Vector3)
data class CalibrationProgress(val active:Boolean,val completed:Boolean,val elapsedMs:Long,val accelerometerSamples:Int,val gyroscopeSamples:Int,val result:CalibrationResult?)
