package com.example.gnss_imu_logger.calibration

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * 短時間窓の平均値とばらつきから端末の静止状態を判定する。
 * 入力: 未校正加速度[m/s^2]、未校正角速度[rad/s]、elapsedRealtimeNs[ns]
 * 出力: 静止判定、判定継続時間、窓内の代表値
 */
class StationaryDetector {
    private val accelerationSamples = ArrayDeque<ScalarSample>()
    private val gyroscopeSamples = ArrayDeque<ScalarSample>()
    private var stationaryCandidateStartedNs: Long? = null

    @Synchronized
    fun addAccelerometer(
        timestampNs: Long,
        values: FloatArray
    ): StationaryStatus {
        accelerationSamples.addLast(
            ScalarSample(timestampNs, values.vectorNorm())
        )
        removeOldSamples(timestampNs)
        return evaluate(timestampNs)
    }

    @Synchronized
    fun addGyroscope(
        timestampNs: Long,
        values: FloatArray
    ): StationaryStatus {
        gyroscopeSamples.addLast(
            ScalarSample(timestampNs, values.vectorNorm())
        )
        removeOldSamples(timestampNs)
        return evaluate(timestampNs)
    }

    @Synchronized
    fun reset() {
        accelerationSamples.clear()
        gyroscopeSamples.clear()
        stationaryCandidateStartedNs = null
    }

    private fun evaluate(nowNs: Long): StationaryStatus {
        if (!hasFullWindow(nowNs)) {
            stationaryCandidateStartedNs = null
            return StationaryStatus.waiting()
        }

        val accelerationMean = accelerationSamples.meanValue()
        val accelerationStd = accelerationSamples.standardDeviation(
            accelerationMean
        )
        val gyroscopeRms = gyroscopeSamples.rootMeanSquare()
        val accelerationError = abs(
            accelerationMean - STANDARD_GRAVITY_MPS2
        )

        val candidate =
            accelerationError <= MAX_ACCELERATION_ERROR_MPS2 &&
                accelerationStd <= MAX_ACCELERATION_STD_MPS2 &&
                gyroscopeRms <= MAX_GYROSCOPE_RMS_RADPS

        if (candidate) {
            if (stationaryCandidateStartedNs == null) {
                stationaryCandidateStartedNs = nowNs
            }
        } else {
            stationaryCandidateStartedNs = null
        }

        val durationNs = stationaryCandidateStartedNs?.let {
            nowNs - it
        } ?: 0L

        return StationaryStatus(
            stationary = candidate &&
                durationNs >= REQUIRED_STATIONARY_DURATION_NS,
            candidate = candidate,
            durationMs = durationNs / NS_PER_MS,
            accelerationNormMps2 = accelerationMean,
            accelerationErrorMps2 = accelerationError,
            gyroscopeNormRadps = gyroscopeRms
        )
    }

    private fun hasFullWindow(nowNs: Long): Boolean {
        val firstAccelerationNs = accelerationSamples.firstOrNull()?.timestampNs
            ?: return false
        val firstGyroscopeNs = gyroscopeSamples.firstOrNull()?.timestampNs
            ?: return false

        return nowNs - firstAccelerationNs >= EVALUATION_WINDOW_NS &&
            nowNs - firstGyroscopeNs >= EVALUATION_WINDOW_NS &&
            accelerationSamples.size >= MINIMUM_WINDOW_SAMPLES &&
            gyroscopeSamples.size >= MINIMUM_WINDOW_SAMPLES
    }

    private fun removeOldSamples(nowNs: Long) {
        val oldestAllowedNs = nowNs - RETENTION_DURATION_NS
        while (
            accelerationSamples.firstOrNull()?.timestampNs
                ?.let { it < oldestAllowedNs } == true
        ) {
            accelerationSamples.removeFirst()
        }
        while (
            gyroscopeSamples.firstOrNull()?.timestampNs
                ?.let { it < oldestAllowedNs } == true
        ) {
            gyroscopeSamples.removeFirst()
        }
    }

    private fun FloatArray.vectorNorm(): Double {
        val x = getOrElse(0) { 0f }.toDouble()
        val y = getOrElse(1) { 0f }.toDouble()
        val z = getOrElse(2) { 0f }.toDouble()
        return sqrt(x * x + y * y + z * z)
    }

    private fun ArrayDeque<ScalarSample>.meanValue(): Double =
        sumOf { it.value } / size

    private fun ArrayDeque<ScalarSample>.standardDeviation(
        mean: Double
    ): Double = sqrt(
        sumOf {
            val difference = it.value - mean
            difference * difference
        } / size
    )

    private fun ArrayDeque<ScalarSample>.rootMeanSquare(): Double =
        sqrt(sumOf { it.value * it.value } / size)

    private data class ScalarSample(
        val timestampNs: Long,
        val value: Double
    )

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665

        // 約0.5秒の窓で単発ノイズによる判定解除を防ぐ。
        private const val EVALUATION_WINDOW_NS = 500_000_000L
        private const val RETENTION_DURATION_NS = 600_000_000L
        private const val MINIMUM_WINDOW_SAMPLES = 50

        private const val MAX_ACCELERATION_ERROR_MPS2 = 0.35
        private const val MAX_ACCELERATION_STD_MPS2 = 0.12
        private const val MAX_GYROSCOPE_RMS_RADPS = 0.05

        private const val REQUIRED_STATIONARY_DURATION_NS = 2_000_000_000L
        private const val NS_PER_MS = 1_000_000L
    }
}

data class StationaryStatus(
    val stationary: Boolean,
    val candidate: Boolean,
    val durationMs: Long,
    val accelerationNormMps2: Double?,
    val accelerationErrorMps2: Double?,
    val gyroscopeNormRadps: Double?
) {
    companion object {
        fun waiting() = StationaryStatus(
            stationary = false,
            candidate = false,
            durationMs = 0L,
            accelerationNormMps2 = null,
            accelerationErrorMps2 = null,
            gyroscopeNormRadps = null
        )
    }
}
