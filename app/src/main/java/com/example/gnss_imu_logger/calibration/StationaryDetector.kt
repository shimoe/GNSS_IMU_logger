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

    private var stationary = false
    private var stationaryCandidateStartedNs: Long? = null
    private var motionCandidateStartedNs: Long? = null
    private var stationaryStartedNs: Long? = null
    private var lastStatus = StationaryStatus.waiting()

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
        stationary = false
        stationaryCandidateStartedNs = null
        motionCandidateStartedNs = null
        stationaryStartedNs = null
        lastStatus = StationaryStatus.waiting()
    }

    private fun evaluate(nowNs: Long): StationaryStatus {
        if (!hasFullWindow(nowNs)) {
            // 一時的な窓不足だけでは、成立済みの静止状態を解除しない。
            return lastStatus.copy(
                stationary = stationary,
                candidate = stationary,
                durationMs = stationaryDurationMs(nowNs)
            )
        }

        val accelerationMean = accelerationSamples.meanValue()
        val accelerationStd = accelerationSamples.standardDeviation(
            accelerationMean
        )
        val gyroscopeRms = gyroscopeSamples.rootMeanSquare()
        val accelerationError = abs(
            accelerationMean - STANDARD_GRAVITY_MPS2
        )

        val stationaryCandidate =
            accelerationError <= ENTER_ACCELERATION_ERROR_MPS2 &&
                accelerationStd <= ENTER_ACCELERATION_STD_MPS2 &&
                gyroscopeRms <= ENTER_GYROSCOPE_RMS_RADPS

        val motionCandidate =
            accelerationError > EXIT_ACCELERATION_ERROR_MPS2 ||
                accelerationStd > EXIT_ACCELERATION_STD_MPS2 ||
                gyroscopeRms > EXIT_GYROSCOPE_RMS_RADPS

        if (!stationary) {
            motionCandidateStartedNs = null
            if (stationaryCandidate) {
                if (stationaryCandidateStartedNs == null) {
                    stationaryCandidateStartedNs = nowNs
                }
                if (
                    nowNs - requireNotNull(stationaryCandidateStartedNs) >=
                    ENTER_DURATION_NS
                ) {
                    stationary = true
                    stationaryStartedNs = nowNs
                    stationaryCandidateStartedNs = null
                }
            } else {
                stationaryCandidateStartedNs = null
            }
        } else {
            stationaryCandidateStartedNs = null
            if (motionCandidate) {
                if (motionCandidateStartedNs == null) {
                    motionCandidateStartedNs = nowNs
                }
                if (
                    nowNs - requireNotNull(motionCandidateStartedNs) >=
                    EXIT_DURATION_NS
                ) {
                    stationary = false
                    stationaryStartedNs = null
                    motionCandidateStartedNs = null
                }
            } else {
                motionCandidateStartedNs = null
            }
        }

        val status = StationaryStatus(
            stationary = stationary,
            candidate = stationary || stationaryCandidate,
            durationMs = if (stationary) {
                stationaryDurationMs(nowNs)
            } else {
                stationaryCandidateStartedNs?.let {
                    (nowNs - it) / NS_PER_MS
                } ?: 0L
            },
            accelerationNormMps2 = accelerationMean,
            accelerationErrorMps2 = accelerationError,
            gyroscopeNormRadps = gyroscopeRms
        )
        lastStatus = status
        return status
    }

    private fun stationaryDurationMs(nowNs: Long): Long =
        stationaryStartedNs?.let { (nowNs - it) / NS_PER_MS } ?: 0L

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

        private const val EVALUATION_WINDOW_NS = 500_000_000L
        private const val RETENTION_DURATION_NS = 600_000_000L
        private const val MINIMUM_WINDOW_SAMPLES = 50

        // 静止成立条件
        private const val ENTER_ACCELERATION_ERROR_MPS2 = 0.35
        private const val ENTER_ACCELERATION_STD_MPS2 = 0.12
        private const val ENTER_GYROSCOPE_RMS_RADPS = 0.05
        private const val ENTER_DURATION_NS = 2_000_000_000L

        // 静止解除条件。成立条件より広い範囲を許容して判定の往復を防ぐ。
        private const val EXIT_ACCELERATION_ERROR_MPS2 = 0.60
        private const val EXIT_ACCELERATION_STD_MPS2 = 0.20
        private const val EXIT_GYROSCOPE_RMS_RADPS = 0.08
        private const val EXIT_DURATION_NS = 300_000_000L

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
