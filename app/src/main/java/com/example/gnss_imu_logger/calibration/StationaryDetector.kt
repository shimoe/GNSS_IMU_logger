package com.example.gnss_imu_logger.calibration

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * エンジン振動とGNSS速度を考慮して車体の停止状態を判定する。
 * 入力: 加速度[m/s^2]、角速度[rad/s]、GNSS速度[m/s]、elapsedRealtimeNs[ns]
 * 出力: 静止判定、判定継続時間、窓内の代表値
 */
class StationaryDetector {
    private val accelerationSamples = ArrayDeque<ScalarSample>()
    private val gyroscopeSamples = ArrayDeque<ScalarSample>()
    private var stationary = false
    private var stationaryCandidateStartedNs: Long? = null
    private var motionCandidateStartedNs: Long? = null
    private var stationaryStartedNs: Long? = null
    private var latestGnssSpeedMps: Float? = null
    private var latestGnssElapsedNs: Long? = null
    private var lastStatus = StationaryStatus.waiting()

    @Synchronized
    fun updateGnssSpeed(timestampNs: Long, speedMps: Float?) {
        latestGnssElapsedNs = timestampNs
        latestGnssSpeedMps = speedMps
    }

    @Synchronized
    fun addAccelerometer(timestampNs: Long, values: FloatArray): StationaryStatus {
        accelerationSamples.addLast(ScalarSample(timestampNs, values.vectorNorm()))
        removeOldSamples(timestampNs)
        return evaluate(timestampNs)
    }

    @Synchronized
    fun addGyroscope(timestampNs: Long, values: FloatArray): StationaryStatus {
        gyroscopeSamples.addLast(ScalarSample(timestampNs, values.vectorNorm()))
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
        latestGnssSpeedMps = null
        latestGnssElapsedNs = null
        lastStatus = StationaryStatus.waiting()
    }

    private fun evaluate(nowNs: Long): StationaryStatus {
        if (!hasFullWindow(nowNs)) {
            return lastStatus.copy(
                stationary = stationary,
                candidate = stationary,
                durationMs = stationaryDurationMs(nowNs)
            )
        }

        val accelerationMean = accelerationSamples.meanValue()
        val accelerationStd = accelerationSamples.standardDeviation(accelerationMean)
        val gyroscopeRms = gyroscopeSamples.rootMeanSquare()
        val accelerationError = abs(accelerationMean - STANDARD_GRAVITY_MPS2)
        val gnssSpeed = validGnssSpeed(nowNs)
        val gnssStopped = gnssSpeed == null || gnssSpeed <= ENTER_GNSS_SPEED_MPS
        val gnssMoving = gnssSpeed != null && gnssSpeed > EXIT_GNSS_SPEED_MPS

        val stationaryCandidate = gnssStopped &&
            accelerationError <= ENTER_ACCELERATION_ERROR_MPS2 &&
            accelerationStd <= ENTER_ACCELERATION_STD_MPS2 &&
            gyroscopeRms <= ENTER_GYROSCOPE_RMS_RADPS

        val motionCandidate = gnssMoving ||
            accelerationError > EXIT_ACCELERATION_ERROR_MPS2 ||
            accelerationStd > EXIT_ACCELERATION_STD_MPS2 ||
            gyroscopeRms > EXIT_GYROSCOPE_RMS_RADPS

        if (!stationary) {
            motionCandidateStartedNs = null
            if (stationaryCandidate) {
                if (stationaryCandidateStartedNs == null) stationaryCandidateStartedNs = nowNs
                val requiredNs = if (gnssSpeed == null) {
                    ENTER_DURATION_WITHOUT_GNSS_NS
                } else {
                    ENTER_DURATION_WITH_GNSS_NS
                }
                if (nowNs - requireNotNull(stationaryCandidateStartedNs) >= requiredNs) {
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
                if (motionCandidateStartedNs == null) motionCandidateStartedNs = nowNs
                if (nowNs - requireNotNull(motionCandidateStartedNs) >= EXIT_DURATION_NS) {
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
            durationMs = if (stationary) stationaryDurationMs(nowNs) else
                stationaryCandidateStartedNs?.let { (nowNs - it) / NS_PER_MS } ?: 0L,
            accelerationNormMps2 = accelerationMean,
            accelerationErrorMps2 = accelerationError,
            gyroscopeNormRadps = gyroscopeRms
        )
        lastStatus = status
        return status
    }

    private fun validGnssSpeed(nowNs: Long): Float? {
        val timestampNs = latestGnssElapsedNs ?: return null
        if (nowNs - timestampNs > MAX_GNSS_SPEED_AGE_NS) return null
        return latestGnssSpeedMps
    }

    private fun stationaryDurationMs(nowNs: Long): Long =
        stationaryStartedNs?.let { (nowNs - it) / NS_PER_MS } ?: 0L

    private fun hasFullWindow(nowNs: Long): Boolean {
        val firstAccelerationNs = accelerationSamples.firstOrNull()?.timestampNs ?: return false
        val firstGyroscopeNs = gyroscopeSamples.firstOrNull()?.timestampNs ?: return false
        return nowNs - firstAccelerationNs >= EVALUATION_WINDOW_NS &&
            nowNs - firstGyroscopeNs >= EVALUATION_WINDOW_NS &&
            accelerationSamples.size >= MINIMUM_WINDOW_SAMPLES &&
            gyroscopeSamples.size >= MINIMUM_WINDOW_SAMPLES
    }

    private fun removeOldSamples(nowNs: Long) {
        val oldestAllowedNs = nowNs - RETENTION_DURATION_NS
        while (accelerationSamples.firstOrNull()?.timestampNs?.let { it < oldestAllowedNs } == true) {
            accelerationSamples.removeFirst()
        }
        while (gyroscopeSamples.firstOrNull()?.timestampNs?.let { it < oldestAllowedNs } == true) {
            gyroscopeSamples.removeFirst()
        }
    }

    private fun FloatArray.vectorNorm(): Double {
        val x = getOrElse(0) { 0f }.toDouble()
        val y = getOrElse(1) { 0f }.toDouble()
        val z = getOrElse(2) { 0f }.toDouble()
        return sqrt(x * x + y * y + z * z)
    }

    private fun ArrayDeque<ScalarSample>.meanValue(): Double = sumOf { it.value } / size

    private fun ArrayDeque<ScalarSample>.standardDeviation(mean: Double): Double = sqrt(
        sumOf { val difference = it.value - mean; difference * difference } / size
    )

    private fun ArrayDeque<ScalarSample>.rootMeanSquare(): Double =
        sqrt(sumOf { it.value * it.value } / size)

    private data class ScalarSample(val timestampNs: Long, val value: Double)

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val EVALUATION_WINDOW_NS = 500_000_000L
        private const val RETENTION_DURATION_NS = 600_000_000L
        private const val MINIMUM_WINDOW_SAMPLES = 50
        private const val MAX_GNSS_SPEED_AGE_NS = 1_500_000_000L

        private const val ENTER_GNSS_SPEED_MPS = 0.5f
        private const val ENTER_ACCELERATION_ERROR_MPS2 = 0.5
        private const val ENTER_ACCELERATION_STD_MPS2 = 2.0
        private const val ENTER_GYROSCOPE_RMS_RADPS = 0.30
        private const val ENTER_DURATION_WITH_GNSS_NS = 3_000_000_000L
        private const val ENTER_DURATION_WITHOUT_GNSS_NS = 5_000_000_000L

        private const val EXIT_GNSS_SPEED_MPS = 1.0f
        private const val EXIT_ACCELERATION_ERROR_MPS2 = 0.8
        private const val EXIT_ACCELERATION_STD_MPS2 = 3.0
        private const val EXIT_GYROSCOPE_RMS_RADPS = 0.45
        private const val EXIT_DURATION_NS = 500_000_000L
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
        fun waiting() = StationaryStatus(false, false, 0L, null, null, null)
    }
}
