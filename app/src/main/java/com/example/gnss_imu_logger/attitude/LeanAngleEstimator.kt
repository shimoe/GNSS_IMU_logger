package com.example.gnss_imu_logger.attitude

import kotlin.math.PI
import kotlin.math.atan2

/**
 * 重力または向心加速度から作ったロール参照を使ってジャイロ積分ドリフトを補正する。
 * 入力: 加速度[m/s^2]、補正後角速度[rad/s]、GNSS速度[m/s]、elapsedRealtimeNs[ns]
 * 出力: 推定ロール、参照ロール、角速度補正量
 */
class LeanAngleEstimator {
    private val history = ArrayDeque<WindowSample>()
    private var filteredAcceleration: BodyVector? = null
    private var filteredRollRateRadps = 0.0
    private var filteredYawRateRadps = 0.0
    private var lastAccelerationNs: Long? = null
    private var lastGyroscopeNs: Long? = null
    private var latestSpeedMps: Double? = null
    private var latestSpeedNs: Long? = null
    private var rawIntegratedRollRad = 0.0
    private var estimatedRollRad = 0.0
    private var previousReferenceRad: Double? = null
    private var unwrappedReferenceRad = 0.0

    @Synchronized
    fun updateAcceleration(timestampNs: Long, deviceValues: FloatArray) {
        val body = BodyFrameTransform.deviceToBody(deviceValues)
        val previousNs = lastAccelerationNs
        lastAccelerationNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) {
            filteredAcceleration = body
            return
        }
        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        val alpha = lowPassAlpha(dtSec)
        val previous = filteredAcceleration ?: body
        filteredAcceleration = BodyVector(
            x = previous.x + alpha * (body.x - previous.x),
            y = previous.y + alpha * (body.y - previous.y),
            z = previous.z + alpha * (body.z - previous.z)
        )
    }

    @Synchronized
    fun updateSpeed(timestampNs: Long, speedMps: Float?) {
        latestSpeedNs = timestampNs
        latestSpeedMps = speedMps?.toDouble()
    }

    @Synchronized
    fun updateGyroscope(
        timestampNs: Long,
        correctedDeviceValues: FloatArray
    ): LeanAngleSnapshot? {
        val bodyRate = BodyFrameTransform.deviceToBody(correctedDeviceValues)
        val previousNs = lastGyroscopeNs
        lastGyroscopeNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) {
            filteredRollRateRadps = bodyRate.x
            filteredYawRateRadps = bodyRate.z
            return null
        }

        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec <= 0.0 || dtSec > MAX_GYROSCOPE_INTERVAL_SEC) {
            history.clear()
            return null
        }

        val alpha = lowPassAlpha(dtSec)
        filteredRollRateRadps += alpha * (bodyRate.x - filteredRollRateRadps)
        filteredYawRateRadps += alpha * (bodyRate.z - filteredYawRateRadps)
        rawIntegratedRollRad += filteredRollRateRadps * dtSec

        val reference = calculateReference(timestampNs) ?: return null
        updateUnwrappedReference(reference.rollRad)
        history.addLast(
            WindowSample(
                timestampNs = timestampNs,
                referenceRollRad = unwrappedReferenceRad,
                rawIntegratedRollRad = rawIntegratedRollRad
            )
        )
        removeOldSamples(timestampNs)

        val oldest = history.firstOrNull()
        val windowSec = oldest?.let { (timestampNs - it.timestampNs) / NS_PER_SEC } ?: 0.0
        val correctionRadps = if (oldest != null && windowSec >= MINIMUM_WINDOW_SEC) {
            val referenceRate = (unwrappedReferenceRad - oldest.referenceRollRad) / windowSec
            val integratedRate = (rawIntegratedRollRad - oldest.rawIntegratedRollRad) / windowSec
            referenceRate - integratedRate
        } else {
            0.0
        }

        val correctedRollRate = filteredRollRateRadps + correctionRadps
        if (history.size == 1) {
            estimatedRollRad = unwrappedReferenceRad
        } else {
            estimatedRollRad += correctedRollRate * dtSec
        }

        return LeanAngleSnapshot(
            elapsedRealtimeNs = timestampNs,
            rollRad = normalizeAngle(estimatedRollRad),
            referenceRollRad = normalizeAngle(unwrappedReferenceRad),
            measuredRollRateRadps = filteredRollRateRadps,
            correctedRollRateRadps = correctedRollRate,
            driftCorrectionRadps = correctionRadps,
            yawRateRadps = filteredYawRateRadps,
            speedMps = reference.speedMps,
            centripetalAccelerationMps2 = reference.centripetalAccelerationMps2,
            referenceSource = reference.source,
            windowDurationMs = (windowSec * MS_PER_SEC).toLong(),
            estimateValid = windowSec >= MINIMUM_WINDOW_SEC
        )
    }

    @Synchronized
    fun reset() {
        history.clear()
        filteredAcceleration = null
        filteredRollRateRadps = 0.0
        filteredYawRateRadps = 0.0
        lastAccelerationNs = null
        lastGyroscopeNs = null
        latestSpeedMps = null
        latestSpeedNs = null
        rawIntegratedRollRad = 0.0
        estimatedRollRad = 0.0
        previousReferenceRad = null
        unwrappedReferenceRad = 0.0
    }

    private fun calculateReference(timestampNs: Long): RollReference? {
        val speed = validSpeed(timestampNs)
        if (speed != null && speed > MOVING_SPEED_MPS) {
            val centripetal = filteredYawRateRadps * speed
            return RollReference(
                rollRad = atan2(centripetal, STANDARD_GRAVITY_MPS2),
                speedMps = speed,
                centripetalAccelerationMps2 = centripetal,
                source = ReferenceSource.CENTRIPETAL
            )
        }
        val acceleration = filteredAcceleration ?: return null
        return RollReference(
            rollRad = atan2(acceleration.y, acceleration.z),
            speedMps = speed,
            centripetalAccelerationMps2 = 0.0,
            source = ReferenceSource.GRAVITY
        )
    }

    private fun validSpeed(timestampNs: Long): Double? {
        val speedNs = latestSpeedNs ?: return null
        if (timestampNs - speedNs > MAX_SPEED_AGE_NS) return null
        return latestSpeedMps
    }

    private fun updateUnwrappedReference(currentRad: Double) {
        val previous = previousReferenceRad
        if (previous == null) {
            unwrappedReferenceRad = currentRad
        } else {
            unwrappedReferenceRad += normalizeAngle(currentRad - previous)
        }
        previousReferenceRad = currentRad
    }

    private fun removeOldSamples(timestampNs: Long) {
        val oldestAllowedNs = timestampNs - WINDOW_RETENTION_NS
        while (history.firstOrNull()?.timestampNs?.let { it < oldestAllowedNs } == true) {
            history.removeFirst()
        }
    }

    private fun lowPassAlpha(dtSec: Double): Double {
        val timeConstantSec = 1.0 / (2.0 * PI * LOW_PASS_CUTOFF_HZ)
        return dtSec / (timeConstantSec + dtSec)
    }

    private fun normalizeAngle(angleRad: Double): Double {
        var value = angleRad
        while (value > PI) value -= 2.0 * PI
        while (value < -PI) value += 2.0 * PI
        return value
    }

    private data class WindowSample(
        val timestampNs: Long,
        val referenceRollRad: Double,
        val rawIntegratedRollRad: Double
    )

    private data class RollReference(
        val rollRad: Double,
        val speedMps: Double?,
        val centripetalAccelerationMps2: Double,
        val source: ReferenceSource
    )

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val LOW_PASS_CUTOFF_HZ = 5.0
        private const val MOVING_SPEED_MPS = 0.5
        private const val MINIMUM_WINDOW_SEC = 1.9
        private const val WINDOW_RETENTION_NS = 2_100_000_000L
        private const val MAX_SPEED_AGE_NS = 1_500_000_000L
        private const val MAX_GYROSCOPE_INTERVAL_SEC = 0.1
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val MS_PER_SEC = 1_000.0
    }
}

enum class ReferenceSource {
    GRAVITY,
    CENTRIPETAL
}

data class LeanAngleSnapshot(
    val elapsedRealtimeNs: Long,
    val rollRad: Double,
    val referenceRollRad: Double,
    val measuredRollRateRadps: Double,
    val correctedRollRateRadps: Double,
    val driftCorrectionRadps: Double,
    val yawRateRadps: Double,
    val speedMps: Double?,
    val centripetalAccelerationMps2: Double,
    val referenceSource: ReferenceSource,
    val windowDurationMs: Long,
    val estimateValid: Boolean
)
