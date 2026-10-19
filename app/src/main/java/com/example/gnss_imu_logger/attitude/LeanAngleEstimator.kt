package com.example.gnss_imu_logger.attitude

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 単車のロール角を、車体前後軸角速度の積分と時間窓内の角速度補正から推定する。
 * 入力: 加速度[m/s^2]、補正後角速度[rad/s]、GNSS速度[m/s]、elapsedRealtimeNs[ns]、静止判定
 * 出力: 推定ロール、参照ロール、参照有効状態、角速度補正量
 */
class LeanAngleEstimator(
    private val mountTransform: DeviceMountTransform,
    val parameters: LeanAngleParameters = LeanAngleParameters.DEFAULT
) {
    private val history = ArrayDeque<WindowSample>()
    private var filteredAcceleration: BodyVector? = null
    private var filteredRollRateRadps = 0.0
    private var filteredYawRateRadps = 0.0
    private var filteredYawAccelerationRadps2 = 0.0
    private var lastAccelerationNs: Long? = null
    private var lastGyroscopeNs: Long? = null
    private var latestSpeedMps: Double? = null
    private var latestSpeedNs: Long? = null
    private var speedAccelerationMps2: Double? = null
    private var rawIntegratedRollRad = 0.0
    private var estimatedRollRad = 0.0
    private var previousReferenceRad: Double? = null
    private var unwrappedReferenceRad = 0.0
    private var driftCorrectionRadps = 0.0
    private var referenceRateRadps = 0.0
    private var integratedRateRadps = 0.0
    private var lastReferenceSource = ReferenceSource.HOLD
    private var activeReferenceSource = ReferenceSource.HOLD
    private var entryCandidateStartedNs: Long? = null
    private var exitCandidateStartedNs: Long? = null
    private var holdStartedNs: Long? = null
    private var sourceStartedNs: Long? = null
    private var angleRecoveryRateRadps = 0.0
    private var initialized = false
    private var stationary = false
    private val statistics = LeanAngleStatistics()

    @Synchronized
    fun updateAcceleration(timestampNs: Long, deviceValues: FloatArray) {
        val body = mountTransform.deviceToBody(deviceValues)
        val previousNs = lastAccelerationNs
        lastAccelerationNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) {
            filteredAcceleration = body
            return
        }
        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec <= 0.0 || dtSec > parameters.maximumAccelerationIntervalSec) return
        val alpha = lowPassAlpha(dtSec)
        val previous = filteredAcceleration ?: body
        filteredAcceleration = BodyVector(
            x = previous.x + alpha * (body.x - previous.x),
            y = previous.y + alpha * (body.y - previous.y),
            z = previous.z + alpha * (body.z - previous.z)
        )
    }

    /**
     * GNSS速度と速度変化量を更新する。
     * 入力: elapsedRealtimeNs[ns]、GNSS速度[m/s]
     * 出力: なし
     */
    @Synchronized
    fun updateSpeed(timestampNs: Long, speedMps: Float?) {
        val speed = speedMps?.toDouble()?.takeIf { it.isFinite() && it >= 0.0 }
        val oldSpeed = latestSpeedMps
        val oldNs = latestSpeedNs
        latestSpeedNs = timestampNs
        latestSpeedMps = speed

        speedAccelerationMps2 = if (
            speed != null && oldSpeed != null && oldNs != null && timestampNs > oldNs
        ) {
            val dtSec = (timestampNs - oldNs) / NS_PER_SEC
            if (dtSec in parameters.minimumSpeedIntervalSec..parameters.maximumSpeedIntervalSec) {
                (speed - oldSpeed) / dtSec
            } else {
                null
            }
        } else {
            null
        }
    }

    /** 静止中だけ重力参照を有効にする。 */
    @Synchronized
    fun updateStationary(stationary: Boolean) {
        this.stationary = stationary
    }

    @Synchronized
    fun updateGyroscope(
        timestampNs: Long,
        correctedDeviceValues: FloatArray
    ): LeanAngleSnapshot? {
        val bodyRate = mountTransform.deviceToBody(correctedDeviceValues)
        val previousNs = lastGyroscopeNs
        lastGyroscopeNs = timestampNs
        if (previousNs == null || timestampNs <= previousNs) {
            filteredRollRateRadps = bodyRate.x
            filteredYawRateRadps = bodyRate.z
            return null
        }

        val dtSec = (timestampNs - previousNs) / NS_PER_SEC
        if (dtSec <= 0.0 || dtSec > parameters.maximumGyroscopeIntervalSec) {
            clearReferenceWindow()
            return null
        }

        val previousYawRate = filteredYawRateRadps
        val alpha = lowPassAlpha(dtSec)
        filteredRollRateRadps += alpha * (bodyRate.x - filteredRollRateRadps)
        filteredYawRateRadps += alpha * (bodyRate.z - filteredYawRateRadps)
        val yawAcceleration = (filteredYawRateRadps - previousYawRate) / dtSec
        filteredYawAccelerationRadps2 += parameters.dynamicsFilterAlpha *
            (yawAcceleration - filteredYawAccelerationRadps2)
        rawIntegratedRollRad += filteredRollRateRadps * dtSec

        val reference = calculateReference(timestampNs)
        val windowSec = if (reference.source != ReferenceSource.HOLD) {
            updateReferenceWindow(timestampNs, reference)
        } else {
            holdCorrection(timestampNs, dtSec)
            0.0
        }

        angleRecoveryRateRadps = 0.0
        if (!initialized && reference.source != ReferenceSource.HOLD) {
            estimatedRollRad = reference.rollRad
            initialized = true
        } else {
            if (reference.source == ReferenceSource.GRAVITY && initialized) {
                val errorRad = normalizeAngle(reference.rollRad - normalizeAngle(estimatedRollRad))
                angleRecoveryRateRadps = (errorRad / parameters.gravityRecoveryTimeSec).coerceIn(
                    -parameters.maximumGravityRecoveryRateRadps,
                    parameters.maximumGravityRecoveryRateRadps
                )
            }
            estimatedRollRad += (filteredRollRateRadps + driftCorrectionRadps + angleRecoveryRateRadps) * dtSec
        }

        val snapshot = LeanAngleSnapshot(
            elapsedRealtimeNs = timestampNs,
            rollRad = normalizeAngle(estimatedRollRad),
            referenceRollRad = normalizeAngle(unwrappedReferenceRad),
            measuredRollRateRadps = filteredRollRateRadps,
            correctedRollRateRadps = filteredRollRateRadps + driftCorrectionRadps + angleRecoveryRateRadps,
            driftCorrectionRadps = driftCorrectionRadps,
            angleRecoveryRateRadps = angleRecoveryRateRadps,
            referenceRateRadps = referenceRateRadps,
            integratedRateRadps = integratedRateRadps,
            yawRateRadps = filteredYawRateRadps,
            yawAccelerationRadps2 = filteredYawAccelerationRadps2,
            speedMps = reference.speedMps,
            speedAccelerationMps2 = speedAccelerationMps2,
            centripetalAccelerationMps2 = reference.centripetalAccelerationMps2,
            referenceSource = reference.source,
            referenceValid = reference.source != ReferenceSource.HOLD,
            referenceConfidence = reference.confidence,
            referenceStateDurationMs = sourceStartedNs?.let { (timestampNs - it) / 1_000_000L } ?: 0L,
            windowDurationMs = (windowSec * MS_PER_SEC).toLong(),
            estimateValid = initialized && (
                reference.source == ReferenceSource.HOLD || windowSec >= parameters.minimumWindowSec
            )
        )
        statistics.record(snapshot)
        return snapshot
    }

    /**
     * セッション終了時に参照状態と補正量の統計を返す。
     * 入力: なし
     * 出力: LeanAngleSummary
     */
    @Synchronized
    fun summary(): LeanAngleSummary = statistics.summary()

    @Synchronized
    fun reset() {
        history.clear()
        filteredAcceleration = null
        filteredRollRateRadps = 0.0
        filteredYawRateRadps = 0.0
        filteredYawAccelerationRadps2 = 0.0
        lastAccelerationNs = null
        lastGyroscopeNs = null
        latestSpeedMps = null
        latestSpeedNs = null
        speedAccelerationMps2 = null
        rawIntegratedRollRad = 0.0
        estimatedRollRad = 0.0
        previousReferenceRad = null
        unwrappedReferenceRad = 0.0
        driftCorrectionRadps = 0.0
        referenceRateRadps = 0.0
        integratedRateRadps = 0.0
        lastReferenceSource = ReferenceSource.HOLD
        activeReferenceSource = ReferenceSource.HOLD
        entryCandidateStartedNs = null
        exitCandidateStartedNs = null
        holdStartedNs = null
        sourceStartedNs = null
        angleRecoveryRateRadps = 0.0
        initialized = false
        stationary = false
        statistics.reset()
    }

    private fun calculateReference(timestampNs: Long): RollReference {
        val speed = validSpeed(timestampNs)
        val acceleration = filteredAcceleration
        if (stationary && acceleration != null) {
            val confidence = gravityReferenceConfidence(acceleration)
            if (confidence > 0.0) {
                setSource(ReferenceSource.GRAVITY, timestampNs)
                return RollReference(atan2(acceleration.y, acceleration.z), speed, 0.0, ReferenceSource.GRAVITY, confidence)
            }
        }
        val confidence = centripetalReferenceConfidence(speed)
        updateCentripetalState(timestampNs, speed, confidence)
        if (activeReferenceSource == ReferenceSource.CENTRIPETAL && speed != null) {
            val centripetal = filteredYawRateRadps * speed
            return RollReference(atan2(centripetal, STANDARD_GRAVITY_MPS2), speed, centripetal, ReferenceSource.CENTRIPETAL, confidence)
        }
        setSource(ReferenceSource.HOLD, timestampNs)
        return RollReference(normalizeAngle(unwrappedReferenceRad), speed, 0.0, ReferenceSource.HOLD, 0.0)
    }

    /** 速度と信頼度へ時間ヒステリシスを適用して定常旋回状態を更新する。 */
    private fun updateCentripetalState(timestampNs: Long, speedMps: Double?, confidence: Double) {
        if (activeReferenceSource == ReferenceSource.GRAVITY && !stationary) setSource(ReferenceSource.HOLD, timestampNs)
        if (activeReferenceSource == ReferenceSource.CENTRIPETAL) {
            val maintain = speedMps != null && speedMps >= parameters.centripetalMaintainSpeedMps &&
                confidence >= parameters.centripetalMaintainConfidence
            if (maintain) { exitCandidateStartedNs = null; return }
            val startedNs = exitCandidateStartedNs ?: timestampNs.also { exitCandidateStartedNs = it }
            if ((timestampNs - startedNs) / NS_PER_SEC >= parameters.centripetalExitDurationSec) {
                setSource(ReferenceSource.HOLD, timestampNs)
                exitCandidateStartedNs = null
            }
            return
        }
        val enter = speedMps != null && speedMps >= parameters.centripetalEntrySpeedMps &&
            confidence >= parameters.centripetalEntryConfidence
        if (!enter) { entryCandidateStartedNs = null; return }
        val startedNs = entryCandidateStartedNs ?: timestampNs.also { entryCandidateStartedNs = it }
        if ((timestampNs - startedNs) / NS_PER_SEC >= parameters.centripetalEntryDurationSec) {
            setSource(ReferenceSource.CENTRIPETAL, timestampNs)
            entryCandidateStartedNs = null
        }
    }

    private fun setSource(source: ReferenceSource, timestampNs: Long) {
        if (activeReferenceSource != source || sourceStartedNs == null) sourceStartedNs = timestampNs
        activeReferenceSource = source
        if (source != ReferenceSource.HOLD) holdStartedNs = null
    }

    /** 定常旋回の仮定を満たす範囲から向心加速度参照の信頼度を求める。 */
    private fun centripetalReferenceConfidence(speedMps: Double?): Double {
        if (speedMps == null || speedMps < parameters.centripetalMaintainSpeedMps) return 0.0
        val speedChange = abs(speedAccelerationMps2 ?: return 0.0)
        val speedConfidence = linearConfidence(
            speedChange,
            parameters.fullSpeedAccelerationMps2,
            parameters.maximumSpeedAccelerationMps2
        )
        val yawAccelerationConfidence = linearConfidence(
            abs(filteredYawAccelerationRadps2),
            parameters.fullYawAccelerationRadps2,
            parameters.maximumYawAccelerationRadps2
        )
        val rollRateConfidence = linearConfidence(
            abs(filteredRollRateRadps),
            parameters.fullRollRateRadps,
            parameters.maximumRollRateRadps
        )
        return speedConfidence * yawAccelerationConfidence * rollRateConfidence
    }

    private fun gravityReferenceConfidence(acceleration: BodyVector): Double {
        val norm = sqrt(
            acceleration.x * acceleration.x +
                acceleration.y * acceleration.y +
                acceleration.z * acceleration.z
        )
        val error = abs(norm - STANDARD_GRAVITY_MPS2)
        return (1.0 - error / parameters.maximumGravityErrorMps2).coerceIn(0.0, 1.0)
    }

    private fun updateReferenceWindow(timestampNs: Long, reference: RollReference): Double {
        holdStartedNs = null
        if (reference.source != lastReferenceSource && lastReferenceSource != ReferenceSource.HOLD) {
            clearReferenceWindow()
            previousReferenceRad = null
            lastReferenceSource = reference.source
        }
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
        if (oldest != null && windowSec >= parameters.minimumWindowSec) {
            referenceRateRadps =
                (unwrappedReferenceRad - oldest.referenceRollRad) / windowSec
            integratedRateRadps =
                (rawIntegratedRollRad - oldest.rawIntegratedRollRad) / windowSec
            val targetCorrection = reference.confidence * (referenceRateRadps - integratedRateRadps)
            driftCorrectionRadps += parameters.correctionFilterAlpha *
                (targetCorrection - driftCorrectionRadps)
        }
        return windowSec
    }

    private fun holdCorrection(timestampNs: Long, dtSec: Double) {
        val startedNs = holdStartedNs ?: timestampNs.also { holdStartedNs = it }
        if ((timestampNs - startedNs) / NS_PER_SEC > parameters.shortHoldDurationSec) {
            clearReferenceWindow()
            previousReferenceRad = null
            lastReferenceSource = ReferenceSource.HOLD
        }
        referenceRateRadps = 0.0
        integratedRateRadps = 0.0
        val decay = (dtSec / parameters.holdCorrectionDecaySec).coerceIn(0.0, 1.0)
        driftCorrectionRadps += decay * (0.0 - driftCorrectionRadps)
    }

    private fun validSpeed(timestampNs: Long): Double? {
        val speedNs = latestSpeedNs ?: return null
        if (timestampNs - speedNs > parameters.maximumSpeedAgeNs) return null
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

    private fun clearReferenceWindow() {
        history.clear()
    }

    private fun removeOldSamples(timestampNs: Long) {
        val oldestAllowedNs = timestampNs - parameters.windowRetentionNs
        while (history.firstOrNull()?.timestampNs?.let { it < oldestAllowedNs } == true) {
            history.removeFirst()
        }
    }

    private fun lowPassAlpha(dtSec: Double): Double {
        val timeConstantSec = 1.0 / (2.0 * PI * parameters.lowPassCutoffHz)
        return dtSec / (timeConstantSec + dtSec)
    }

    private fun linearConfidence(value: Double, fullAt: Double, zeroAt: Double): Double {
        if (!value.isFinite() || zeroAt <= fullAt) return 0.0
        return ((zeroAt - value) / (zeroAt - fullAt)).coerceIn(0.0, 1.0)
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
        val source: ReferenceSource,
        val confidence: Double
    )

    companion object {
        private const val STANDARD_GRAVITY_MPS2 = 9.80665
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val MS_PER_SEC = 1_000.0
    }
}

enum class ReferenceSource {
    GRAVITY,
    CENTRIPETAL,
    HOLD
}

data class LeanAngleSnapshot(
    val elapsedRealtimeNs: Long,
    val rollRad: Double,
    val referenceRollRad: Double,
    val measuredRollRateRadps: Double,
    val correctedRollRateRadps: Double,
    val driftCorrectionRadps: Double,
    val angleRecoveryRateRadps: Double,
    val referenceRateRadps: Double,
    val integratedRateRadps: Double,
    val yawRateRadps: Double,
    val yawAccelerationRadps2: Double,
    val speedMps: Double?,
    val speedAccelerationMps2: Double?,
    val centripetalAccelerationMps2: Double,
    val referenceSource: ReferenceSource,
    val referenceValid: Boolean,
    val referenceConfidence: Double,
    val referenceStateDurationMs: Long,
    val windowDurationMs: Long,
    val estimateValid: Boolean
)


/**
 * 単車ロール推定の参照状態と角速度補正量のセッション統計。
 * 入力: LeanAngleSnapshot
 * 出力: session.jsonのstatistics.lean_angle
 */
data class LeanAngleSummary(
    val totalCount: Long,
    val validCount: Long,
    val gravityCount: Long,
    val centripetalCount: Long,
    val holdCount: Long,
    val gravityDurationMs: Long,
    val centripetalDurationMs: Long,
    val holdDurationMs: Long,
    val referenceValidRatio: Double,
    val meanReferenceConfidence: Double,
    val maximumAbsoluteDriftCorrectionRadps: Double,
    val driftCorrectionRmsRadps: Double
)

private class LeanAngleStatistics {
    private var totalCount = 0L
    private var validCount = 0L
    private var gravityCount = 0L
    private var centripetalCount = 0L
    private var holdCount = 0L
    private var gravityDurationNs = 0L
    private var centripetalDurationNs = 0L
    private var holdDurationNs = 0L
    private var referenceConfidenceSum = 0.0
    private var absoluteDriftCorrectionMaximumRadps = 0.0
    private var driftCorrectionSquareSum = 0.0
    private var previousTimestampNs: Long? = null
    private var previousSource: ReferenceSource? = null

    fun record(value: LeanAngleSnapshot) {
        val previousNs = previousTimestampNs
        val source = previousSource
        if (previousNs != null && source != null && value.elapsedRealtimeNs > previousNs) {
            val durationNs = value.elapsedRealtimeNs - previousNs
            if (durationNs <= MAXIMUM_STATISTICS_INTERVAL_NS) when (source) {
                ReferenceSource.GRAVITY -> gravityDurationNs += durationNs
                ReferenceSource.CENTRIPETAL -> centripetalDurationNs += durationNs
                ReferenceSource.HOLD -> holdDurationNs += durationNs
            }
        }
        totalCount += 1
        if (value.referenceValid) validCount += 1
        when (value.referenceSource) {
            ReferenceSource.GRAVITY -> gravityCount += 1
            ReferenceSource.CENTRIPETAL -> centripetalCount += 1
            ReferenceSource.HOLD -> holdCount += 1
        }
        referenceConfidenceSum += value.referenceConfidence
        val absoluteCorrection = kotlin.math.abs(value.driftCorrectionRadps)
        absoluteDriftCorrectionMaximumRadps = maxOf(
            absoluteDriftCorrectionMaximumRadps,
            absoluteCorrection
        )
        driftCorrectionSquareSum += value.driftCorrectionRadps * value.driftCorrectionRadps
        previousTimestampNs = value.elapsedRealtimeNs
        previousSource = value.referenceSource
    }

    fun summary(): LeanAngleSummary {
        val count = totalCount.coerceAtLeast(1L).toDouble()
        return LeanAngleSummary(
            totalCount = totalCount,
            validCount = validCount,
            gravityCount = gravityCount,
            centripetalCount = centripetalCount,
            holdCount = holdCount,
            gravityDurationMs = gravityDurationNs / NS_PER_MS,
            centripetalDurationMs = centripetalDurationNs / NS_PER_MS,
            holdDurationMs = holdDurationNs / NS_PER_MS,
            referenceValidRatio = if (totalCount > 0) validCount.toDouble() / totalCount else 0.0,
            meanReferenceConfidence = if (totalCount > 0) referenceConfidenceSum / totalCount else 0.0,
            maximumAbsoluteDriftCorrectionRadps = absoluteDriftCorrectionMaximumRadps,
            driftCorrectionRmsRadps = if (totalCount > 0) {
                kotlin.math.sqrt(driftCorrectionSquareSum / count)
            } else {
                0.0
            }
        )
    }

    fun reset() {
        totalCount = 0L
        validCount = 0L
        gravityCount = 0L
        centripetalCount = 0L
        holdCount = 0L
        gravityDurationNs = 0L
        centripetalDurationNs = 0L
        holdDurationNs = 0L
        referenceConfidenceSum = 0.0
        absoluteDriftCorrectionMaximumRadps = 0.0
        driftCorrectionSquareSum = 0.0
        previousTimestampNs = null
        previousSource = null
    }

    companion object {
        private const val NS_PER_MS = 1_000_000L
        private const val MAXIMUM_STATISTICS_INTERVAL_NS = 100_000_000L
    }
}
