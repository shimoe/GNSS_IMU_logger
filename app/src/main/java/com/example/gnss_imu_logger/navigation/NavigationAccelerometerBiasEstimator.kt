package com.example.gnss_imu_logger.navigation

import kotlin.math.abs

/** 車体前方・左方向の速度innovationをTheil-Sen法で直接回帰する。 */
class NavigationAccelerometerBiasEstimator(
    private val parameters: NavigationAccelerometerBiasWindowParameters =
        NavigationAccelerometerBiasWindowParameters.DEFAULT
) {
    fun estimate(snapshot: NavigationBiasWindowSnapshot): NavigationAccelerometerBiasEstimate? {
        val samples = snapshot.samples
        if (samples.size < parameters.minimumEstimateSampleCount) return null
        val durationNs = samples.last().elapsedRealtimeNs - samples.first().elapsedRealtimeNs
        if (durationNs < parameters.minimumEstimateDurationNs) return null
        val east = robustLine(samples) { it.velocityInnovationEastMps }
        val north = robustLine(samples) { it.velocityInnovationNorthMps }
        val forward = robustLine(samples) { it.velocityInnovationBodyForwardMps }
        val left = robustLine(samples) { it.velocityInnovationBodyLeftMps }
        val yawChange = wrapAngle(samples.last().yawRad - samples.first().yawRad)
        return NavigationAccelerometerBiasEstimate(
            windowSequence = snapshot.sequence,
            windowStartedElapsedRealtimeNs = snapshot.startedElapsedRealtimeNs,
            windowLastSampleElapsedRealtimeNs = snapshot.lastSampleElapsedRealtimeNs,
            windowCompletedElapsedRealtimeNs = snapshot.completedElapsedRealtimeNs,
            windowEndReason = snapshot.endReason,
            sampleCount = samples.size,
            durationSec = durationNs / NS_PER_SEC,
            yawChangeRad = yawChange,
            eastSlopeMps2 = east.slopePerSec,
            northSlopeMps2 = north.slopePerSec,
            eastResidualMadMps = east.residualMad,
            northResidualMadMps = north.residualMad,
            bodyForwardInnovationFirstMps = samples.first().velocityInnovationBodyForwardMps,
            bodyForwardInnovationLastMps = samples.last().velocityInnovationBodyForwardMps,
            bodyLeftInnovationFirstMps = samples.first().velocityInnovationBodyLeftMps,
            bodyLeftInnovationLastMps = samples.last().velocityInnovationBodyLeftMps,
            bodyForwardSlopeMps2 = forward.slopePerSec,
            bodyLeftSlopeMps2 = left.slopePerSec,
            bodyForwardResidualMadMps = forward.residualMad,
            bodyLeftResidualMadMps = left.residualMad,
            forwardSignRatio = signRatio(samples.map { it.velocityInnovationBodyForwardMps }),
            leftSignRatio = signRatio(samples.map { it.velocityInnovationBodyLeftMps })
        )
    }

    private fun robustLine(
        samples: List<NavigationAccelerometerBiasSample>,
        value: (NavigationAccelerometerBiasSample) -> Double
    ): RobustLine {
        val originNs = samples.first().elapsedRealtimeNs
        val slopes = mutableListOf<Double>()
        for (i in samples.indices) for (j in i + 1 until samples.size) {
            val dtSec = (samples[j].elapsedRealtimeNs - samples[i].elapsedRealtimeNs) / NS_PER_SEC
            if (dtSec > 0.0) slopes += (value(samples[j]) - value(samples[i])) / dtSec
        }
        val slope = slopes.median()
        val intercept = samples.map {
            value(it) - slope * ((it.elapsedRealtimeNs - originNs) / NS_PER_SEC)
        }.median()
        val residualMad = samples.map {
            val timeSec = (it.elapsedRealtimeNs - originNs) / NS_PER_SEC
            abs(value(it) - (intercept + slope * timeSec))
        }.median()
        return RobustLine(slope, residualMad)
    }

    private fun signRatio(values: List<Double>): Double {
        val positive = values.count { it > 0.0 }
        val negative = values.count { it < 0.0 }
        return maxOf(positive, negative).toDouble() / values.size.coerceAtLeast(1)
    }

    private fun List<Double>.median(): Double {
        if (isEmpty()) return Double.NaN
        val sorted = sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) * 0.5
        else sorted[middle]
    }

    private fun wrapAngle(value: Double): Double = kotlin.math.atan2(
        kotlin.math.sin(value), kotlin.math.cos(value)
    )

    private data class RobustLine(val slopePerSec: Double, val residualMad: Double)
    companion object { private const val NS_PER_SEC = 1_000_000_000.0 }
}

data class NavigationAccelerometerBiasWindowParameters(
    val minimumEstimateSampleCount: Int,
    val minimumUpdateSampleCount: Int,
    val minimumEstimateDurationNs: Long,
    val minimumUpdateDurationNs: Long,
    val maximumDurationNs: Long,
    val maximumValidSampleGapNs: Long,
    val maximumHoldNs: Long,
    val maximumEstimateYawChangeRad: Double,
    val maximumForwardYawChangeRad: Double,
    val maximumLeftYawChangeRad: Double,
    val maximumForwardResidualMadMps: Double,
    val maximumLeftResidualMadMps: Double,
    val minimumForwardSlopeMps2: Double,
    val maximumForwardSlopeMps2: Double,
    val minimumLeftSlopeMps2: Double,
    val maximumLeftSlopeMps2: Double,
    val minimumForwardSignRatio: Double,
    val minimumLeftSignRatio: Double,
    val maximumAbsoluteBiasMps2: Double,
    val maximumCorrectionPerUpdateMps2: Double,
    val applyBiasUpdate: Boolean
) {
    companion object {
        val DEFAULT = NavigationAccelerometerBiasWindowParameters(
            minimumEstimateSampleCount = 4,
            minimumUpdateSampleCount = 5,
            minimumEstimateDurationNs = 3_000_000_000L,
            minimumUpdateDurationNs = 4_000_000_000L,
            maximumDurationNs = 20_000_000_000L,
            maximumValidSampleGapNs = 3_000_000_000L,
            maximumHoldNs = 5_000_000_000L,
            maximumEstimateYawChangeRad = Math.toRadians(15.0),
            maximumForwardYawChangeRad = Math.toRadians(12.0),
            maximumLeftYawChangeRad = Math.toRadians(7.0),
            maximumForwardResidualMadMps = 1.0,
            maximumLeftResidualMadMps = 0.7,
            minimumForwardSlopeMps2 = 0.05,
            maximumForwardSlopeMps2 = 1.0,
            minimumLeftSlopeMps2 = 0.05,
            maximumLeftSlopeMps2 = 0.7,
            minimumForwardSignRatio = 0.70,
            minimumLeftSignRatio = 0.75,
            maximumAbsoluteBiasMps2 = 0.30,
            maximumCorrectionPerUpdateMps2 = 0.02,
            applyBiasUpdate = false
        )
    }
}

data class NavigationAccelerometerBiasEstimate(
    val windowSequence: Long,
    val windowStartedElapsedRealtimeNs: Long,
    val windowLastSampleElapsedRealtimeNs: Long,
    val windowCompletedElapsedRealtimeNs: Long,
    val windowEndReason: String,
    val sampleCount: Int,
    val durationSec: Double,
    val yawChangeRad: Double,
    val eastSlopeMps2: Double,
    val northSlopeMps2: Double,
    val eastResidualMadMps: Double,
    val northResidualMadMps: Double,
    val bodyForwardInnovationFirstMps: Double,
    val bodyForwardInnovationLastMps: Double,
    val bodyLeftInnovationFirstMps: Double,
    val bodyLeftInnovationLastMps: Double,
    val bodyForwardSlopeMps2: Double,
    val bodyLeftSlopeMps2: Double,
    val bodyForwardResidualMadMps: Double,
    val bodyLeftResidualMadMps: Double,
    val forwardSignRatio: Double,
    val leftSignRatio: Double
)
