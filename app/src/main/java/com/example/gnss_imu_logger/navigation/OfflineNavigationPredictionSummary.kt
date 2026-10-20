package com.example.gnss_imu_logger.navigation

import kotlin.math.abs
import kotlin.math.sqrt

/**
 * 純粋IMU予測とGNSS残差から静的評価値を集計する。
 * 入力: OfflineNavigationPredictionResult、GNSS欠落判定間隔[ns]
 * 出力: 全体誤差、発散傾向、GNSS欠落区間評価
 */
class OfflineNavigationPredictionSummarizer(
    private val gnssGapThresholdNs: Long = DEFAULT_GNSS_GAP_THRESHOLD_NS
) {
    fun summarize(result: OfflineNavigationPredictionResult): OfflineNavigationPredictionSummary {
        require(result.samples.isNotEmpty()) { "ナビゲーション予測サンプルがありません。" }
        require(result.gnssResiduals.isNotEmpty()) { "GNSS残差がありません。" }
        val residuals = result.gnssResiduals
        val horizontalErrors = residuals.map { it.horizontalErrorM }
        val velocityErrors = residuals.mapNotNull { it.velocityErrorMps }
        val gapEvaluations = residuals.zipWithNext().mapNotNull { (start, end) ->
            val durationNs = end.elapsedRealtimeNs - start.elapsedRealtimeNs
            if (durationNs <= gnssGapThresholdNs) return@mapNotNull null
            NavigationGnssGapEvaluation(
                startElapsedRealtimeNs = start.elapsedRealtimeNs,
                endElapsedRealtimeNs = end.elapsedRealtimeNs,
                durationNs = durationNs,
                startHorizontalErrorM = start.horizontalErrorM,
                endHorizontalErrorM = end.horizontalErrorM,
                horizontalErrorIncreaseM = end.horizontalErrorM - start.horizontalErrorM,
                startVelocityErrorMps = start.velocityErrorMps,
                endVelocityErrorMps = end.velocityErrorMps,
                velocityErrorIncreaseMps = difference(end.velocityErrorMps, start.velocityErrorMps)
            )
        }
        val finalSample = result.samples.last()
        return OfflineNavigationPredictionSummary(
            predictionSampleCount = result.samples.size,
            gnssResidualCount = residuals.size,
            firstHorizontalErrorM = horizontalErrors.first(),
            medianHorizontalErrorM = horizontalErrors.median(),
            finalHorizontalErrorM = horizontalErrors.last(),
            maximumHorizontalErrorM = horizontalErrors.maxOrNull() ?: 0.0,
            finalVelocityErrorMps = velocityErrors.lastOrNull(),
            maximumVelocityErrorMps = velocityErrors.maxOrNull(),
            maximumAbsoluteUpVelocityMps = result.samples.maxOf {
                abs(it.velocityEnuMps.upMps)
            },
            maximumLinearAccelerationMps2 = result.samples.maxOf {
                it.accelerationEnuMps2.norm()
            },
            finalPositionEnuM = finalSample.positionEnuM,
            finalVelocityEnuMps = finalSample.velocityEnuMps,
            imuGapSampleCount = result.samples.count {
                it.status == NavigationPredictionStatus.IMU_GAP
            },
            gnssGapEvaluations = gapEvaluations,
            assessment = assess(horizontalErrors.last(), velocityErrors.lastOrNull())
        )
    }

    private fun assess(
        finalHorizontalErrorM: Double,
        finalVelocityErrorMps: Double?
    ): NavigationPredictionAssessment = when {
        !finalHorizontalErrorM.isFinite() || finalVelocityErrorMps?.isFinite() == false ->
            NavigationPredictionAssessment.INVALID
        finalHorizontalErrorM > DIVERGED_POSITION_ERROR_M ||
            (finalVelocityErrorMps ?: 0.0) > DIVERGED_VELOCITY_ERROR_MPS ->
            NavigationPredictionAssessment.DIVERGED
        finalHorizontalErrorM > LARGE_POSITION_ERROR_M ||
            (finalVelocityErrorMps ?: 0.0) > LARGE_VELOCITY_ERROR_MPS ->
            NavigationPredictionAssessment.LARGE_DRIFT
        else -> NavigationPredictionAssessment.ACCEPTABLE_FOR_PURE_PREDICTION
    }

    private fun difference(first: Double?, second: Double?): Double? =
        if (first != null && second != null) first - second else null

    private fun List<Double>.median(): Double {
        val sorted = sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) {
            (sorted[middle - 1] + sorted[middle]) * 0.5
        } else sorted[middle]
    }

    private fun com.example.gnss_imu_logger.attitude.BodyVector.norm(): Double =
        sqrt(x * x + y * y + z * z)

    companion object {
        private const val DEFAULT_GNSS_GAP_THRESHOLD_NS = 3_000_000_000L
        private const val LARGE_POSITION_ERROR_M = 100.0
        private const val DIVERGED_POSITION_ERROR_M = 1_000.0
        private const val LARGE_VELOCITY_ERROR_MPS = 20.0
        private const val DIVERGED_VELOCITY_ERROR_MPS = 100.0
    }
}

data class OfflineNavigationPredictionSummary(
    val predictionSampleCount: Int,
    val gnssResidualCount: Int,
    val firstHorizontalErrorM: Double,
    val medianHorizontalErrorM: Double,
    val finalHorizontalErrorM: Double,
    val maximumHorizontalErrorM: Double,
    val finalVelocityErrorMps: Double?,
    val maximumVelocityErrorMps: Double?,
    val maximumAbsoluteUpVelocityMps: Double,
    val maximumLinearAccelerationMps2: Double,
    val finalPositionEnuM: EnuPosition,
    val finalVelocityEnuMps: EnuVelocity,
    val imuGapSampleCount: Int,
    val gnssGapEvaluations: List<NavigationGnssGapEvaluation>,
    val assessment: NavigationPredictionAssessment
)

data class NavigationGnssGapEvaluation(
    val startElapsedRealtimeNs: Long,
    val endElapsedRealtimeNs: Long,
    val durationNs: Long,
    val startHorizontalErrorM: Double,
    val endHorizontalErrorM: Double,
    val horizontalErrorIncreaseM: Double,
    val startVelocityErrorMps: Double?,
    val endVelocityErrorMps: Double?,
    val velocityErrorIncreaseMps: Double?
)

enum class NavigationPredictionAssessment {
    ACCEPTABLE_FOR_PURE_PREDICTION,
    LARGE_DRIFT,
    DIVERGED,
    INVALID
}
