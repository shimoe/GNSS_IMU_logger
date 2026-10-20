package com.example.gnss_imu_logger.navigation

import kotlin.math.abs

/** GNSS進行方位の通常方向と180度反転方向を、状態適用前に時系列評価する。 */
class NavigationHeadingDirectionTracker(
    private val parameters: NavigationHeadingDirectionParameters =
        NavigationHeadingDirectionParameters.DEFAULT
) {
    private var state = NavigationHeadingDirectionState.UNRESOLVED
    private var candidate = NavigationHeadingDirectionCandidate.NONE
    private var candidateSinceNs: Long? = null

    fun update(
        elapsedRealtimeNs: Long,
        speedMps: Double?,
        angularRateNormRadps: Double?,
        regularInnovationRad: Double?,
        oppositeInnovationRad: Double?
    ): NavigationHeadingDirectionRecord {
        val evidence = selectEvidence(
            speedMps,
            angularRateNormRadps,
            regularInnovationRad,
            oppositeInnovationRad
        )
        val nextCandidate = when (evidence) {
            NavigationHeadingDirectionEvidence.REGULAR -> NavigationHeadingDirectionCandidate.FORWARD
            NavigationHeadingDirectionEvidence.OPPOSITE -> NavigationHeadingDirectionCandidate.REVERSE
            else -> NavigationHeadingDirectionCandidate.NONE
        }
        if (nextCandidate != candidate) {
            candidate = nextCandidate
            candidateSinceNs = if (candidate == NavigationHeadingDirectionCandidate.NONE) null else elapsedRealtimeNs
        }
        val durationSec = candidateSinceNs?.let { (elapsedRealtimeNs - it) / NS_PER_SEC } ?: 0.0
        if (candidate != NavigationHeadingDirectionCandidate.NONE &&
            durationSec >= parameters.minimumCandidateDurationSec) {
            state = if (candidate == NavigationHeadingDirectionCandidate.FORWARD) {
                NavigationHeadingDirectionState.FORWARD
            } else {
                NavigationHeadingDirectionState.REVERSE
            }
        }
        // 低速・旋回中は判定を保留するが、確定済みの方向は保持する。
        return NavigationHeadingDirectionRecord(
            elapsedRealtimeNs = elapsedRealtimeNs,
            speedMps = speedMps,
            angularRateNormRadps = angularRateNormRadps,
            regularInnovationRad = regularInnovationRad,
            oppositeInnovationRad = oppositeInnovationRad,
            evidence = evidence,
            candidate = candidate,
            candidateDurationSec = durationSec,
            state = state
        )
    }

    private fun selectEvidence(
        speedMps: Double?,
        angularRateNormRadps: Double?,
        regularInnovationRad: Double?,
        oppositeInnovationRad: Double?
    ): NavigationHeadingDirectionEvidence {
        if (speedMps == null || speedMps < parameters.minimumSpeedMps) {
            return NavigationHeadingDirectionEvidence.HOLD
        }
        if (angularRateNormRadps != null &&
            angularRateNormRadps > parameters.maximumAngularRateRadps) {
            return NavigationHeadingDirectionEvidence.HOLD
        }
        if (regularInnovationRad == null || oppositeInnovationRad == null) {
            return NavigationHeadingDirectionEvidence.UNAVAILABLE
        }
        val marginRad = abs(regularInnovationRad) - abs(oppositeInnovationRad)
        return when {
            marginRad >= parameters.minimumInnovationMarginRad ->
                NavigationHeadingDirectionEvidence.OPPOSITE
            marginRad <= -parameters.minimumInnovationMarginRad ->
                NavigationHeadingDirectionEvidence.REGULAR
            else -> NavigationHeadingDirectionEvidence.AMBIGUOUS
        }
    }

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
    }
}

enum class NavigationHeadingDirectionState { UNRESOLVED, FORWARD, REVERSE }
enum class NavigationHeadingDirectionCandidate { NONE, FORWARD, REVERSE }
enum class NavigationHeadingDirectionEvidence { UNAVAILABLE, HOLD, AMBIGUOUS, REGULAR, OPPOSITE }

data class NavigationHeadingDirectionParameters(
    val minimumSpeedMps: Double,
    val maximumAngularRateRadps: Double,
    val minimumInnovationMarginRad: Double,
    val minimumCandidateDurationSec: Double
) {
    companion object {
        val DEFAULT = NavigationHeadingDirectionParameters(
            minimumSpeedMps = 5.0,
            maximumAngularRateRadps = 0.35,
            minimumInnovationMarginRad = Math.toRadians(30.0),
            minimumCandidateDurationSec = 3.0
        )
    }
}

data class NavigationHeadingDirectionRecord(
    val elapsedRealtimeNs: Long,
    val speedMps: Double?,
    val angularRateNormRadps: Double?,
    val regularInnovationRad: Double?,
    val oppositeInnovationRad: Double?,
    val evidence: NavigationHeadingDirectionEvidence,
    val candidate: NavigationHeadingDirectionCandidate,
    val candidateDurationSec: Double,
    val state: NavigationHeadingDirectionState
)
