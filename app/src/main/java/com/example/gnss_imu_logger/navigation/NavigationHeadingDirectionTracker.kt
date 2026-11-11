package com.example.gnss_imu_logger.navigation

import kotlin.math.abs

/** GNSS進行方位の通常方向と180度反転方向を複数観測で判定し、確定後は保持する。 */
class NavigationHeadingDirectionTracker(
    private val parameters: NavigationHeadingDirectionParameters =
        NavigationHeadingDirectionParameters.DEFAULT
) {
    private var state = NavigationHeadingDirectionState.UNRESOLVED
    private var candidate = NavigationHeadingDirectionCandidate.NONE
    private var candidateCount = 0
    private var candidateStartedNs: Long? = null
    private var lastCandidateEvidenceNs: Long? = null
    private var confirmedElapsedRealtimeNs: Long? = null
    private var conflictingEvidenceCount = 0

    fun update(
        elapsedRealtimeNs: Long,
        speedMps: Double?,
        bearingAccuracyRad: Double?,
        speedAccuracyMps: Double?,
        yawRateRadps: Double?,
        regularInnovationRad: Double?,
        oppositeInnovationRad: Double?
    ): NavigationHeadingDirectionRecord {
        val stateBefore = state
        val decisionReason = decisionRejectionReason(
            speedMps,
            bearingAccuracyRad,
            speedAccuracyMps,
            yawRateRadps,
            regularInnovationRad,
            oppositeInnovationRad
        )
        val evidence = if (decisionReason.isEmpty()) {
            selectEvidence(regularInnovationRad!!, oppositeInnovationRad!!)
        } else NavigationHeadingDirectionEvidence.UNAVAILABLE
        val evidenceCandidate = evidence.toCandidate()
        val confirmed = state.isConfirmed()
        val conflict = confirmed && evidenceCandidate != NavigationHeadingDirectionCandidate.NONE &&
            evidenceCandidate != state.toCandidate()

        if (confirmed) {
            if (conflict) conflictingEvidenceCount++
        } else {
            updateCandidate(elapsedRealtimeNs, evidenceCandidate)
            val durationNs = candidateStartedNs?.let { elapsedRealtimeNs - it } ?: 0L
            if (candidate != NavigationHeadingDirectionCandidate.NONE &&
                candidateCount >= parameters.requiredCandidateCount &&
                durationNs >= parameters.requiredCandidateDurationNs) {
                state = if (candidate == NavigationHeadingDirectionCandidate.FORWARD) {
                    NavigationHeadingDirectionState.FORWARD
                } else NavigationHeadingDirectionState.REVERSE
                confirmedElapsedRealtimeNs = elapsedRealtimeNs
            }
        }

        val durationSec = candidateStartedNs?.let {
            (elapsedRealtimeNs - it).coerceAtLeast(0L) / NS_PER_SEC
        } ?: 0.0
        val updateReason = when {
            !state.isConfirmed() -> "DIRECTION_UNRESOLVED"
            conflict -> "DIRECTION_CONFLICT"
            yawRateRadps != null && abs(yawRateRadps) > parameters.maximumUpdateYawRateRadps ->
                "YAW_RATE"
            else -> ""
        }
        return NavigationHeadingDirectionRecord(
            elapsedRealtimeNs = elapsedRealtimeNs,
            speedMps = speedMps,
            bearingAccuracyRad = bearingAccuracyRad,
            speedAccuracyMps = speedAccuracyMps,
            yawRateRadps = yawRateRadps,
            regularInnovationRad = regularInnovationRad,
            oppositeInnovationRad = oppositeInnovationRad,
            candidateMarginRad = innovationMargin(regularInnovationRad, oppositeInnovationRad),
            evidence = evidence,
            candidate = candidate,
            candidateCount = candidateCount,
            candidateDurationSec = durationSec,
            candidateStartedNs = candidateStartedNs,
            lastCandidateEvidenceNs = lastCandidateEvidenceNs,
            stateBefore = stateBefore,
            stateAfter = state,
            confirmedElapsedRealtimeNs = confirmedElapsedRealtimeNs,
            conflictingEvidenceCount = conflictingEvidenceCount,
            directionDecisionAvailable = decisionReason.isEmpty(),
            directionDecisionRejectionReason = decisionReason,
            headingUpdateAllowed = updateReason.isEmpty(),
            headingUpdateRejectionReason = updateReason
        )
    }

    private fun updateCandidate(
        elapsedRealtimeNs: Long,
        evidenceCandidate: NavigationHeadingDirectionCandidate
    ) {
        val gapExceeded = lastCandidateEvidenceNs?.let {
            elapsedRealtimeNs - it > parameters.maximumCandidateGapNs
        } ?: false
        if (gapExceeded) resetCandidate()
        if (evidenceCandidate == NavigationHeadingDirectionCandidate.NONE) return
        if (candidate != evidenceCandidate) {
            candidate = evidenceCandidate
            candidateCount = 1
            candidateStartedNs = elapsedRealtimeNs
        } else {
            candidateCount++
        }
        lastCandidateEvidenceNs = elapsedRealtimeNs
        state = if (candidate == NavigationHeadingDirectionCandidate.FORWARD) {
            NavigationHeadingDirectionState.FORWARD_CANDIDATE
        } else NavigationHeadingDirectionState.REVERSE_CANDIDATE
    }

    private fun resetCandidate() {
        state = NavigationHeadingDirectionState.UNRESOLVED
        candidate = NavigationHeadingDirectionCandidate.NONE
        candidateCount = 0
        candidateStartedNs = null
        lastCandidateEvidenceNs = null
    }

    private fun decisionRejectionReason(
        speedMps: Double?,
        bearingAccuracyRad: Double?,
        speedAccuracyMps: Double?,
        yawRateRadps: Double?,
        regularInnovationRad: Double?,
        oppositeInnovationRad: Double?
    ): String = when {
        speedMps == null || speedMps < parameters.minimumSpeedMps -> "SPEED"
        bearingAccuracyRad == null -> "BEARING_UNAVAILABLE"
        bearingAccuracyRad > parameters.maximumBearingAccuracyRad -> "BEARING_ACCURACY"
        speedAccuracyMps == null -> "SPEED_ACCURACY_UNAVAILABLE"
        speedAccuracyMps > parameters.maximumSpeedAccuracyMps -> "SPEED_ACCURACY"
        yawRateRadps == null || abs(yawRateRadps) > parameters.maximumDecisionYawRateRadps ->
            "YAW_RATE"
        regularInnovationRad == null || oppositeInnovationRad == null -> "INNOVATION_UNAVAILABLE"
        else -> ""
    }

    private fun selectEvidence(
        regularInnovationRad: Double,
        oppositeInnovationRad: Double
    ): NavigationHeadingDirectionEvidence {
        val regular = abs(regularInnovationRad)
        val opposite = abs(oppositeInnovationRad)
        return when {
            regular <= parameters.maximumCandidateInnovationRad &&
                opposite - regular >= parameters.minimumCandidateMarginRad ->
                NavigationHeadingDirectionEvidence.REGULAR
            opposite <= parameters.maximumCandidateInnovationRad &&
                regular - opposite >= parameters.minimumCandidateMarginRad ->
                NavigationHeadingDirectionEvidence.OPPOSITE
            else -> NavigationHeadingDirectionEvidence.AMBIGUOUS
        }
    }

    private fun innovationMargin(first: Double?, second: Double?): Double? =
        if (first != null && second != null) abs(abs(first) - abs(second)) else null
    private fun NavigationHeadingDirectionEvidence.toCandidate() = when (this) {
        NavigationHeadingDirectionEvidence.REGULAR -> NavigationHeadingDirectionCandidate.FORWARD
        NavigationHeadingDirectionEvidence.OPPOSITE -> NavigationHeadingDirectionCandidate.REVERSE
        else -> NavigationHeadingDirectionCandidate.NONE
    }
    private fun NavigationHeadingDirectionState.toCandidate() = when (this) {
        NavigationHeadingDirectionState.FORWARD -> NavigationHeadingDirectionCandidate.FORWARD
        NavigationHeadingDirectionState.REVERSE -> NavigationHeadingDirectionCandidate.REVERSE
        else -> NavigationHeadingDirectionCandidate.NONE
    }
    private fun NavigationHeadingDirectionState.isConfirmed() =
        this == NavigationHeadingDirectionState.FORWARD ||
            this == NavigationHeadingDirectionState.REVERSE

    companion object { private const val NS_PER_SEC = 1_000_000_000.0 }
}

enum class NavigationHeadingDirectionState {
    UNRESOLVED,
    FORWARD_CANDIDATE,
    FORWARD,
    REVERSE_CANDIDATE,
    REVERSE
}

enum class NavigationHeadingDirectionCandidate { NONE, FORWARD, REVERSE }
enum class NavigationHeadingDirectionEvidence { REGULAR, OPPOSITE, AMBIGUOUS, UNAVAILABLE }

data class NavigationHeadingDirectionParameters(
    val minimumSpeedMps: Double,
    val maximumBearingAccuracyRad: Double,
    val maximumSpeedAccuracyMps: Double,
    val maximumDecisionYawRateRadps: Double,
    val maximumUpdateYawRateRadps: Double,
    val maximumCandidateInnovationRad: Double,
    val minimumCandidateMarginRad: Double,
    val requiredCandidateCount: Int,
    val requiredCandidateDurationNs: Long,
    val maximumCandidateGapNs: Long
) {
    companion object {
        val DEFAULT = NavigationHeadingDirectionParameters(
            minimumSpeedMps = 5.0,
            maximumBearingAccuracyRad = Math.toRadians(15.0),
            maximumSpeedAccuracyMps = 2.0,
            maximumDecisionYawRateRadps = 0.35,
            maximumUpdateYawRateRadps = 1.0,
            maximumCandidateInnovationRad = Math.toRadians(45.0),
            minimumCandidateMarginRad = Math.toRadians(30.0),
            requiredCandidateCount = 4,
            requiredCandidateDurationNs = 3_000_000_000L,
            maximumCandidateGapNs = 1_500_000_000L
        )
    }
}

data class NavigationHeadingDirectionRecord(
    val elapsedRealtimeNs: Long,
    val speedMps: Double?,
    val bearingAccuracyRad: Double?,
    val speedAccuracyMps: Double?,
    val yawRateRadps: Double?,
    val regularInnovationRad: Double?,
    val oppositeInnovationRad: Double?,
    val candidateMarginRad: Double?,
    val evidence: NavigationHeadingDirectionEvidence,
    val candidate: NavigationHeadingDirectionCandidate,
    val candidateCount: Int,
    val candidateDurationSec: Double,
    val candidateStartedNs: Long?,
    val lastCandidateEvidenceNs: Long?,
    val stateBefore: NavigationHeadingDirectionState,
    val stateAfter: NavigationHeadingDirectionState,
    val confirmedElapsedRealtimeNs: Long?,
    val conflictingEvidenceCount: Int,
    val directionDecisionAvailable: Boolean,
    val directionDecisionRejectionReason: String,
    val headingUpdateAllowed: Boolean,
    val headingUpdateRejectionReason: String
)
