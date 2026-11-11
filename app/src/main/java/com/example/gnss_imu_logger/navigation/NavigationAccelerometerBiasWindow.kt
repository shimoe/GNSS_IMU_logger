package com.example.gnss_imu_logger.navigation

/** 車体座標へ変換済みのGNSS水平速度innovationを保持する。 */
data class NavigationAccelerometerBiasSample(
    val elapsedRealtimeNs: Long,
    val velocityInnovationEastMps: Double,
    val velocityInnovationNorthMps: Double,
    val velocityInnovationBodyForwardMps: Double,
    val velocityInnovationBodyLeftMps: Double,
    val yawRad: Double,
    val speedMps: Double,
    val yawRateRadps: Double
)

enum class NavigationBiasWindowState {
    EMPTY,
    COLLECTING,
    HOLDING,
    READY,
    ESTIMATED,
    REJECTED
}

data class NavigationBiasWindowSnapshot(
    val sequence: Long,
    val startedElapsedRealtimeNs: Long,
    val lastSampleElapsedRealtimeNs: Long,
    val completedElapsedRealtimeNs: Long,
    val samples: List<NavigationAccelerometerBiasSample>,
    val endReason: String
)

/**
 * 一時的な品質低下では標本を保持し、1つの窓につき推定結果を1回だけ確定する。
 * 入力: 有効な速度innovation、保持または終了の通知
 * 出力: 一意なwindow_sequenceを持つ推定用スナップショット
 */
class NavigationAccelerometerBiasWindow(
    private val maximumDurationNs: Long,
    private val maximumValidSampleGapNs: Long,
    private val maximumHoldNs: Long
) {
    private val samples = ArrayDeque<NavigationAccelerometerBiasSample>()
    var state: NavigationBiasWindowState = NavigationBiasWindowState.EMPTY
        private set
    var holdReason: String = ""
        private set
    var windowEndReason: String = ""
        private set
    var sequence: Long = 0L
        private set
    private var holdStartedNs: Long? = null
    private var lastEventNs: Long? = null
    private var estimatedSampleCount: Int = 0

    fun add(sample: NavigationAccelerometerBiasSample): NavigationBiasWindowSnapshot? {
        if (lastEventNs?.let { sample.elapsedRealtimeNs <= it } == true) {
            return finish(sample.elapsedRealtimeNs, "TIMESTAMP_ORDER")
        }
        val gapExceeded = samples.lastOrNull()?.let {
            sample.elapsedRealtimeNs - it.elapsedRealtimeNs > maximumValidSampleGapNs
        } == true
        val completed = if (gapExceeded) finish(sample.elapsedRealtimeNs, "VALID_SAMPLE_GAP") else null
        if (samples.isEmpty()) {
            sequence++
            estimatedSampleCount = 0
        }
        samples.addLast(sample)
        lastEventNs = sample.elapsedRealtimeNs
        holdStartedNs = null
        holdReason = ""
        windowEndReason = ""
        state = NavigationBiasWindowState.COLLECTING
        if (validDurationNs() >= maximumDurationNs) {
            return completed ?: finish(sample.elapsedRealtimeNs, "MAXIMUM_DURATION")
        }
        return completed
    }

    fun hold(elapsedRealtimeNs: Long, reason: String): NavigationBiasWindowSnapshot? {
        if (samples.isEmpty()) {
            state = NavigationBiasWindowState.EMPTY
            return null
        }
        if (lastEventNs?.let { elapsedRealtimeNs <= it } == true) {
            return finish(elapsedRealtimeNs, "TIMESTAMP_ORDER")
        }
        lastEventNs = elapsedRealtimeNs
        if (holdStartedNs == null) holdStartedNs = elapsedRealtimeNs
        holdReason = reason
        state = NavigationBiasWindowState.HOLDING
        return if (elapsedRealtimeNs - holdStartedNs!! > maximumHoldNs) {
            finish(elapsedRealtimeNs, "HOLD_TIMEOUT")
        } else null
    }

    fun finish(elapsedRealtimeNs: Long, reason: String): NavigationBiasWindowSnapshot? {
        if (samples.isEmpty()) {
            resetToRejected(reason)
            return null
        }
        val snapshot = NavigationBiasWindowSnapshot(
            sequence = sequence,
            startedElapsedRealtimeNs = samples.first().elapsedRealtimeNs,
            lastSampleElapsedRealtimeNs = samples.last().elapsedRealtimeNs,
            completedElapsedRealtimeNs = elapsedRealtimeNs,
            samples = samples.toList(),
            endReason = reason
        )
        resetToRejected(reason)
        return snapshot
    }

    /** 5サンプル到達時は同じ窓を1回だけ中間推定する。 */
    fun snapshotWhenReady(minimumSampleCount: Int, minimumDurationNs: Long): NavigationBiasWindowSnapshot? {
        if (samples.size < minimumSampleCount || validDurationNs() < minimumDurationNs) return null
        if (estimatedSampleCount == samples.size) return null
        estimatedSampleCount = samples.size
        state = NavigationBiasWindowState.READY
        return NavigationBiasWindowSnapshot(
            sequence = sequence,
            startedElapsedRealtimeNs = samples.first().elapsedRealtimeNs,
            lastSampleElapsedRealtimeNs = samples.last().elapsedRealtimeNs,
            completedElapsedRealtimeNs = samples.last().elapsedRealtimeNs,
            samples = samples.toList(),
            endReason = "SAMPLE_THRESHOLD"
        )
    }

    fun markEstimated() {
        state = NavigationBiasWindowState.ESTIMATED
    }

    fun size(): Int = samples.size
    fun validDurationNs(): Long = if (samples.size < 2) 0L else {
        samples.last().elapsedRealtimeNs - samples.first().elapsedRealtimeNs
    }
    fun lastValidSampleGapNs(): Long? = if (samples.size < 2) null else {
        samples.last().elapsedRealtimeNs - samples.elementAt(samples.size - 2).elapsedRealtimeNs
    }
    fun holdDurationNs(nowNs: Long): Long = holdStartedNs?.let { nowNs - it } ?: 0L
    fun startedNs(): Long? = samples.firstOrNull()?.elapsedRealtimeNs
    fun lastSampleNs(): Long? = samples.lastOrNull()?.elapsedRealtimeNs

    private fun resetToRejected(reason: String) {
        samples.clear()
        holdStartedNs = null
        estimatedSampleCount = 0
        holdReason = ""
        windowEndReason = reason
        state = NavigationBiasWindowState.REJECTED
    }
}
