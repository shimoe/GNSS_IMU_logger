package com.example.gnss_imu_logger.measurement

/**
 * READY成立後のGNSS品質低下、受信喪失、復旧を判定する。
 * 入力: 現在時刻とReadinessStatus
 * 出力: GNSS品質イベント
 */
class GnssQualityMonitor {
    private var lostReported = false
    private var degradedStartedNs: Long? = null
    private var readyStartedNs: Long? = null
    private var lastUpdateNs: Long? = null
    private var readyCount = 0
    private var degradedCount = 0
    private var lostCount = 0
    private var readyDurationNs = 0L
    private var degradedDurationNs = 0L

    fun onReady(nowNs: Long, recovered: Boolean): List<GnssQualityEvent> {
        val events = mutableListOf<GnssQualityEvent>()
        degradedStartedNs?.let {
            degradedDurationNs += nowNs - it
            degradedStartedNs = null
        }
        readyStartedNs = nowNs
        readyCount++
        lostReported = false
        lastUpdateNs = nowNs
        events += if (recovered) GnssQualityEvent.RECOVERED else GnssQualityEvent.READY
        return events
    }

    fun onDegraded(nowNs: Long): List<GnssQualityEvent> {
        val events = mutableListOf<GnssQualityEvent>()
        readyStartedNs?.let {
            readyDurationNs += nowNs - it
            readyStartedNs = null
        }
        if (degradedStartedNs == null) {
            degradedStartedNs = nowNs
            degradedCount++
            events += GnssQualityEvent.DEGRADED
        }
        lastUpdateNs = nowNs
        return events
    }

    fun evaluateLocationAge(nowNs: Long, lastLocationNs: Long?): List<GnssQualityEvent> {
        if (lastLocationNs == null || lostReported) return emptyList()
        val ageNs = nowNs - lastLocationNs
        if (ageNs < LOST_THRESHOLD_NS) return emptyList()
        lostReported = true
        lostCount++
        return listOf(GnssQualityEvent.LOST)
    }

    fun finish(nowNs: Long): GnssQualitySummary {
        readyStartedNs?.let { readyDurationNs += nowNs - it }
        degradedStartedNs?.let { degradedDurationNs += nowNs - it }
        readyStartedNs = null
        degradedStartedNs = null
        return snapshot()
    }

    fun snapshot(): GnssQualitySummary = GnssQualitySummary(
        readyEnteredCount = readyCount,
        degradedCount = degradedCount,
        lostCount = lostCount,
        totalReadyDurationMs = readyDurationNs / NS_PER_MS,
        totalDegradedDurationMs = degradedDurationNs / NS_PER_MS
    )

    companion object {
        private const val LOST_THRESHOLD_NS = 3_000_000_000L
        private const val NS_PER_MS = 1_000_000L
    }
}

enum class GnssQualityEvent {
    READY,
    DEGRADED,
    LOST,
    RECOVERED
}

data class GnssQualitySummary(
    val readyEnteredCount: Int,
    val degradedCount: Int,
    val lostCount: Int,
    val totalReadyDurationMs: Long,
    val totalDegradedDurationMs: Long
)
