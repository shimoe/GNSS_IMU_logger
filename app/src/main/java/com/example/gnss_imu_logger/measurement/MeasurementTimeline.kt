package com.example.gnss_imu_logger.measurement

import android.os.SystemClock

/**
 * 計測状態の主要時刻とREADY区間を集計する。
 * 入力: 状態遷移
 * 出力: session.jsonへ保存可能な計測状態統計
 */
class MeasurementTimeline(
    private val measurementStartedNs: Long = SystemClock.elapsedRealtimeNanos()
) {
    private var firstReadyNs: Long? = null
    private var lastReadyNs: Long? = null
    private var stoppedNs: Long? = null

    fun markReady(nowNs: Long) {
        if (firstReadyNs == null) firstReadyNs = nowNs
        lastReadyNs = nowNs
    }

    fun markStopped(nowNs: Long) {
        stoppedNs = nowNs
    }

    fun summary(quality: GnssQualitySummary): MeasurementStateSummary =
        MeasurementStateSummary(
            measurementStartedElapsedNs = measurementStartedNs,
            firstReadyElapsedNs = firstReadyNs,
            lastReadyElapsedNs = lastReadyNs,
            measurementStoppedElapsedNs = stoppedNs,
            timeToFirstReadyMs = firstReadyNs?.let {
                (it - measurementStartedNs) / NS_PER_MS
            },
            readyEnteredCount = quality.readyEnteredCount,
            degradedCount = quality.degradedCount,
            gnssLostCount = quality.lostCount,
            totalReadyDurationMs = quality.totalReadyDurationMs,
            totalDegradedDurationMs = quality.totalDegradedDurationMs
        )

    companion object {
        private const val NS_PER_MS = 1_000_000L
    }
}

data class MeasurementStateSummary(
    val measurementStartedElapsedNs: Long,
    val firstReadyElapsedNs: Long?,
    val lastReadyElapsedNs: Long?,
    val measurementStoppedElapsedNs: Long?,
    val timeToFirstReadyMs: Long?,
    val readyEnteredCount: Int,
    val degradedCount: Int,
    val gnssLostCount: Int,
    val totalReadyDurationMs: Long,
    val totalDegradedDurationMs: Long
)
