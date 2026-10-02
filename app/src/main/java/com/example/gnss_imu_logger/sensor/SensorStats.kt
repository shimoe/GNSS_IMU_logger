package com.example.gnss_imu_logger.sensor

import kotlin.math.roundToLong

/**
 * センサーの受信周期と欠落推定数を集計する。
 * 入力: センサーイベントのelapsedRealtimeNs
 * 出力: 受信数、推定欠落数、平均周波数、最大受信間隔
 */
class SensorStats {
    private var receivedCount = 0L
    private var estimatedMissingCount = 0L
    private var firstTimestampNs: Long? = null
    private var lastTimestampNs: Long? = null
    private var maximumIntervalNs = 0L
    private var intervalSumNs = 0L
    private val recentIntervals = LongArray(MEDIAN_WINDOW_SIZE)
    private var recentCount = 0
    private var recentIndex = 0

    @Synchronized
    fun add(timestampNs: Long) {
        val previousNs = lastTimestampNs
        if (firstTimestampNs == null) firstTimestampNs = timestampNs
        receivedCount++

        if (previousNs != null && timestampNs > previousNs) {
            val intervalNs = timestampNs - previousNs
            intervalSumNs += intervalNs
            if (intervalNs > maximumIntervalNs) maximumIntervalNs = intervalNs
            recentIntervals[recentIndex] = intervalNs
            recentIndex = (recentIndex + 1) % MEDIAN_WINDOW_SIZE
            if (recentCount < MEDIAN_WINDOW_SIZE) recentCount++

            val expectedNs = medianIntervalNsInternal()
            if (expectedNs > 0L && intervalNs > expectedNs * GAP_RATIO) {
                val missing = (intervalNs.toDouble() / expectedNs)
                    .roundToLong()
                    .minus(1L)
                    .coerceAtLeast(1L)
                estimatedMissingCount += missing
            }
        }
        lastTimestampNs = timestampNs
    }

    @Synchronized
    fun snapshot(writtenCount: Long, droppedCount: Long): Snapshot {
        val firstNs = firstTimestampNs
        val lastNs = lastTimestampNs
        val durationNs = if (firstNs != null && lastNs != null) lastNs - firstNs else 0L
        val rateHz = if (durationNs > 0L && receivedCount > 1L) {
            (receivedCount - 1L) * NS_PER_SEC.toDouble() / durationNs
        } else 0.0
        val meanIntervalNs = if (receivedCount > 1L) {
            intervalSumNs.toDouble() / (receivedCount - 1L)
        } else 0.0
        return Snapshot(
            receivedCount = receivedCount,
            writtenCount = writtenCount,
            droppedCount = droppedCount,
            estimatedMissingCount = estimatedMissingCount,
            meanRateHz = rateHz,
            meanIntervalMs = meanIntervalNs / NS_PER_MS,
            medianIntervalMs = medianIntervalNsInternal() / NS_PER_MS,
            maximumIntervalMs = maximumIntervalNs / NS_PER_MS
        )
    }

    private fun medianIntervalNsInternal(): Long {
        if (recentCount == 0) return 0L
        val sorted = recentIntervals.copyOf(recentCount).sortedArray()
        return sorted[recentCount / 2]
    }

    data class Snapshot(
        val receivedCount: Long,
        val writtenCount: Long,
        val droppedCount: Long,
        val estimatedMissingCount: Long,
        val meanRateHz: Double,
        val meanIntervalMs: Double,
        val medianIntervalMs: Double,
        val maximumIntervalMs: Double
    )

    companion object {
        private const val MEDIAN_WINDOW_SIZE = 2048
        private const val GAP_RATIO = 2.5
        private const val NS_PER_SEC = 1_000_000_000L
        private const val NS_PER_MS = 1_000_000.0
    }
}
