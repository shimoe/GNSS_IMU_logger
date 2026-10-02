package com.example.gnss_imu_logger.sensor

/**
 * GNSS位置更新の周期を集計する。
 * 入力: Location.elapsedRealtimeNanos
 * 出力: 件数、平均周波数、最大更新間隔
 */
class GnssStats {
    private var count = 0L
    private var firstNs: Long? = null
    private var lastNs: Long? = null
    private var maximumIntervalNs = 0L

    @Synchronized
    fun add(elapsedRealtimeNs: Long) {
        val previousNs = lastNs
        if (firstNs == null) firstNs = elapsedRealtimeNs
        if (previousNs != null && elapsedRealtimeNs > previousNs) {
            val intervalNs = elapsedRealtimeNs - previousNs
            if (intervalNs > maximumIntervalNs) maximumIntervalNs = intervalNs
        }
        lastNs = elapsedRealtimeNs
        count++
    }

    @Synchronized
    fun snapshot(): Snapshot {
        val first = firstNs
        val last = lastNs
        val durationNs = if (first != null && last != null) last - first else 0L
        val rateHz = if (durationNs > 0L && count > 1L) {
            (count - 1L) * NS_PER_SEC.toDouble() / durationNs
        } else 0.0
        return Snapshot(count, rateHz, maximumIntervalNs / NS_PER_MS)
    }

    data class Snapshot(
        val count: Long,
        val meanRateHz: Double,
        val maximumIntervalMs: Double
    )

    companion object {
        private const val NS_PER_SEC = 1_000_000_000L
        private const val NS_PER_MS = 1_000_000.0
    }
}
