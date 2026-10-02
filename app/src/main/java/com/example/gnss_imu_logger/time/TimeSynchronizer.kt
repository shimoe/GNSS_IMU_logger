package com.example.gnss_imu_logger.time

import android.os.SystemClock
import kotlin.math.abs

/**
 * elapsedRealtimeNanosを基準にセッション時刻とUTC対応を管理する。
 * 入力: センサーまたはGNSSのelapsedRealtimeNs、対応するUTC
 * 出力: セッション経過時刻、UTC推定値、UTCオフセット変更判定
 */
class TimeSynchronizer(
    val startElapsedNs: Long = SystemClock.elapsedRealtimeNanos(),
    val startUtcNs: Long = System.currentTimeMillis() * NS_PER_MS
) {
    val initialUtcOffsetNs: Long = startUtcNs - startElapsedNs
    private var lastUtcOffsetNs: Long = initialUtcOffsetNs

    fun sessionElapsedNs(elapsedRealtimeNs: Long): Long =
        elapsedRealtimeNs - startElapsedNs

    fun estimateUtcNs(elapsedRealtimeNs: Long): Long =
        elapsedRealtimeNs + lastUtcOffsetNs

    fun updateUtcOffset(elapsedRealtimeNs: Long, utcMs: Long): OffsetUpdate {
        val newOffsetNs = utcMs * NS_PER_MS - elapsedRealtimeNs
        val changed = abs(newOffsetNs - lastUtcOffsetNs) >= UTC_CHANGE_THRESHOLD_NS
        val previousOffsetNs = lastUtcOffsetNs
        lastUtcOffsetNs = newOffsetNs
        return OffsetUpdate(changed, previousOffsetNs, newOffsetNs)
    }

    data class OffsetUpdate(
        val changed: Boolean,
        val previousOffsetNs: Long,
        val currentOffsetNs: Long
    )

    companion object {
        private const val NS_PER_MS = 1_000_000L
        private const val UTC_CHANGE_THRESHOLD_NS = 100_000_000L
    }
}
