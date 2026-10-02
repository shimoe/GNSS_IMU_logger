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
    private var activeUtcOffsetNs: Long = initialUtcOffsetNs
    private var lastGnssUtcOffsetNs: Long? = null

    fun sessionElapsedNs(elapsedRealtimeNs: Long): Long =
        elapsedRealtimeNs - startElapsedNs

    @Synchronized
    fun estimateUtcNs(elapsedRealtimeNs: Long): Long =
        elapsedRealtimeNs + activeUtcOffsetNs

    /**
     * GNSS位置時刻からUTCオフセットを更新する。
     * 初回GNSS受信は比較対象がないため、変更警告を出さず初期化だけ行う。
     */
    @Synchronized
    fun updateGnssUtcOffset(elapsedRealtimeNs: Long, utcMs: Long): OffsetUpdate {
        val currentOffsetNs = utcMs * NS_PER_MS - elapsedRealtimeNs
        val previousOffsetNs = lastGnssUtcOffsetNs
        val initialized = previousOffsetNs == null
        val changed = previousOffsetNs != null &&
            abs(currentOffsetNs - previousOffsetNs) >= UTC_CHANGE_THRESHOLD_NS

        lastGnssUtcOffsetNs = currentOffsetNs
        activeUtcOffsetNs = currentOffsetNs
        return OffsetUpdate(
            initialized = initialized,
            changed = changed,
            previousOffsetNs = previousOffsetNs,
            currentOffsetNs = currentOffsetNs
        )
    }

    data class OffsetUpdate(
        val initialized: Boolean,
        val changed: Boolean,
        val previousOffsetNs: Long?,
        val currentOffsetNs: Long
    )

    companion object {
        private const val NS_PER_MS = 1_000_000L
        private const val UTC_CHANGE_THRESHOLD_NS = 100_000_000L
    }
}
