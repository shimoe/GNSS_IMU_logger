package com.example.gnss_imu_logger.playback

/**
 * セッションの再生時間と、任意時刻へ移動するための検索を管理する。
 * 入力: 再生位置[ms]
 * 出力: 指定時刻から開始するPlaybackCursor
 */
class PlaybackTimeline(private val session: PlaybackSession) {
    val durationMs: Long =
        (session.endElapsedNs - session.startElapsedNs).coerceAtLeast(0L) / NS_PER_MS

    /**
     * 指定した再生位置から順方向再生するカーソルを生成する。
     * 入力: セッション先頭からの再生位置[ms]
     * 出力: 各時系列の開始位置を持つPlaybackCursor
     */
    fun createCursor(startPositionMs: Long = 0L): PlaybackCursor {
        val positionMs = clampPosition(startPositionMs)
        val elapsedNs = elapsedRealtimeNs(positionMs)
        return PlaybackCursor(
            session = session,
            initialElapsedNs = elapsedNs,
            initialIndices = findIndices(elapsedNs)
        )
    }

    private fun clampPosition(positionMs: Long): Long =
        positionMs.coerceIn(0L, durationMs)

    private fun elapsedRealtimeNs(positionMs: Long): Long =
        session.startElapsedNs + positionMs * NS_PER_MS

    /**
     * シーク開始位置を二分探索する。
     * 通常の順方向再生では呼び出さない。
     */
    private fun findIndices(elapsedNs: Long): PlaybackIndices = PlaybackIndices(
        location = session.locations.findPreviousIndex(elapsedNs) { it.elapsedRealtimeNs },
        attitude = session.attitudes.findPreviousIndex(elapsedNs) { it.elapsedRealtimeNs },
        event = session.events.findPreviousIndex(elapsedNs) { it.elapsedRealtimeNs }
    )

    private fun <T> List<T>.findPreviousIndex(
        elapsedNs: Long,
        timestampNs: (T) -> Long
    ): Int {
        var low = 0
        var high = lastIndex
        var result = -1
        while (low <= high) {
            val middle = (low + high).ushr(1)
            if (timestampNs(this[middle]) <= elapsedNs) {
                result = middle
                low = middle + 1
            } else {
                high = middle - 1
            }
        }
        return result
    }

    companion object {
        internal const val NS_PER_MS = 1_000_000L
    }
}
