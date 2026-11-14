package com.example.gnss_imu_logger.playback

import com.example.gnss_imu_logger.navigation.LapRecord

/**
 * 再生対象区間。
 * 入力: セッション先頭基準の開始・終了位置[ms]
 * 出力: 相対再生位置と絶対再生位置の相互変換
 */
data class PlaybackRange(
    val label: String,
    val startMs: Long,
    val finishMs: Long,
    val lap: LapRecord? = null
) {
    val durationMs: Long get() = (finishMs - startMs).coerceAtLeast(0L)

    fun relativePositionMs(absolutePositionMs: Long): Long =
        (absolutePositionMs - startMs).coerceIn(0L, durationMs)

    companion object {
        fun whole(durationMs: Long) = PlaybackRange("セッション全体", 0L, durationMs)

        fun from(session: PlaybackSession, lap: LapRecord): PlaybackRange {
            val startMs = (lap.startElapsedRealtimeNs - session.startElapsedNs)
                .coerceAtLeast(0L) / NS_PER_MS
            val sessionDurationMs = (session.endElapsedNs - session.startElapsedNs)
                .coerceAtLeast(0L) / NS_PER_MS
            val finishMs = ((lap.finishElapsedRealtimeNs - session.startElapsedNs)
                .coerceAtLeast(0L) / NS_PER_MS).coerceAtMost(sessionDurationMs)
            return PlaybackRange(
                lapLabel(lap),
                startMs.coerceAtMost(finishMs),
                finishMs,
                lap
            )
        }

        private fun lapLabel(lap: LapRecord): String = when (lap.lapType.name) {
            "OUT_LAP" -> "OUT"
            "IN_LAP" -> "IN"
            else -> "Lap ${lap.lapIndex}"
        }

        private const val NS_PER_MS = 1_000_000L
    }
}
