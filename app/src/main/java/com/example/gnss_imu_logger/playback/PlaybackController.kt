package com.example.gnss_imu_logger.playback

/**
 * 再生、一時停止、再生速度、シークを管理する。
 * 入力: 単調増加する実時間[ms]と再生操作
 * 出力: 再生位置と同期サンプル
 */
class PlaybackController(private val timeline: PlaybackTimeline) {
    private var cursor = timeline.createCursor()
    private var previousRealtimeMs: Long? = null

    var state = PlaybackState(
        durationMs = timeline.durationMs,
        sample = cursor.currentSample()
    )
        private set

    fun play(realtimeMs: Long) {
        if (state.positionMs >= state.durationMs) return
        previousRealtimeMs = realtimeMs
        state = state.copy(playing = true)
    }

    fun pause() {
        previousRealtimeMs = null
        state = state.copy(playing = false)
    }

    fun setSpeed(multiplier: Double) {
        require(multiplier in ALLOWED_SPEEDS) {
            "対応していない再生速度です: $multiplier"
        }
        state = state.copy(speedMultiplier = multiplier)
    }

    /**
     * 実時間の進行分だけ再生位置を前進させる。
     * 通常再生では二分探索を行わない。
     */
    fun update(realtimeMs: Long): PlaybackState {
        if (!state.playing) return state

        val previousMs = previousRealtimeMs ?: realtimeMs
        previousRealtimeMs = realtimeMs
        val elapsedMs = (realtimeMs - previousMs).coerceAtLeast(0L)
        val advancedMs = (elapsedMs * state.speedMultiplier).toLong()
        val positionMs = (state.positionMs + advancedMs)
            .coerceAtMost(state.durationMs)

        state = state.copy(
            playing = positionMs < state.durationMs,
            positionMs = positionMs,
            sample = cursor.advanceTo(positionMs)
        )
        if (!state.playing) previousRealtimeMs = null
        return state
    }

    /**
     * 任意時刻へ移動し、各時系列のカーソルを再配置する。
     * 入力: セッション先頭からの再生位置[ms]
     * 出力: 移動後のPlaybackState
     */
    fun seek(positionMs: Long): PlaybackState {
        val clampedPositionMs = positionMs.coerceIn(0L, state.durationMs)
        val wasPlaying = state.playing
        cursor = timeline.createCursor(clampedPositionMs)
        previousRealtimeMs = null
        state = state.copy(
            playing = wasPlaying && clampedPositionMs < state.durationMs,
            positionMs = clampedPositionMs,
            sample = cursor.currentSample()
        )
        return state
    }

    fun restart(): PlaybackState = seek(0L)

    companion object {
        val ALLOWED_SPEEDS = setOf(0.25, 0.5, 1.0, 2.0, 4.0)
    }
}

data class PlaybackState(
    val playing: Boolean = false,
    val speedMultiplier: Double = 1.0,
    val positionMs: Long = 0L,
    val durationMs: Long,
    val sample: PlaybackSample? = null,
    val errorMessage: String? = null
)
