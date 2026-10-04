package com.example.gnss_imu_logger.playback

/**
 * 通常再生中のGNSS、姿勢、イベントの現在位置を保持する。
 * 入力: 単調増加する再生位置[ms]
 * 出力: 現在時刻以前の最新サンプル
 */
class PlaybackCursor internal constructor(
    private val session: PlaybackSession,
    initialElapsedNs: Long,
    initialIndices: PlaybackIndices
) {
    private var locationIndex = initialIndices.location
    private var attitudeIndex = initialIndices.attitude
    private var eventIndex = initialIndices.event
    private var currentElapsedNs = initialElapsedNs

    /**
     * 現在位置から指定位置まで各インデックスを前進させる。
     * 入力: セッション先頭からの再生位置[ms]
     * 出力: 指定位置に対応するPlaybackSample
     */
    fun advanceTo(positionMs: Long): PlaybackSample {
        val requestedNs = session.startElapsedNs +
            positionMs.coerceAtLeast(0L) * PlaybackTimeline.NS_PER_MS
        val targetNs = requestedNs.coerceAtMost(session.endElapsedNs)

        require(targetNs >= currentElapsedNs) {
            "通常再生中に時刻を戻すことはできません。シーク処理を使用してください"
        }

        locationIndex = session.locations.advanceIndex(
            locationIndex,
            targetNs
        ) { it.elapsedRealtimeNs }
        attitudeIndex = session.attitudes.advanceIndex(
            attitudeIndex,
            targetNs
        ) { it.elapsedRealtimeNs }
        eventIndex = session.events.advanceIndex(
            eventIndex,
            targetNs
        ) { it.elapsedRealtimeNs }
        currentElapsedNs = targetNs
        return currentSample()
    }

    fun currentSample(): PlaybackSample {
        val location = session.locations.getOrNull(locationIndex)
            ?.takeIf {
                currentElapsedNs >= it.elapsedRealtimeNs &&
                    currentElapsedNs - it.elapsedRealtimeNs <= MAX_LOCATION_AGE_NS
            }
        val attitude = session.attitudes.getOrNull(attitudeIndex)
            ?.takeIf { it.elapsedRealtimeNs <= currentElapsedNs }
        val event = session.events.getOrNull(eventIndex)
            ?.takeIf { it.elapsedRealtimeNs <= currentElapsedNs }

        return PlaybackSample(
            elapsedRealtimeNs = currentElapsedNs,
            sessionElapsedNs = currentElapsedNs - session.startElapsedNs,
            location = location,
            attitude = attitude,
            latestEvent = event
        )
    }

    private fun <T> List<T>.advanceIndex(
        currentIndex: Int,
        targetNs: Long,
        timestampNs: (T) -> Long
    ): Int {
        var index = currentIndex
        while (index + 1 < size && timestampNs(this[index + 1]) <= targetNs) {
            index++
        }
        return index
    }

    companion object {
        private const val MAX_LOCATION_AGE_NS = 3_000_000_000L
    }
}
