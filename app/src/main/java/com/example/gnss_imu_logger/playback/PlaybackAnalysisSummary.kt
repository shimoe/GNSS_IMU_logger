package com.example.gnss_imu_logger.playback

import kotlin.math.abs

/**
 * セッション全体の走行解析値。
 * 入力: PlaybackSession
 * 出力: 再生画面へ表示する代表値
 */
data class PlaybackAnalysisSummary(
    val sustainedTopSpeedMps: Double?,
    val maximumAccelerationMps2: Double?,
    val maximumDecelerationMps2: Double?,
    val maximumLateralAccelerationMps2: Double?,
    val maximumYawRateRadps: Double?,
    val maximumRollRad: Double?
)

/** セッション全体の走行解析値を算出する。 */
object PlaybackAnalysisSummaryCalculator {

    /**
     * 走行解析値を算出する。
     * 入力: 時刻順に整列した再生セッション
     * 出力: セッション全体の走行解析値
     */
    fun calculate(session: PlaybackSession): PlaybackAnalysisSummary = PlaybackAnalysisSummary(
        sustainedTopSpeedMps = sustainedTopSpeed(session.locations),
        maximumAccelerationMps2 = session.locations
            .mapNotNull { it.longitudinalAccelerationMps2 }
            .maxOrNull(),
        maximumDecelerationMps2 = session.locations
            .mapNotNull { it.longitudinalAccelerationMps2 }
            .minOrNull(),
        maximumLateralAccelerationMps2 = session.attitudes
            .mapNotNull { it.lateralAccelerationMps2 }
            .maximumAbsoluteValue(),
        maximumYawRateRadps = session.attitudes
            .mapNotNull { it.yawRateRadps }
            .maximumAbsoluteValue(),
        maximumRollRad = session.attitudes
            .filter { it.valid }
            .mapNotNull { it.rollRad }
            .maximumAbsoluteValue()
    )

    /**
     * 0.2秒以上継続した速度区間から参考最高速度を算出する。
     * 区間内の低い側の速度を代表値とし、単一サンプルのスパイクを採用しない。
     */
    private fun sustainedTopSpeed(locations: List<PlaybackLocation>): Double? {
        var topSpeedMps: Double? = null
        for (index in 1 until locations.size) {
            val previous = locations[index - 1]
            val current = locations[index]
            val intervalNs = current.elapsedRealtimeNs - previous.elapsedRealtimeNs
            if (intervalNs !in MINIMUM_SPEED_WINDOW_NS..MAXIMUM_SPEED_INTERVAL_NS) continue
            val previousSpeed = previous.speedMps ?: continue
            val currentSpeed = current.speedMps ?: continue
            if (!previousSpeed.isFinite() || !currentSpeed.isFinite()) continue
            val sustainedSpeed = minOf(previousSpeed, currentSpeed)
            if (topSpeedMps == null || sustainedSpeed > topSpeedMps) {
                topSpeedMps = sustainedSpeed
            }
        }
        return topSpeedMps
    }

    /** 絶対値が最大の値を符号付きで返す。 */
    private fun List<Double>.maximumAbsoluteValue(): Double? =
        filter { it.isFinite() }.maxByOrNull { abs(it) }

    private const val MINIMUM_SPEED_WINDOW_NS = 200_000_000L
    private const val MAXIMUM_SPEED_INTERVAL_NS = 2_000_000_000L
}
