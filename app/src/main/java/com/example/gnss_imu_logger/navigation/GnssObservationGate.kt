package com.example.gnss_imu_logger.navigation

/**
 * 保存済みGNSS観測の品質から、オフライン推定へ適用する分散と使用可否を決める。
 * 入力: GnssObservation
 * 出力: 採用可否、位置・高度・速度の標準偏差、方位使用可否
 */
class GnssObservationGate(
    private val parameters: GnssObservationGateParameters = GnssObservationGateParameters.DEFAULT
) {
    fun evaluate(observation: GnssObservation): GnssObservationDecision {
        if (observation.mock) return rejected("模擬位置のため使用しません。")
        if (observation.elapsedRealtimeNs <= 0L) return rejected("GNSS時刻が不正です。")
        val horizontalAccuracyM = observation.horizontalAccuracyM
        if (horizontalAccuracyM == null || !horizontalAccuracyM.isFinite() || horizontalAccuracyM <= 0.0) {
            return rejected("水平精度が不正です。")
        }
        if (horizontalAccuracyM > parameters.maximumHorizontalAccuracyM) {
            return rejected("水平精度が許容範囲を超えています。")
        }

        var qualityScale = 1.0
        val statusFresh = observation.statusAgeMs?.let {
            it in 0..parameters.maximumStatusAgeMs
        } == true
        if (statusFresh) {
            val used = observation.usedSatellites
            if (used != null && used < parameters.minimumUsedSatellites) {
                qualityScale *= parameters.lowSatelliteScale
            }
            val cn0 = observation.usedSatelliteMeanCn0DbHz
            if (cn0 != null && cn0 < parameters.minimumMeanCn0DbHz) {
                qualityScale *= parameters.lowCn0Scale
            }
        }

        val timeUncertaintyNs = observation.elapsedRealtimeUncertaintyNs
        if (timeUncertaintyNs != null && timeUncertaintyNs > parameters.maximumTimeUncertaintyNs) {
            qualityScale *= parameters.highTimeUncertaintyScale
        }

        val verticalStdM = observation.verticalAccuracyM
            ?.takeIf { it.isFinite() && it > 0.0 }
            ?.coerceAtLeast(parameters.minimumVerticalStdM)
            ?: parameters.defaultVerticalStdM
        val speedStdMps = observation.speedAccuracyMps
            ?.takeIf { it.isFinite() && it > 0.0 }
            ?.coerceAtLeast(parameters.minimumSpeedStdMps)
            ?: parameters.defaultSpeedStdMps
        val bearingUsable = observation.speedMps?.let { speed ->
            speed >= parameters.minimumBearingSpeedMps &&
                observation.bearingDeg?.isFinite() == true &&
                observation.bearingAccuracyDeg?.let {
                    it.isFinite() && it <= parameters.maximumBearingAccuracyDeg
                } == true
        } == true
        return GnssObservationDecision(
            accepted = true,
            positionStdM = horizontalAccuracyM.coerceAtLeast(parameters.minimumHorizontalStdM) * qualityScale,
            verticalStdM = verticalStdM * qualityScale,
            speedStdMps = speedStdMps * qualityScale,
            bearingUsable = bearingUsable,
            reason = if (qualityScale > 1.0) "品質低下のため観測分散を拡大します。" else "GNSS観測を使用します。"
        )
    }

    private fun rejected(reason: String) = GnssObservationDecision(
        accepted = false,
        positionStdM = Double.POSITIVE_INFINITY,
        verticalStdM = Double.POSITIVE_INFINITY,
        speedStdMps = Double.POSITIVE_INFINITY,
        bearingUsable = false,
        reason = reason
    )
}

/** GNSS観測ゲートの調整値。 */
data class GnssObservationGateParameters(
    val maximumHorizontalAccuracyM: Double,
    val minimumHorizontalStdM: Double,
    val minimumVerticalStdM: Double,
    val defaultVerticalStdM: Double,
    val minimumSpeedStdMps: Double,
    val defaultSpeedStdMps: Double,
    val minimumUsedSatellites: Int,
    val minimumMeanCn0DbHz: Double,
    val maximumStatusAgeMs: Long,
    val maximumTimeUncertaintyNs: Double,
    val lowSatelliteScale: Double,
    val lowCn0Scale: Double,
    val highTimeUncertaintyScale: Double,
    val minimumBearingSpeedMps: Double,
    val maximumBearingAccuracyDeg: Double
) {
    companion object {
        val DEFAULT = GnssObservationGateParameters(
            maximumHorizontalAccuracyM = 25.0,
            minimumHorizontalStdM = 1.0,
            minimumVerticalStdM = 1.5,
            defaultVerticalStdM = 10.0,
            minimumSpeedStdMps = 0.15,
            defaultSpeedStdMps = 1.0,
            minimumUsedSatellites = 12,
            minimumMeanCn0DbHz = 20.0,
            maximumStatusAgeMs = 1_500L,
            maximumTimeUncertaintyNs = 50_000_000.0,
            lowSatelliteScale = 1.5,
            lowCn0Scale = 1.5,
            highTimeUncertaintyScale = 1.5,
            minimumBearingSpeedMps = 5.0,
            maximumBearingAccuracyDeg = 15.0
        )
    }
}
