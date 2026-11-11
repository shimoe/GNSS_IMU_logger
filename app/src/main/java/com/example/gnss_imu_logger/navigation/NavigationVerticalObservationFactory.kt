package com.example.gnss_imu_logger.navigation

import kotlin.math.abs

/** 気圧前処理結果を10 Hz以下の鉛直観測へ変換する。 */
class NavigationVerticalObservationFactory(
    private val outputIntervalNs: Long = 100_000_000L,
    private val positionStdM: Double = 5.0,
    private val speedStdMps: Double = 0.5,
    private val maximumAbsoluteSpeedMps: Double = 8.0
) {
    fun create(
        barometric: BarometricAltitudeResult,
        reference: TerrainVerticalReferenceResult?
    ): List<NavigationVerticalObservation> {
        val usable = barometric.records.filter {
            it.valid && it.verticalSpeedMps != null &&
                abs(it.verticalSpeedMps) <= maximumAbsoluteSpeedMps
        }
        if (usable.isEmpty()) return emptyList()
        val originAltitudeM = usable.first().calibratedAltitudeM
        val source = if (
            reference?.recommendedAltitudeSource == "BAROMETER_TERRAIN_ALIGNED"
        ) NavigationAltitudeSource.BAROMETER_TERRAIN_ALIGNED
        else NavigationAltitudeSource.BAROMETER_RELATIVE
        val result = mutableListOf<NavigationVerticalObservation>()
        var nextOutputNs = usable.first().elapsedRealtimeNs
        usable.forEach { value ->
            if (value.elapsedRealtimeNs < nextOutputNs) return@forEach
            result += NavigationVerticalObservation(
                elapsedRealtimeNs = value.elapsedRealtimeNs,
                relativeUpM = value.calibratedAltitudeM - originAltitudeM,
                verticalSpeedMps = value.verticalSpeedMps!!,
                positionStdM = positionStdM,
                speedStdMps = speedStdMps,
                altitudeSource = source,
                valid = true,
                rejectionReason = ""
            )
            nextOutputNs = value.elapsedRealtimeNs + outputIntervalNs
        }
        return result
    }
}
