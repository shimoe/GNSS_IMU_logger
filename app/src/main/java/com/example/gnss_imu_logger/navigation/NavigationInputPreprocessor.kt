package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.BodyFrameTransform
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * 時刻順の保存ログをナビゲーション推定器へ投入できる入力へ変換する。
 * 入力: OfflineNavigationInput
 * 出力: 車体座標IMU、ENU座標GNSS更新、入力診断
 */
class NavigationInputPreprocessor(
    private val parameters: NavigationInputParameters = NavigationInputParameters.DEFAULT
) {
    fun prepare(input: OfflineNavigationInput): PreparedNavigationInput {
        val plane = LocalTangentPlane(input.origin)
        val diagnostics = MutableDiagnostics()
        var previousEventNs: Long? = null
        var previousAccelerationNs: Long? = null
        var previousAngularRateNs: Long? = null
        var previousGnssNs: Long? = null
        var firstAcceptedGnss: NavigationGnssUpdate? = null
        val events = input.events.mapNotNull { event ->
            val previous = previousEventNs
            if (previous != null && event.elapsedRealtimeNs < previous) diagnostics.reversedTimestampCount++
            previousEventNs = maxOf(previous ?: event.elapsedRealtimeNs, event.elapsedRealtimeNs)
            when (event) {
                is OfflineNavigationEvent.Acceleration -> {
                    diagnostics.accelerationSampleCount++
                    previousAccelerationNs = updateInterval(
                        event.elapsedRealtimeNs,
                        previousAccelerationNs,
                        parameters.maximumImuIntervalNs,
                        { diagnostics.maximumAccelerationIntervalNs = maxOf(diagnostics.maximumAccelerationIntervalNs, it) },
                        { diagnostics.accelerationGapCount++ }
                    )
                    val sample = event.sample
                    PreparedNavigationEvent.Imu(
                        NavigationImuSample(
                            elapsedRealtimeNs = sample.elapsedRealtimeNs,
                            accelerationBodyMps2 = BodyFrameTransform.deviceToBody(
                                floatArrayOf(
                                    sample.correctedX.toFloat(),
                                    sample.correctedY.toFloat(),
                                    sample.correctedZ.toFloat()
                                )
                            ),
                            angularRateBodyRadps = null,
                            sensorAccuracy = sample.accuracy
                        )
                    )
                }
                is OfflineNavigationEvent.AngularRate -> {
                    diagnostics.angularRateSampleCount++
                    previousAngularRateNs = updateInterval(
                        event.elapsedRealtimeNs,
                        previousAngularRateNs,
                        parameters.maximumImuIntervalNs,
                        { diagnostics.maximumAngularRateIntervalNs = maxOf(diagnostics.maximumAngularRateIntervalNs, it) },
                        { diagnostics.angularRateGapCount++ }
                    )
                    val sample = event.sample
                    PreparedNavigationEvent.Imu(
                        NavigationImuSample(
                            elapsedRealtimeNs = sample.elapsedRealtimeNs,
                            accelerationBodyMps2 = null,
                            angularRateBodyRadps = BodyFrameTransform.deviceToBody(
                                floatArrayOf(
                                    sample.correctedX.toFloat(),
                                    sample.correctedY.toFloat(),
                                    sample.correctedZ.toFloat()
                                )
                            ),
                            sensorAccuracy = sample.accuracy
                        )
                    )
                }
                is OfflineNavigationEvent.Gnss -> {
                    diagnostics.gnssObservationCount++
                    previousGnssNs = updateInterval(
                        event.elapsedRealtimeNs,
                        previousGnssNs,
                        parameters.maximumGnssIntervalNs,
                        { diagnostics.maximumGnssIntervalNs = maxOf(diagnostics.maximumGnssIntervalNs, it) },
                        { diagnostics.gnssGapCount++ }
                    )
                    if (!event.decision.accepted) {
                        diagnostics.rejectedGnssCount++
                        null
                    } else {
                        diagnostics.acceptedGnssCount++
                        if (event.decision.bearingUsable) diagnostics.bearingUpdateCount++
                        val update = toGnssUpdate(event, plane)
                        if (firstAcceptedGnss == null) firstAcceptedGnss = update
                        val horizontalDistanceM = sqrt(
                            update.positionEnuM.eastM * update.positionEnuM.eastM +
                                update.positionEnuM.northM * update.positionEnuM.northM
                        )
                        diagnostics.maximumHorizontalDistanceM = maxOf(
                            diagnostics.maximumHorizontalDistanceM,
                            horizontalDistanceM
                        )
                        PreparedNavigationEvent.Gnss(update)
                    }
                }
            }
        }.toList()
        val initialGnss = firstAcceptedGnss
            ?: error("採用可能なGNSS観測がありません。")
        return PreparedNavigationInput(
            origin = input.origin,
            initialElapsedRealtimeNs = initialGnss.elapsedRealtimeNs,
            initialPositionEnuM = initialGnss.positionEnuM,
            initialVelocityEnuMps = initialGnss.velocityEnuMps ?: EnuVelocity.ZERO,
            events = events,
            diagnostics = diagnostics.snapshot()
        )
    }

    private fun toGnssUpdate(
        event: OfflineNavigationEvent.Gnss,
        plane: LocalTangentPlane
    ): NavigationGnssUpdate {
        val observation = event.observation
        val bearingRad = observation.bearingDeg?.let(Math::toRadians)
        val velocity = if (
            observation.speedMps != null && bearingRad != null &&
            observation.speedMps.isFinite() && bearingRad.isFinite()
        ) {
            EnuVelocity(
                eastMps = observation.speedMps * sin(bearingRad),
                northMps = observation.speedMps * cos(bearingRad),
                upMps = 0.0
            )
        } else null
        return NavigationGnssUpdate(
            elapsedRealtimeNs = observation.elapsedRealtimeNs,
            positionEnuM = plane.toEnu(observation.position),
            velocityEnuMps = velocity,
            positionStdM = event.decision.positionStdM,
            verticalStdM = event.decision.verticalStdM,
            speedStdMps = event.decision.speedStdMps,
            bearingRad = bearingRad?.takeIf { event.decision.bearingUsable },
            bearingAccuracyRad = observation.bearingAccuracyDeg
                ?.takeIf { event.decision.bearingUsable }
                ?.let(Math::toRadians),
            usedSatellites = observation.usedSatellites,
            usedSatelliteMeanCn0DbHz = observation.usedSatelliteMeanCn0DbHz
        )
    }

    private fun updateInterval(
        timestampNs: Long,
        previousTimestampNs: Long?,
        maximumIntervalNs: Long,
        updateMaximum: (Long) -> Unit,
        addGap: () -> Unit
    ): Long {
        if (previousTimestampNs != null) {
            val intervalNs = timestampNs - previousTimestampNs
            if (intervalNs >= 0L) {
                updateMaximum(intervalNs)
                if (intervalNs > maximumIntervalNs) addGap()
            }
        }
        return timestampNs
    }

    private class MutableDiagnostics {
        var accelerationSampleCount = 0L
        var angularRateSampleCount = 0L
        var gnssObservationCount = 0L
        var acceptedGnssCount = 0L
        var rejectedGnssCount = 0L
        var bearingUpdateCount = 0L
        var reversedTimestampCount = 0L
        var accelerationGapCount = 0L
        var angularRateGapCount = 0L
        var gnssGapCount = 0L
        var maximumAccelerationIntervalNs = 0L
        var maximumAngularRateIntervalNs = 0L
        var maximumGnssIntervalNs = 0L
        var maximumHorizontalDistanceM = 0.0

        fun snapshot() = OfflineNavigationInputDiagnostics(
            accelerationSampleCount = accelerationSampleCount,
            angularRateSampleCount = angularRateSampleCount,
            gnssObservationCount = gnssObservationCount,
            acceptedGnssCount = acceptedGnssCount,
            rejectedGnssCount = rejectedGnssCount,
            bearingUpdateCount = bearingUpdateCount,
            reversedTimestampCount = reversedTimestampCount,
            accelerationGapCount = accelerationGapCount,
            angularRateGapCount = angularRateGapCount,
            gnssGapCount = gnssGapCount,
            maximumAccelerationIntervalNs = maximumAccelerationIntervalNs,
            maximumAngularRateIntervalNs = maximumAngularRateIntervalNs,
            maximumGnssIntervalNs = maximumGnssIntervalNs,
            maximumHorizontalDistanceM = maximumHorizontalDistanceM
        )
    }
}

/** 推定器へ渡す車体座標IMUサンプル。 */
data class NavigationImuSample(
    val elapsedRealtimeNs: Long,
    val accelerationBodyMps2: BodyVector?,
    val angularRateBodyRadps: BodyVector?,
    val sensorAccuracy: Int
)

/** 推定器へ渡すENU座標GNSS更新。 */
data class NavigationGnssUpdate(
    val elapsedRealtimeNs: Long,
    val positionEnuM: EnuPosition,
    val velocityEnuMps: EnuVelocity?,
    val positionStdM: Double,
    val verticalStdM: Double,
    val speedStdMps: Double,
    val bearingRad: Double?,
    val bearingAccuracyRad: Double?,
    val usedSatellites: Int?,
    val usedSatelliteMeanCn0DbHz: Double?
)

data class EnuVelocity(
    val eastMps: Double,
    val northMps: Double,
    val upMps: Double
) {
    companion object {
        val ZERO = EnuVelocity(0.0, 0.0, 0.0)
    }
}

sealed interface PreparedNavigationEvent {
    val elapsedRealtimeNs: Long

    data class Imu(val sample: NavigationImuSample) : PreparedNavigationEvent {
        override val elapsedRealtimeNs: Long = sample.elapsedRealtimeNs
    }

    data class Gnss(val update: NavigationGnssUpdate) : PreparedNavigationEvent {
        override val elapsedRealtimeNs: Long = update.elapsedRealtimeNs
    }
}

data class PreparedNavigationInput(
    val origin: GeodeticPosition,
    val initialElapsedRealtimeNs: Long,
    val initialPositionEnuM: EnuPosition,
    val initialVelocityEnuMps: EnuVelocity,
    val events: List<PreparedNavigationEvent>,
    val diagnostics: OfflineNavigationInputDiagnostics
)

data class OfflineNavigationInputDiagnostics(
    val accelerationSampleCount: Long,
    val angularRateSampleCount: Long,
    val gnssObservationCount: Long,
    val acceptedGnssCount: Long,
    val rejectedGnssCount: Long,
    val bearingUpdateCount: Long,
    val reversedTimestampCount: Long,
    val accelerationGapCount: Long,
    val angularRateGapCount: Long,
    val gnssGapCount: Long,
    val maximumAccelerationIntervalNs: Long,
    val maximumAngularRateIntervalNs: Long,
    val maximumGnssIntervalNs: Long,
    val maximumHorizontalDistanceM: Double
)

data class NavigationInputParameters(
    val maximumImuIntervalNs: Long,
    val maximumGnssIntervalNs: Long
) {
    companion object {
        val DEFAULT = NavigationInputParameters(
            maximumImuIntervalNs = 100_000_000L,
            maximumGnssIntervalNs = 3_000_000_000L
        )
    }
}
