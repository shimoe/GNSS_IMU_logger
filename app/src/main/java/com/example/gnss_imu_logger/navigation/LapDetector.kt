package com.example.gnss_imu_logger.navigation

import kotlin.math.hypot

/**
 * 方向付きスタートライン交差からラップを確定する。
 * 入力: 100 Hz自己位置推定、スタートライン設定、セッション原点
 * 出力: OUT/TIMED/INラップと全交差候補の診断
 */
class LapDetector(
    config: LapDetectionConfig,
    origin: GeodeticPosition
) {
    private val config = config
    private val plane = LocalTangentPlane(origin)
    private val geometry = LapGeometry(
        start = plane.toEnu(config.lineStart.copy(altitudeM = origin.altitudeM)),
        end = plane.toEnu(config.lineEnd.copy(altitudeM = origin.altitudeM))
    )

    fun analyze(samples: List<NavigationPredictionSample>): LapAnalysisResult {
        require(samples.size >= 2) { "ラップ判定には2件以上の自己位置推定が必要です。" }
        val records = mutableListOf<LapRecord>()
        val events = mutableListOf<LapEventRecord>()
        var state = LapDetectionState.WAITING_FOR_START
        var previous = samples.first()
        var lastAcceptedCrossingNs: Long? = null
        var currentLapStartNs = samples.first().elapsedRealtimeNs
        var currentDistanceM = 0.0
        var timedLapIndex = 0

        samples.drop(1).forEach { current ->
            val segmentDistanceM = horizontalDistance(previous.positionEnuM, current.positionEnuM)
            currentDistanceM += segmentDistanceM
            if (state != LapDetectionState.ARMED &&
                geometry.lineDistance(current.positionEnuM) >= config.rearmDistanceM
            ) {
                state = LapDetectionState.ARMED
            }
            val beforeDistanceM = geometry.signedDistance(previous.positionEnuM)
            val afterDistanceM = geometry.signedDistance(current.positionEnuM)
            val candidate = geometry.crossingCandidate(
                previous.positionEnuM,
                current.positionEnuM
            )
            if (candidate.crossedInfiniteLine) {
                val stateBefore = state
                val ratio = requireNotNull(candidate.trackRatio).coerceIn(0.0, 1.0)
                val crossingNs = interpolateTime(
                    previous.elapsedRealtimeNs,
                    current.elapsedRealtimeNs,
                    ratio
                )
                val speedMps = interpolate(
                    previous.velocityEnuMps.horizontalSpeedMps(),
                    current.velocityEnuMps.horizontalSpeedMps(),
                    ratio
                )
                val normalSpeedMps = geometry.normalSpeed(current.velocityEnuMps)
                val uncertaintyM = current.uncertainty?.horizontalPositionStdM
                val uncertaintyExtensionM = uncertaintyM
                    ?.coerceAtLeast(0.0)
                    ?.coerceAtMost(config.maximumUncertaintyExtensionM)
                    ?: 0.0
                val effectiveExtensionM = config.lineEndExtensionM + uncertaintyExtensionM
                val endpointDistanceM = requireNotNull(candidate.endpointDistanceM)
                val rejection = rejectionReason(
                    state = state,
                    crossingNs = crossingNs,
                    lastAcceptedCrossingNs = lastAcceptedCrossingNs,
                    speedMps = speedMps,
                    normalSpeedMps = normalSpeedMps,
                    uncertaintyM = uncertaintyM,
                    status = current.status,
                    beforeDistanceM = beforeDistanceM,
                    afterDistanceM = afterDistanceM,
                    endpointDistanceM = endpointDistanceM,
                    effectiveExtensionM = effectiveExtensionM
                )
                if (rejection.isEmpty()) {
                    val distanceAtCrossingM = currentDistanceM - segmentDistanceM * (1.0 - ratio)
                    if (lastAcceptedCrossingNs == null) {
                        records += createRecord(
                            lapIndex = 0,
                            lapType = LapType.OUT_LAP,
                            startNs = currentLapStartNs,
                            finishNs = crossingNs,
                            distanceM = distanceAtCrossingM,
                            crossingSpeedMps = speedMps,
                            uncertaintyM = uncertaintyM
                        )
                    } else {
                        timedLapIndex++
                        records += createRecord(
                            lapIndex = timedLapIndex,
                            lapType = LapType.TIMED_LAP,
                            startNs = currentLapStartNs,
                            finishNs = crossingNs,
                            distanceM = distanceAtCrossingM,
                            crossingSpeedMps = speedMps,
                            uncertaintyM = uncertaintyM
                        )
                    }
                    lastAcceptedCrossingNs = crossingNs
                    currentLapStartNs = crossingNs
                    currentDistanceM = segmentDistanceM * (1.0 - ratio)
                    state = LapDetectionState.LOCKED_OUT
                }
                events += LapEventRecord(
                    elapsedRealtimeNs = crossingNs,
                    stateBefore = stateBefore,
                    stateAfter = state,
                    signedDistanceBeforeM = beforeDistanceM,
                    signedDistanceAfterM = afterDistanceM,
                    lineRatio = candidate.lineRatio,
                    crossedInfiniteLine = candidate.crossedInfiniteLine,
                    endpointDistanceM = endpointDistanceM,
                    baseExtensionM = config.lineEndExtensionM,
                    uncertaintyExtensionM = uncertaintyExtensionM,
                    effectiveExtensionM = effectiveExtensionM,
                    crossingSpeedMps = speedMps,
                    normalSpeedMps = normalSpeedMps,
                    positionUncertaintyM = uncertaintyM,
                    accepted = rejection.isEmpty(),
                    rejectionReason = rejection
                )
            }
            previous = current
        }
        val last = samples.last()
        if (last.elapsedRealtimeNs > currentLapStartNs) {
            records += createRecord(
                lapIndex = if (lastAcceptedCrossingNs == null) 0 else timedLapIndex + 1,
                lapType = LapType.IN_LAP,
                startNs = currentLapStartNs,
                finishNs = last.elapsedRealtimeNs,
                distanceM = currentDistanceM,
                crossingSpeedMps = null,
                uncertaintyM = last.uncertainty?.horizontalPositionStdM
            )
        }
        return LapAnalysisResult(markDurationOutliers(records), events)
    }

    private fun rejectionReason(
        state: LapDetectionState,
        crossingNs: Long,
        lastAcceptedCrossingNs: Long?,
        speedMps: Double,
        normalSpeedMps: Double,
        uncertaintyM: Double?,
        status: NavigationPredictionStatus,
        beforeDistanceM: Double,
        afterDistanceM: Double,
        endpointDistanceM: Double,
        effectiveExtensionM: Double
    ): String = when {
        state != LapDetectionState.ARMED -> "NOT_REARMED"
        status != NavigationPredictionStatus.PREDICTING -> "NO_VALID_POSITION"
        uncertaintyM == null || !uncertaintyM.isFinite() ||
            uncertaintyM > config.maximumPositionUncertaintyM -> "POSITION_UNCERTAIN"
        speedMps < config.minimumSpeedMps -> "SPEED_TOO_LOW"
        !isAllowedDirection(beforeDistanceM, afterDistanceM, normalSpeedMps) -> "WRONG_DIRECTION"
        endpointDistanceM > effectiveExtensionM -> "OUTSIDE_LINE_SEGMENT"
        lastAcceptedCrossingNs != null &&
            crossingNs - lastAcceptedCrossingNs <
            (config.minimumLapDurationSec * NS_PER_SEC).toLong() -> "MINIMUM_LAP_TIME"
        else -> ""
    }

    private fun isAllowedDirection(beforeM: Double, afterM: Double, normalSpeedMps: Double): Boolean =
        when (config.allowedDirection) {
            LapCrossingDirection.LEFT_TO_RIGHT ->
                beforeM > 0.0 && afterM <= 0.0 &&
                    normalSpeedMps <= -config.minimumNormalSpeedMps
            LapCrossingDirection.RIGHT_TO_LEFT ->
                beforeM < 0.0 && afterM >= 0.0 &&
                    normalSpeedMps >= config.minimumNormalSpeedMps
        }

    private fun createRecord(
        lapIndex: Int,
        lapType: LapType,
        startNs: Long,
        finishNs: Long,
        distanceM: Double,
        crossingSpeedMps: Double?,
        uncertaintyM: Double?
    ): LapRecord {
        val durationSec = (finishNs - startNs).coerceAtLeast(0L) / NS_PER_SEC
        return LapRecord(
            lapIndex = lapIndex,
            lapType = lapType,
            startElapsedRealtimeNs = startNs,
            finishElapsedRealtimeNs = finishNs,
            durationSec = durationSec,
            distanceM = distanceM,
            averageSpeedMps = if (durationSec > 0.0) distanceM / durationSec else 0.0,
            crossingSpeedMps = crossingSpeedMps,
            positionUncertaintyM = uncertaintyM,
            positionSource = LapPositionSource.OFFLINE_NAVIGATION,
            valid = true,
            invalidReason = "",
            durationOutlier = false
        )
    }

    /** 計測ラップが3件以上ある場合、全体中央値の1.8倍を超えるラップへ警告を付ける。 */
    private fun markDurationOutliers(records: List<LapRecord>): List<LapRecord> {
        val durations = records
            .filter { it.lapType == LapType.TIMED_LAP && it.valid }
            .map { it.durationSec }
            .sorted()
        if (durations.size < 3) return records
        val middle = durations.size / 2
        val median = if (durations.size % 2 == 0) {
            (durations[middle - 1] + durations[middle]) * 0.5
        } else {
            durations[middle]
        }
        return records.map { record ->
            record.copy(
                durationOutlier = record.lapType == LapType.TIMED_LAP &&
                    record.valid &&
                    record.durationSec > median * 1.8
            )
        }
    }

    private fun horizontalDistance(first: EnuPosition, second: EnuPosition): Double =
        hypot(second.eastM - first.eastM, second.northM - first.northM)

    private fun EnuVelocity.horizontalSpeedMps(): Double = hypot(eastMps, northMps)
    private fun interpolate(first: Double, second: Double, ratio: Double) =
        first + (second - first) * ratio
    private fun interpolateTime(first: Long, second: Long, ratio: Double): Long =
        first + ((second - first) * ratio).toLong()

    companion object { private const val NS_PER_SEC = 1_000_000_000.0 }
}
