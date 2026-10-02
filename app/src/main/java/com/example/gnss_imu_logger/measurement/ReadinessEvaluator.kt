package com.example.gnss_imu_logger.measurement

/**
 * IMUとGNSSの最新状態からREADY条件を評価する。
 * 入力: IMU統計、GNSS位置、使用衛星数、保存先状態
 * 出力: ReadinessStatus
 */
class ReadinessEvaluator {
    private var conditionStartedNs: Long? = null

    fun evaluate(
        nowNs: Long,
        accelerometerCount: Long,
        gyroscopeCount: Long,
        imuDroppedCount: Long,
        locationCount: Int,
        usedSatellites: Int,
        horizontalAccuracyM: Float?,
        lastLocationElapsedNs: Long?,
        storageReady: Boolean
    ): ReadinessStatus {
        val reason = when {
            !storageReady -> ReadinessReason.LOW_STORAGE
            accelerometerCount == 0L || gyroscopeCount == 0L -> ReadinessReason.NO_IMU_DATA
            lastLocationElapsedNs == null -> ReadinessReason.NO_GNSS_LOCATION
            locationCount < REQUIRED_LOCATION_COUNT -> ReadinessReason.INSUFFICIENT_LOCATION_COUNT
            usedSatellites < MIN_USED_SATELLITES -> ReadinessReason.INSUFFICIENT_SATELLITES
            horizontalAccuracyM == null || horizontalAccuracyM > MAX_HORIZONTAL_ACCURACY_M ->
                ReadinessReason.POOR_HORIZONTAL_ACCURACY
            nowNs - lastLocationElapsedNs > MAX_LOCATION_AGE_NS ->
                ReadinessReason.GNSS_UPDATE_DELAYED
            imuDroppedCount > 0L -> ReadinessReason.WRITE_ERROR
            else -> ReadinessReason.NONE
        }

        if (reason != ReadinessReason.NONE) conditionStartedNs = null
        else if (conditionStartedNs == null) conditionStartedNs = nowNs

        val stableDurationNs = conditionStartedNs?.let { nowNs - it } ?: 0L
        return ReadinessStatus(
            ready = reason == ReadinessReason.NONE && stableDurationNs >= REQUIRED_STABLE_NS,
            reason = reason,
            locationCount = locationCount,
            usedSatellites = usedSatellites,
            horizontalAccuracyM = horizontalAccuracyM,
            lastLocationAgeMs = lastLocationElapsedNs?.let { (nowNs - it) / NS_PER_MS },
            stableDurationMs = stableDurationNs / NS_PER_MS
        )
    }

    companion object {
        private const val REQUIRED_LOCATION_COUNT = 3
        private const val MIN_USED_SATELLITES = 8
        private const val MAX_HORIZONTAL_ACCURACY_M = 15f
        private const val MAX_LOCATION_AGE_NS = 1_500_000_000L
        private const val REQUIRED_STABLE_NS = 2_000_000_000L
        private const val NS_PER_MS = 1_000_000L
    }
}
