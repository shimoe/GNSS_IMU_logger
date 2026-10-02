package com.example.gnss_imu_logger.measurement

enum class ReadinessReason {
    NONE,
    INITIALIZING_SENSORS,
    NO_IMU_DATA,
    NO_GNSS_LOCATION,
    INSUFFICIENT_LOCATION_COUNT,
    INSUFFICIENT_SATELLITES,
    POOR_HORIZONTAL_ACCURACY,
    GNSS_UPDATE_DELAYED,
    LOW_STORAGE,
    WRITE_ERROR
}

data class ReadinessStatus(
    val ready: Boolean,
    val reason: ReadinessReason,
    val locationCount: Int,
    val usedSatellites: Int,
    val horizontalAccuracyM: Float?,
    val lastLocationAgeMs: Long?,
    val stableDurationMs: Long
)

data class GnssQualitySample(
    val elapsedRealtimeNs: Long,
    val horizontalAccuracyM: Float,
    val usedSatellites: Int,
    val consecutiveLocationCount: Int
)
