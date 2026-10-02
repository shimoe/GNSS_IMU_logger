package com.example.gnss_imu_logger.model

data class SessionEvent(
    val elapsedRealtimeNs: Long,
    val sessionElapsedNs: Long,
    val utcNs: Long,
    val level: EventLevel,
    val type: EventType,
    val message: String
)

enum class EventLevel { INFO, WARNING, ERROR }

enum class EventType {
    SESSION_CREATED, SERVICE_STARTED, WAKE_LOCK_ACQUIRED,
    SENSOR_REGISTERED, SENSOR_UNAVAILABLE, GNSS_REGISTERED,
    SCREEN_ON, SCREEN_OFF, UTC_OFFSET_CHANGED, WRITE_ERROR,
    LOW_STORAGE, INCOMPLETE_SESSION_FOUND, STOP_REQUESTED,
    SESSION_COMPLETED, SERVICE_DESTROYED,
    MEASUREMENT_STARTED, GNSS_WAIT_STARTED, READY_ENTERED,
    GNSS_DEGRADED, GNSS_LOST, GNSS_RECOVERED,
    MEASUREMENT_ERROR, MEASUREMENT_COMPLETED,
    STATE_TRANSITION_REJECTED
}
