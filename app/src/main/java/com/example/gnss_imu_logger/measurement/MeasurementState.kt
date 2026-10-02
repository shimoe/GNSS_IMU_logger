package com.example.gnss_imu_logger.measurement

enum class MeasurementState {
    IDLE,
    INITIALIZING,
    WAITING_FOR_GNSS,
    READY,
    DEGRADED,
    STOPPING,
    ERROR
}
