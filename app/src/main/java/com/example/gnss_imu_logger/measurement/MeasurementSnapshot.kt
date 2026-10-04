package com.example.gnss_imu_logger.measurement

/**
 * 画面へ通知する現在の計測状態。
 * 入力: サービス内の最新状態
 * 出力: 状態表示、GNSS品質表示、IMU周期表示
 */
data class MeasurementSnapshot(
    val state: MeasurementState = MeasurementState.IDLE,
    val reason: ReadinessReason = ReadinessReason.NONE,
    val usedSatellites: Int = 0,
    val horizontalAccuracyM: Float? = null,
    val lastLocationAgeMs: Long? = null,
    val accelerometerRateHz: Double = 0.0,
    val gyroscopeRateHz: Double = 0.0,
    val sessionElapsedMs: Long = 0L,
    val stationary: Boolean = false,
    val stationaryDurationMs: Long = 0L,
    val accelerationNormMps2: Double? = null,
    val gyroscopeNormRadps: Double? = null,
    val calibrationActive: Boolean = false,
    val calibrationCompleted: Boolean = false,
    val calibrationElapsedMs: Long = 0L,
    val calibrationAccelerometerSamples: Int = 0,
    val calibrationGyroscopeSamples: Int = 0,
    val correctedGyroscopeNormRadps: Double? = null,
    val initialRollDeg: Double? = null,
    val initialPitchDeg: Double? = null,
    val estimatedRollDeg: Double? = null,
    val rollReferenceDeg: Double? = null,
    val rollDriftCorrectionRadps: Double? = null,
    val leanEstimateValid: Boolean = false,
    val attitudeRollDeg: Double? = null,
    val attitudePitchDeg: Double? = null,
    val attitudeYawDeg: Double? = null,
    val attitudeConfidence: Double? = null,
    val attitudeValid: Boolean = false
)

fun interface MeasurementStateListener {
    fun onSnapshotChanged(snapshot: MeasurementSnapshot)
}
