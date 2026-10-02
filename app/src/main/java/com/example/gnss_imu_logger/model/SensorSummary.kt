package com.example.gnss_imu_logger.model

import com.example.gnss_imu_logger.measurement.MeasurementStateSummary

/**
 * 計測終了時に保存するセンサー統計。
 * 入力: Collectorが集計した受信・書き込み統計
 * 出力: session.jsonのstatistics
 */
data class SensorSummary(
    val receivedCount: Long,
    val writtenCount: Long,
    val droppedCount: Long,
    val estimatedMissingCount: Long,
    val meanRateHz: Double,
    val medianIntervalMs: Double,
    val maximumIntervalMs: Double
)

data class GnssSummary(
    val count: Long,
    val meanRateHz: Double,
    val maximumIntervalMs: Double
)

data class SessionSummary(
    val accelerometer: SensorSummary,
    val gyroscope: SensorSummary,
    val gnss: GnssSummary,
    val totalBytes: Long,
    val freeBytesAtEnd: Long,
    val measurement: MeasurementStateSummary
)
