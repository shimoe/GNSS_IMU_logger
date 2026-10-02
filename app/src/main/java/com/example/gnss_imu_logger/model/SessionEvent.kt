package com.example.gnss_imu_logger.model

/**
 * 計測中に発生したイベント。
 * 入力: 共通時刻、重要度、イベント種別、人が読む日本語メッセージ
 * 出力: events.csvへ保存する1レコード
 */
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
    SESSION_CREATED,
    SERVICE_STARTED,
    WAKE_LOCK_ACQUIRED,
    SENSOR_REGISTERED,
    SENSOR_UNAVAILABLE,
    GNSS_REGISTERED,
    SCREEN_ON,
    SCREEN_OFF,
    UTC_OFFSET_CHANGED,
    WRITE_ERROR,
    STOP_REQUESTED,
    SESSION_COMPLETED,
    SERVICE_DESTROYED
}
