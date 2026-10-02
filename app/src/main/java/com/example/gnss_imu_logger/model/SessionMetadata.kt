package com.example.gnss_imu_logger.model

/**
 * セッション全体のメタデータ。
 * 入力: 端末情報、開始・終了時刻、計測モード
 * 出力: session.json
 */
data class SessionMetadata(
    val formatVersion: Int = 1,
    val sessionId: String,
    val status: SessionStatus,
    val mode: LogMode,
    val manufacturer: String,
    val model: String,
    val androidVersion: String,
    val apiLevel: Int,
    val startElapsedNs: Long,
    val startUtcNs: Long,
    val initialUtcOffsetNs: Long,
    val endElapsedNs: Long? = null,
    val endUtcNs: Long? = null,
    val errorMessage: String? = null
)
