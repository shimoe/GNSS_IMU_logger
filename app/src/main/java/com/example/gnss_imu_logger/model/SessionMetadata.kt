package com.example.gnss_imu_logger.model

/**
 * セッション全体のメタデータ。
 * 入力: 端末情報、開始・終了時刻、計測モード
 * 出力: session.json
 */
data class SessionMetadata(
    val formatVersion: Int = 2,
    val sessionId: String,
    val status: SessionStatus,
    val mode: LogMode,
    val deviceMount: String,
    val leanAngleParameters: LeanAngleParameterMetadata,
    val manufacturer: String,
    val model: String,
    val androidVersion: String,
    val apiLevel: Int,
    val startElapsedNs: Long,
    val startUtcNs: Long,
    val initialUtcOffsetNs: Long,
    val endElapsedNs: Long? = null,
    val endUtcNs: Long? = null,
    val errorMessage: String? = null,
    val summary: SessionSummary? = null
)


/**
 * 単車ロール推定を再現するために保存するパラメーター。
 * 入力: LeanAngleParameters
 * 出力: session.jsonのlean_angle_parameters
 */
data class LeanAngleParameterMetadata(
    val parameterSetId: String,
    val lowPassCutoffHz: Double,
    val minimumCentripetalSpeedMps: Double,
    val minimumReferenceConfidence: Double,
    val centripetalEntrySpeedMps: Double,
    val centripetalMaintainSpeedMps: Double,
    val centripetalEntryConfidence: Double,
    val centripetalMaintainConfidence: Double,
    val centripetalEntryDurationSec: Double,
    val centripetalExitDurationSec: Double,
    val shortHoldDurationSec: Double,
    val fullSpeedAccelerationMps2: Double,
    val maximumSpeedAccelerationMps2: Double,
    val fullYawAccelerationRadps2: Double,
    val maximumYawAccelerationRadps2: Double,
    val fullRollRateRadps: Double,
    val maximumRollRateRadps: Double,
    val minimumWindowSec: Double,
    val windowRetentionSec: Double,
    val maximumSpeedAgeSec: Double,
    val minimumSpeedIntervalSec: Double,
    val maximumSpeedIntervalSec: Double,
    val maximumAccelerationIntervalSec: Double,
    val maximumGyroscopeIntervalSec: Double,
    val correctionFilterAlpha: Double,
    val dynamicsFilterAlpha: Double,
    val holdCorrectionDecaySec: Double,
    val maximumGravityErrorMps2: Double,
    val gravityRecoveryTimeSec: Double,
    val maximumGravityRecoveryRateRadps: Double
)
