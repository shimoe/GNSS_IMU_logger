package com.example.gnss_imu_logger.navigation

/**
 * スタートラインとラップ成立条件。
 * 入力: 緯度・経度[deg]、距離[m]、速度[m/s]、時間[s]
 * 出力: LapDetectorが使用する不変な設定
 */
data class LapDetectionConfig(
    val lineStart: GeodeticPosition,
    val lineEnd: GeodeticPosition,
    val allowedDirection: LapCrossingDirection,
    val minimumSpeedMps: Double = 3.0,
    val minimumLapDurationSec: Double = 30.0,
    val rearmDistanceM: Double = 40.0,
    val maximumPositionUncertaintyM: Double = 20.0,
    val lineEndExtensionM: Double = 10.0,
    val maximumUncertaintyExtensionM: Double = 15.0,
    val minimumNormalSpeedMps: Double = 1.0
) {
    init {
        require(minimumSpeedMps >= 0.0) { "最低通過速度は0 m/s以上にしてください。" }
        require(minimumLapDurationSec >= 0.0) { "最低ラップ時間は0秒以上にしてください。" }
        require(rearmDistanceM > 0.0) { "再アーム距離は0 mより大きくしてください。" }
        require(maximumPositionUncertaintyM > 0.0) { "位置不確かさ上限は0 mより大きくしてください。" }
        require(lineEndExtensionM >= 0.0) { "ライン端の許容延長は0 m以上にしてください。" }
        require(maximumUncertaintyExtensionM >= 0.0) {
            "位置不確かさによる延長上限は0 m以上にしてください。"
        }
        require(minimumNormalSpeedMps >= 0.0) { "最低法線速度は0 m/s以上にしてください。" }
    }
}

enum class LapCrossingDirection { LEFT_TO_RIGHT, RIGHT_TO_LEFT }
enum class LapType { OUT_LAP, TIMED_LAP, IN_LAP, INVALID_LAP }
enum class LapDetectionState { WAITING_FOR_START, ARMED, LOCKED_OUT }
enum class LapPositionSource { OFFLINE_NAVIGATION }

data class LapRecord(
    val lapIndex: Int,
    val lapType: LapType,
    val startElapsedRealtimeNs: Long,
    val finishElapsedRealtimeNs: Long,
    val durationSec: Double,
    val distanceM: Double,
    val averageSpeedMps: Double,
    val crossingSpeedMps: Double?,
    val positionUncertaintyM: Double?,
    val positionSource: LapPositionSource,
    val valid: Boolean,
    val invalidReason: String,
    val durationOutlier: Boolean
)

data class LapEventRecord(
    val elapsedRealtimeNs: Long,
    val stateBefore: LapDetectionState,
    val stateAfter: LapDetectionState,
    val signedDistanceBeforeM: Double,
    val signedDistanceAfterM: Double,
    val lineRatio: Double?,
    val crossedInfiniteLine: Boolean,
    val endpointDistanceM: Double?,
    val baseExtensionM: Double,
    val uncertaintyExtensionM: Double,
    val effectiveExtensionM: Double,
    val crossingSpeedMps: Double,
    val normalSpeedMps: Double,
    val positionUncertaintyM: Double?,
    val accepted: Boolean,
    val rejectionReason: String
)

data class LapAnalysisResult(
    val records: List<LapRecord>,
    val events: List<LapEventRecord>
)
