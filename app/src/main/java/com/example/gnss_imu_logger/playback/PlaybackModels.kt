package com.example.gnss_imu_logger.playback

import java.io.File

/** 保存済みセッション一覧に表示する情報。 */
data class SessionListItem(
    val sessionId: String,
    val directory: File,
    val startUtcNs: Long?,
    val endUtcNs: Long?,
    val durationMs: Long?,
    val mode: String,
    val status: String,
    val gnssCount: Long?,
    val calibrationCompleted: Boolean,
    val hasLeanAngle: Boolean,
    val incomplete: Boolean,
    val errorMessage: String?
)

data class PlaybackLocation(
    val elapsedRealtimeNs: Long,
    val sessionElapsedNs: Long,
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val altitudeM: Double?,
    val speedMps: Double?,
    val bearingDeg: Double?,
    val horizontalAccuracyM: Double?
)

data class PlaybackAttitude(
    val elapsedRealtimeNs: Long,
    val rollRad: Double?,
    val rollReferenceRad: Double?,
    val yawRateRadps: Double?,
    val lateralAccelerationMps2: Double?,
    val valid: Boolean,
    val source: AttitudeSource
)

enum class AttitudeSource {
    RECORDED_LEAN_ANGLE,
    RECALCULATED_LEAN_ANGLE,
    UNAVAILABLE
}

data class PlaybackEvent(
    val elapsedRealtimeNs: Long,
    val sessionElapsedNs: Long,
    val type: String,
    val message: String
)

data class PlaybackSession(
    val item: SessionListItem,
    val startElapsedNs: Long,
    val endElapsedNs: Long,
    val locations: List<PlaybackLocation>,
    val attitudes: List<PlaybackAttitude>,
    val events: List<PlaybackEvent>,
    val diagnostics: PlaybackDiagnostics
)

data class PlaybackDiagnostics(
    val gnssSamples: Int,
    val attitudeSamples: Int,
    val eventSamples: Int,
    val rejectedRows: Int,
    val reversedTimestamps: Int,
    val maximumGnssIntervalMs: Double,
    val gnssGapCount: Int
)

data class PlaybackSample(
    val elapsedRealtimeNs: Long,
    val sessionElapsedNs: Long,
    val location: PlaybackLocation?,
    val attitude: PlaybackAttitude?,
    val latestEvent: PlaybackEvent?
)

internal data class PlaybackIndices(
    val location: Int,
    val attitude: Int,
    val event: Int
)
