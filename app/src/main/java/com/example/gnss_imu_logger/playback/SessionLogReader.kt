package com.example.gnss_imu_logger.playback

import java.io.File
import kotlin.math.max

/**
 * セッションのGNSS、記録済みロール、イベントを読み込む。
 * 入力: SessionListItem
 * 出力: elapsedRealtimeNsで整列したPlaybackSessionとログ診断値
 */
class SessionLogReader {
    fun read(item: SessionListItem): PlaybackSession {
        val locations = mutableListOf<PlaybackLocation>()
        val attitudes = mutableListOf<PlaybackAttitude>()
        val events = mutableListOf<PlaybackEvent>()
        var rejectedRows = 0
        var reversedTimestamps = 0

        readRows(File(item.directory, "gnss.csv")) { row ->
            val value = runCatching { row.toLocation() }.getOrNull()
            if (value == null) rejectedRows++ else locations += value
        }
        readRows(File(item.directory, "lean_angle.csv")) { row ->
            val value = runCatching { row.toAttitude() }.getOrNull()
            if (value == null) rejectedRows++ else attitudes += value
        }
        readRows(File(item.directory, "events.csv")) { row ->
            val value = runCatching { row.toEvent() }.getOrNull()
            if (value == null) rejectedRows++ else events += value
        }

        reversedTimestamps += locations.countReversed { it.elapsedRealtimeNs }
        reversedTimestamps += attitudes.countReversed { it.elapsedRealtimeNs }
        reversedTimestamps += events.countReversed { it.elapsedRealtimeNs }
        locations.sortBy { it.elapsedRealtimeNs }
        attitudes.sortBy { it.elapsedRealtimeNs }
        events.sortBy { it.elapsedRealtimeNs }

        val first = listOfNotNull(
            locations.firstOrNull()?.elapsedRealtimeNs,
            attitudes.firstOrNull()?.elapsedRealtimeNs,
            events.firstOrNull()?.elapsedRealtimeNs
        ).minOrNull() ?: error("再生できるデータがありません")
        val last = listOfNotNull(
            locations.lastOrNull()?.elapsedRealtimeNs,
            attitudes.lastOrNull()?.elapsedRealtimeNs,
            events.lastOrNull()?.elapsedRealtimeNs
        ).maxOrNull() ?: first

        var maximumGnssIntervalNs = 0L
        var gnssGapCount = 0
        locations.zipWithNext().forEach { (previous, current) ->
            val intervalNs = current.elapsedRealtimeNs - previous.elapsedRealtimeNs
            maximumGnssIntervalNs = max(maximumGnssIntervalNs, intervalNs)
            if (intervalNs > GNSS_GAP_NS) gnssGapCount++
        }

        return PlaybackSession(
            item = item,
            startElapsedNs = first,
            endElapsedNs = last,
            locations = locations,
            attitudes = attitudes,
            events = events,
            diagnostics = PlaybackDiagnostics(
                gnssSamples = locations.size,
                attitudeSamples = attitudes.size,
                eventSamples = events.size,
                rejectedRows = rejectedRows,
                reversedTimestamps = reversedTimestamps,
                maximumGnssIntervalMs = maximumGnssIntervalNs / NS_PER_MS,
                gnssGapCount = gnssGapCount
            )
        )
    }

    private fun readRows(file: File, block: (Map<String, String>) -> Unit) {
        if (file.isFile) CsvRowReader(file).forEachRow(block)
    }

    private fun Map<String, String>.toLocation(): PlaybackLocation {
        val latitude = requiredDouble("latitude_deg")
        val longitude = requiredDouble("longitude_deg")
        require(latitude.isFinite() && longitude.isFinite())
        require(latitude in -90.0..90.0 && longitude in -180.0..180.0)
        return PlaybackLocation(
            elapsedRealtimeNs = requiredLong("elapsed_realtime_ns"),
            sessionElapsedNs = requiredLong("session_elapsed_ns"),
            latitudeDeg = latitude,
            longitudeDeg = longitude,
            altitudeM = optionalDouble("altitude_m"),
            speedMps = optionalDouble("speed_mps"),
            bearingDeg = optionalDouble("bearing_deg"),
            horizontalAccuracyM = optionalDouble("horizontal_accuracy_m")
        )
    }

    private fun Map<String, String>.toAttitude(): PlaybackAttitude {
        val roll = optionalDouble("roll_rad")
        val reference = optionalDouble("reference_roll_rad")
        val yawRate = optionalDouble("yaw_rate_radps")
        val lateralAcceleration = optionalDouble("centripetal_acceleration_mps2")
        require(roll == null || roll.isFinite())
        require(reference == null || reference.isFinite())
        require(yawRate == null || yawRate.isFinite())
        require(lateralAcceleration == null || lateralAcceleration.isFinite())
        return PlaybackAttitude(
            elapsedRealtimeNs = requiredLong("elapsed_realtime_ns"),
            rollRad = roll,
            rollReferenceRad = reference,
            yawRateRadps = yawRate,
            lateralAccelerationMps2 = lateralAcceleration,
            referenceSource = get("reference_source")?.takeIf { it.isNotBlank() },
            valid = get("estimate_valid")?.toBooleanStrictOrNull() ?: false,
            source = AttitudeSource.RECORDED_LEAN_ANGLE
        )
    }

    private fun Map<String, String>.toEvent() = PlaybackEvent(
        elapsedRealtimeNs = requiredLong("elapsed_realtime_ns"),
        sessionElapsedNs = requiredLong("session_elapsed_ns"),
        type = get("event_type").orEmpty(),
        message = get("message").orEmpty()
    )

    private fun Map<String, String>.requiredLong(name: String): Long =
    get(name)?.toLongOrNull()?: error("${name}が不正です")
    private fun Map<String, String>.requiredDouble(name: String): Double =
    optionalDouble(name)?: error("${name}が不正です")
    private fun Map<String, String>.optionalDouble(name: String): Double? =
        get(name)?.takeIf { it.isNotBlank() }?.toDoubleOrNull()

    private fun <T> List<T>.countReversed(timestampNs: (T) -> Long): Int {
        var count = 0
        var previous: Long? = null
        forEach { value ->
            val current = timestampNs(value)
            if (previous != null && current < requireNotNull(previous)) count++
            previous = current
        }
        return count
    }

    companion object {
        private const val GNSS_GAP_NS = 3_000_000_000L
        private const val NS_PER_MS = 1_000_000.0
    }
}
