package com.example.gnss_imu_logger.sensor

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.GnssClock
import android.location.GnssMeasurement
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import androidx.core.app.ActivityCompat
import com.example.gnss_imu_logger.log.CsvLogWriter
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import com.example.gnss_imu_logger.model.LogMode
import java.io.Closeable
import java.util.concurrent.atomic.AtomicLong

/**
 * GNSS位置、衛星状態、RAW GNSSをセッション内へ保存する。
 * 入力: LocationManager、計測モード、共通時刻基準
 * 出力: gnss.csv、gnss_status.csv、診断モード時のRAW GNSS CSV
 */
class GnssCollector(
    private val context: Context,
    private val locationManager: LocationManager,
    private val session: SessionContext,
    private val onFatalError: (String, Throwable?) -> Unit
) : LocationListener, Closeable {
    private val handler = Handler(Looper.getMainLooper())
    private val stats = GnssStats()
    private val epochId = AtomicLong(0L)
    private var started = false
    private var closed = false

    private val locationWriter = CsvLogWriter(
        session.file("gnss.csv"),
        listOf(
            "elapsed_realtime_ns", "session_elapsed_ns", "utc_ms",
            "latitude_deg", "longitude_deg", "altitude_m",
            "speed_mps", "bearing_deg", "horizontal_accuracy_m",
            "vertical_accuracy_m", "speed_accuracy_mps",
            "bearing_accuracy_deg", "provider", "mock"
        )
    )
    private val statusWriter = CsvLogWriter(
        session.file("gnss_status.csv"),
        listOf(
            "elapsed_realtime_ns", "session_elapsed_ns",
            "total_satellites", "used_satellites"
        )
    )
    private val rawClockWriter = if (session.mode == LogMode.DIAGNOSTIC) {
        CsvLogWriter(
            session.file("raw_gnss_clock.csv"),
            listOf(
                "epoch_id", "elapsed_realtime_ns",
                "elapsed_realtime_uncertainty_ns", "time_ns",
                "time_uncertainty_ns", "full_bias_ns", "bias_ns",
                "bias_uncertainty_ns", "drift_nsps",
                "drift_uncertainty_nsps",
                "hardware_clock_discontinuity_count"
            )
        )
    } else null
    private val rawMeasurementWriter = if (session.mode == LogMode.DIAGNOSTIC) {
        CsvLogWriter(
            session.file("raw_gnss_measurements.csv"),
            listOf(
                "epoch_id", "constellation_type", "svid", "code_type",
                "time_offset_ns", "state", "received_sv_time_ns",
                "received_sv_time_uncertainty_ns", "cn0_dbhz",
                "pseudorange_rate_mps", "pseudorange_rate_uncertainty_mps",
                "adr_state", "adr_m", "adr_uncertainty_m",
                "carrier_frequency_hz", "multipath_indicator", "agc_db"
            )
        )
    } else null

    private val statusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (index in 0 until status.satelliteCount) {
                if (status.usedInFix(index)) used++
            }
            val elapsedNs = android.os.SystemClock.elapsedRealtimeNanos()
            statusWriter.write(
                listOf(
                    elapsedNs,
                    session.time.sessionElapsedNs(elapsedNs),
                    status.satelliteCount,
                    used
                )
            )
        }
    }

    private val measurementCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val id = epochId.incrementAndGet()
            writeClock(id, event.clock)
            event.measurements.forEach { writeMeasurement(id, it) }
        }
    }

    fun start() {
        if (started) return
        if (ActivityCompat.checkSelfPermission(
                context,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            throw SecurityException("正確な位置情報の権限がありません")
        }
        try {
            locationManager.requestLocationUpdates(
                LocationManager.GPS_PROVIDER,
                LOCATION_PERIOD_MS,
                0f,
                this,
                Looper.getMainLooper()
            )
            locationManager.registerGnssStatusCallback(statusCallback, handler)
            if (session.mode == LogMode.DIAGNOSTIC) {
                val registered = locationManager.registerGnssMeasurementsCallback(
                    measurementCallback,
                    handler
                )
                if (!registered) {
                    throw IllegalStateException("RAW GNSSを登録できませんでした")
                }
            }
            session.event(
                EventLevel.INFO,
                EventType.GNSS_REGISTERED,
                "GNSSを登録しました"
            )
            started = true
        } catch (error: Throwable) {
            close()
            throw error
        }
    }

    override fun onLocationChanged(location: Location) {
        try {
            val elapsedNs = location.elapsedRealtimeNanos
            val offset = session.time.updateUtcOffset(elapsedNs, location.time)
            if (offset.changed) {
                session.event(
                    EventLevel.WARNING,
                    EventType.UTC_OFFSET_CHANGED,
                    "GNSS時刻と端末時刻の差が100 ms以上変化しました"
                )
            }
            stats.add(elapsedNs)
            locationWriter.write(
                listOf(
                    elapsedNs,
                    session.time.sessionElapsedNs(elapsedNs),
                    location.time,
                    location.latitude,
                    location.longitude,
                    if (location.hasAltitude()) location.altitude else null,
                    if (location.hasSpeed()) location.speed else null,
                    if (location.hasBearing()) location.bearing else null,
                    if (location.hasAccuracy()) location.accuracy else null,
                    if (location.hasVerticalAccuracy()) {
                        location.verticalAccuracyMeters
                    } else null,
                    if (location.hasSpeedAccuracy()) {
                        location.speedAccuracyMetersPerSecond
                    } else null,
                    if (location.hasBearingAccuracy()) {
                        location.bearingAccuracyDegrees
                    } else null,
                    location.provider,
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                        location.isMock
                    } else {
                        @Suppress("DEPRECATION")
                        location.isFromMockProvider
                    }
                )
            )
            locationWriter.flush()
        } catch (error: Throwable) {
            onFatalError("GNSS位置ログの書き込みに失敗しました", error)
        }
    }

    fun snapshot(): GnssStats.Snapshot = stats.snapshot()

    private fun writeClock(id: Long, clock: GnssClock) {
        rawClockWriter?.write(
            listOf(
                id,
                if (clock.hasElapsedRealtimeNanos()) {
                    clock.elapsedRealtimeNanos
                } else null,
                if (clock.hasElapsedRealtimeUncertaintyNanos()) {
                    clock.elapsedRealtimeUncertaintyNanos
                } else null,
                clock.timeNanos,
                if (clock.hasTimeUncertaintyNanos()) {
                    clock.timeUncertaintyNanos
                } else null,
                if (clock.hasFullBiasNanos()) clock.fullBiasNanos else null,
                if (clock.hasBiasNanos()) clock.biasNanos else null,
                if (clock.hasBiasUncertaintyNanos()) {
                    clock.biasUncertaintyNanos
                } else null,
                if (clock.hasDriftNanosPerSecond()) {
                    clock.driftNanosPerSecond
                } else null,
                if (clock.hasDriftUncertaintyNanosPerSecond()) {
                    clock.driftUncertaintyNanosPerSecond
                } else null,
                clock.hardwareClockDiscontinuityCount
            )
        )
    }

    private fun writeMeasurement(id: Long, measurement: GnssMeasurement) {
        rawMeasurementWriter?.write(
            listOf(
                id,
                measurement.constellationType,
                measurement.svid,
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                    measurement.codeType
                } else null,
                measurement.timeOffsetNanos,
                measurement.state,
                measurement.receivedSvTimeNanos,
                measurement.receivedSvTimeUncertaintyNanos,
                measurement.cn0DbHz,
                measurement.pseudorangeRateMetersPerSecond,
                measurement.pseudorangeRateUncertaintyMetersPerSecond,
                measurement.accumulatedDeltaRangeState,
                measurement.accumulatedDeltaRangeMeters,
                measurement.accumulatedDeltaRangeUncertaintyMeters,
                if (measurement.hasCarrierFrequencyHz()) {
                    measurement.carrierFrequencyHz
                } else null,
                measurement.multipathIndicator,
                if (measurement.hasAutomaticGainControlLevelDb()) {
                    measurement.automaticGainControlLevelDb
                } else null
            )
        )
    }

    override fun close() {
        if (closed) return
        closed = true
        if (started) {
            locationManager.removeUpdates(this)
            locationManager.unregisterGnssStatusCallback(statusCallback)
            if (session.mode == LogMode.DIAGNOSTIC) {
                locationManager.unregisterGnssMeasurementsCallback(
                    measurementCallback
                )
            }
        }
        locationWriter.close()
        statusWriter.close()
        rawClockWriter?.close()
        rawMeasurementWriter?.close()
        started = false
    }

    companion object {
        private const val LOCATION_PERIOD_MS = 1_000L
    }
}
