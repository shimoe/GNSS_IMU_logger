package com.example.gnss_imu_logger.sensor

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.*
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

class GnssCollector(
    private val context: Context,
    private val locationManager: LocationManager,
    private val session: SessionContext,
    private val onFatalError: (String, Throwable?) -> Unit,
    private val onQualityUpdated: (Long, Int, Float?, Int) -> Unit = { _, _, _, _ -> }
) : LocationListener, Closeable {
    private val handler = Handler(Looper.getMainLooper())
    private val stats = GnssStats()
    private val epochId = AtomicLong(0L)
    private var started = false
    private var closed = false
    private var usedSatellites = 0
    private var consecutiveLocationCount = 0
    private val locationWriter = CsvLogWriter(session.file("gnss.csv"), listOf(
        "elapsed_realtime_ns","session_elapsed_ns","utc_ms","latitude_deg","longitude_deg",
        "altitude_m","speed_mps","bearing_deg","horizontal_accuracy_m","vertical_accuracy_m",
        "speed_accuracy_mps","bearing_accuracy_deg","provider","mock"))
    private val statusWriter = CsvLogWriter(session.file("gnss_status.csv"), listOf(
        "elapsed_realtime_ns","session_elapsed_ns","total_satellites","used_satellites"))
    private val rawClockWriter = if (session.mode == LogMode.DIAGNOSTIC) CsvLogWriter(
        session.file("raw_gnss_clock.csv"), listOf("epoch_id","elapsed_realtime_ns",
        "elapsed_realtime_uncertainty_ns","time_ns","time_uncertainty_ns","full_bias_ns","bias_ns",
        "bias_uncertainty_ns","drift_nsps","drift_uncertainty_nsps","hardware_clock_discontinuity_count")) else null
    private val rawMeasurementWriter = if (session.mode == LogMode.DIAGNOSTIC) CsvLogWriter(
        session.file("raw_gnss_measurements.csv"), listOf("epoch_id","constellation_type","svid","code_type",
        "time_offset_ns","state","received_sv_time_ns","received_sv_time_uncertainty_ns","cn0_dbhz",
        "pseudorange_rate_mps","pseudorange_rate_uncertainty_mps","adr_state","adr_m","adr_uncertainty_m",
        "carrier_frequency_hz","multipath_indicator","agc_db")) else null

    private val statusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (i in 0 until status.satelliteCount) if (status.usedInFix(i)) used++
            usedSatellites = used
            val now = android.os.SystemClock.elapsedRealtimeNanos()
            statusWriter.write(listOf(now, session.time.sessionElapsedNs(now), status.satelliteCount, used))
        }
    }
    private val measurementCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val id = epochId.incrementAndGet(); writeClock(id,event.clock)
            event.measurements.forEach { writeMeasurement(id,it) }
        }
    }
    fun start() {
        if (started) return
        if (ActivityCompat.checkSelfPermission(context,Manifest.permission.ACCESS_FINE_LOCATION)!=PackageManager.PERMISSION_GRANTED)
            throw SecurityException("正確な位置情報の権限がありません")
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER,1000L,0f,this,Looper.getMainLooper())
        locationManager.registerGnssStatusCallback(statusCallback,handler)
        if (session.mode==LogMode.DIAGNOSTIC && !locationManager.registerGnssMeasurementsCallback(measurementCallback,handler))
            throw IllegalStateException("RAW GNSSを登録できませんでした")
        session.event(EventLevel.INFO,EventType.GNSS_REGISTERED,"GNSSを登録しました"); started=true
    }
    override fun onLocationChanged(location: Location) {
        try {
            val t=location.elapsedRealtimeNanos; val offset=session.time.updateGnssUtcOffset(t,location.time)
            if(offset.changed) session.event(EventLevel.WARNING,EventType.UTC_OFFSET_CHANGED,"GNSS時刻と端末時刻の差が100 ms以上変化しました")
            stats.add(t); consecutiveLocationCount++
            onQualityUpdated(t,usedSatellites,if(location.hasAccuracy()) location.accuracy else null,consecutiveLocationCount)
            locationWriter.write(listOf(t,session.time.sessionElapsedNs(t),location.time,location.latitude,location.longitude,
                if(location.hasAltitude()) location.altitude else null,if(location.hasSpeed()) location.speed else null,
                if(location.hasBearing()) location.bearing else null,if(location.hasAccuracy()) location.accuracy else null,
                if(location.hasVerticalAccuracy()) location.verticalAccuracyMeters else null,
                if(location.hasSpeedAccuracy()) location.speedAccuracyMetersPerSecond else null,
                if(location.hasBearingAccuracy()) location.bearingAccuracyDegrees else null,location.provider,
                if(Build.VERSION.SDK_INT>=Build.VERSION_CODES.S) location.isMock else location.isFromMockProvider))
            locationWriter.flush()
        } catch(e:Throwable){ onFatalError("GNSS位置ログの書き込みに失敗しました",e) }
    }
    fun snapshot()=stats.snapshot()
    private fun writeClock(id:Long,c:GnssClock){ rawClockWriter?.write(listOf(id,
        if(c.hasElapsedRealtimeNanos()) c.elapsedRealtimeNanos else null,
        if(c.hasElapsedRealtimeUncertaintyNanos()) c.elapsedRealtimeUncertaintyNanos else null,c.timeNanos,
        if(c.hasTimeUncertaintyNanos()) c.timeUncertaintyNanos else null,if(c.hasFullBiasNanos()) c.fullBiasNanos else null,
        if(c.hasBiasNanos()) c.biasNanos else null,if(c.hasBiasUncertaintyNanos()) c.biasUncertaintyNanos else null,
        if(c.hasDriftNanosPerSecond()) c.driftNanosPerSecond else null,
        if(c.hasDriftUncertaintyNanosPerSecond()) c.driftUncertaintyNanosPerSecond else null,c.hardwareClockDiscontinuityCount)) }
    private fun writeMeasurement(id:Long,m:GnssMeasurement){ rawMeasurementWriter?.write(listOf(id,m.constellationType,m.svid,
        if(Build.VERSION.SDK_INT>=Build.VERSION_CODES.Q)m.codeType else null,m.timeOffsetNanos,m.state,m.receivedSvTimeNanos,
        m.receivedSvTimeUncertaintyNanos,m.cn0DbHz,m.pseudorangeRateMetersPerSecond,m.pseudorangeRateUncertaintyMetersPerSecond,
        m.accumulatedDeltaRangeState,m.accumulatedDeltaRangeMeters,m.accumulatedDeltaRangeUncertaintyMeters,
        if(m.hasCarrierFrequencyHz())m.carrierFrequencyHz else null,m.multipathIndicator,
        if(m.hasAutomaticGainControlLevelDb())m.automaticGainControlLevelDb else null)) }
    override fun close(){ if(closed)return;closed=true;if(started){locationManager.removeUpdates(this);locationManager.unregisterGnssStatusCallback(statusCallback);if(session.mode==LogMode.DIAGNOSTIC)locationManager.unregisterGnssMeasurementsCallback(measurementCallback)};locationWriter.close();statusWriter.close();rawClockWriter?.close();rawMeasurementWriter?.close();started=false }
}
