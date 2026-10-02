package com.example.gnss_imu_logger

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import android.os.SystemClock
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.time.Instant
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.util.Locale

class LogWriter(context: Context) {
    private val lock = Any()
    private val dir = File(context.getExternalFilesDir(null), "Documents").apply { mkdirs() }
    private val session = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneOffset.UTC).format(Instant.now())
    private val sensorOut = open("${session}_sensor.csv", "type,timestamp_ns,accuracy,v0,v1,v2,v3,v4,v5")
    private val locationOut = open("${session}_location.csv", "elapsed_realtime_ns,utc_ms,lat_deg,lon_deg,alt_m,speed_mps,bearing_deg,h_acc_m,v_acc_m,s_acc_m")
    private val rawOut = open("${session}_raw_gnss.csv", "clock_time_ns,constellation,svid,cn0_dbhz,pseudorange_rate_mps,carrier_hz,adr_state,adr_m")
    private val statusOut = open("${session}_status.csv", "elapsed_realtime_ns,total_satellites,used_satellites")
    private val infoOut = open("${session}_sensor_info.csv", "name,vendor,type,min_delay_us,max_delay_us,max_range,resolution,fifo_max,fifo_reserved,wakeup")
    private val errorOut = open("${session}_error.log", "utc,message")

    private fun open(name: String, header: String): BufferedWriter =
        BufferedWriter(FileWriter(File(dir, name), true), 64 * 1024).apply { appendLine(header) }

    fun writeSensorInfo(sensors: List<Sensor>) = synchronized(lock) {
        sensors.forEach { s ->
            infoOut.appendLine(csv(s.name, s.vendor, s.type, s.minDelay, s.maxDelay, s.maximumRange, s.resolution, s.fifoMaxEventCount, s.fifoReservedEventCount, s.isWakeUpSensor))
        }
        infoOut.flush()
    }

    fun sensor(type: Int, timestampNs: Long, accuracy: Int, values: FloatArray) = synchronized(lock) {
        val v = Array(6) { i -> values.getOrNull(i)?.toString() ?: "" }
        sensorOut.appendLine(csv(type, timestampNs, accuracy, *v))
    }

    fun location(l: Location) = synchronized(lock) {
        locationOut.appendLine(csv(
            l.elapsedRealtimeNanos, l.time, l.latitude, l.longitude,
            if (l.hasAltitude()) l.altitude else "",
            if (l.hasSpeed()) l.speed else "",
            if (l.hasBearing()) l.bearing else "",
            if (l.hasAccuracy()) l.accuracy else "",
            if (l.hasVerticalAccuracy()) l.verticalAccuracyMeters else "",
            if (l.hasSpeedAccuracy()) l.speedAccuracyMetersPerSecond else ""
        ))
        locationOut.flush()
    }

    fun rawGnss(clockNs: Long, constellation: Int, svid: Int, cn0: Double, rate: Double, carrier: Float?, adrState: Int, adr: Double) = synchronized(lock) {
        rawOut.appendLine(csv(clockNs, constellation, svid, cn0, rate, carrier ?: "", adrState, adr))
    }

    fun gnssStatus(total: Int, used: Int) = synchronized(lock) {
        statusOut.appendLine(csv(SystemClock.elapsedRealtimeNanos(), total, used))
    }

    fun error(message: String) = synchronized(lock) {
        errorOut.appendLine(csv(Instant.now(), message))
        errorOut.flush()
    }

    fun close() = synchronized(lock) {
        listOf(sensorOut, locationOut, rawOut, statusOut, infoOut, errorOut).forEach {
            it.flush()
            it.close()
        }
    }

    private fun csv(vararg values: Any?): String = values.joinToString(",") { value ->
        val text = value?.toString()?.replace("\"", "\"\"") ?: ""
        "\"$text\""
    }
}
