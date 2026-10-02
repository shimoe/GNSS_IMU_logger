package com.example.gnss_imu_logger

import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Binder
import android.os.Build
import android.os.IBinder
import android.os.Looper
import android.os.PowerManager
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import java.util.concurrent.atomic.AtomicBoolean

class LoggerService : Service(), SensorEventListener, LocationListener {
    companion object {
        const val ACTION_START = "logger.START"
        const val ACTION_STOP = "logger.STOP"
        private const val CHANNEL_ID = "measurement"
        private const val NOTIFICATION_ID = 1001
        private const val SENSOR_PERIOD_US = 5_000 // 200 Hz要求
        private const val SENSOR_LATENCY_US = 100_000 // 100 ms以内でまとめて受信
    }

    private lateinit var sensorManager: SensorManager
    private lateinit var locationManager: LocationManager
    private lateinit var writer: LogWriter
    private var wakeLock: PowerManager.WakeLock? = null
    private val running = AtomicBoolean(false)

    private val rawCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            event.measurements.forEach { m ->
                writer.rawGnss(
                    event.clock.timeNanos,
                    m.constellationType,
                    m.svid,
                    m.cn0DbHz,
                    m.pseudorangeRateMetersPerSecond,
                    if (m.hasCarrierFrequencyHz()) m.carrierFrequencyHz else null,
                    m.accumulatedDeltaRangeState,
                    m.accumulatedDeltaRangeMeters
                )
            }
        }
    }

    private val statusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (i in 0 until status.satelliteCount) if (status.usedInFix(i)) used++
            writer.gnssStatus(status.satelliteCount, used)
        }
    }

    override fun onCreate() {
        super.onCreate()
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        locationManager = getSystemService(LOCATION_SERVICE) as LocationManager
        createChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_STOP -> stopLogging()
            else -> startLogging()
        }
        return START_NOT_STICKY
    }

    private fun startLogging() {
        if (!running.compareAndSet(false, true)) return
        startForeground(
            NOTIFICATION_ID,
            NotificationCompat.Builder(this, CHANNEL_ID)
                .setSmallIcon(android.R.drawable.ic_menu_mylocation)
                .setContentTitle("GNSS・IMU計測中")
                .setContentText("画面オフ中も計測を継続しています")
                .setOngoing(true)
                .build()
        )

        writer = LogWriter(this)
        writer.writeSensorInfo(sensorManager.getSensorList(Sensor.TYPE_ALL))
        acquireWakeLock()
        registerSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        registerSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        registerSensor(Sensor.TYPE_ACCELEROMETER)
        registerSensor(Sensor.TYPE_GYROSCOPE)
        registerSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
        registerSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED, 10_000)
        registerSensor(Sensor.TYPE_PRESSURE, 40_000)
        registerGnss()
    }

    private fun registerSensor(type: Int, periodUs: Int = SENSOR_PERIOD_US) {
        sensorManager.getDefaultSensor(type)?.let {
            sensorManager.registerListener(this, it, periodUs, SENSOR_LATENCY_US)
        }
    }

    private fun registerGnss() {
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            writer.error("正確な位置情報の権限がありません")
            stopLogging()
            return
        }
        locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            1_000L,
            0f,
            this,
            Looper.getMainLooper()
        )
        locationManager.registerGnssMeasurementsCallback(rawCallback, android.os.Handler(Looper.getMainLooper()))
        locationManager.registerGnssStatusCallback(statusCallback, android.os.Handler(Looper.getMainLooper()))
    }

    override fun onSensorChanged(event: SensorEvent) {
        writer.sensor(event.sensor.type, event.timestamp, event.accuracy, event.values)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onLocationChanged(location: Location) {
        writer.location(location)
    }

    private fun stopLogging() {
        if (!running.compareAndSet(true, false)) {
            stopSelf()
            return
        }
        sensorManager.unregisterListener(this)
        locationManager.removeUpdates(this)
        locationManager.unregisterGnssMeasurementsCallback(rawCallback)
        locationManager.unregisterGnssStatusCallback(statusCallback)
        wakeLock?.takeIf { it.isHeld }?.release()
        writer.close()
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun acquireWakeLock() {
        val pm = getSystemService(POWER_SERVICE) as PowerManager
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "GNSS_IMU_logger:measurement").apply {
            setReferenceCounted(false)
            acquire()
        }
    }

    private fun createChannel() {
        val channel = NotificationChannel(CHANNEL_ID, "計測状態", NotificationManager.IMPORTANCE_LOW)
        getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
    }

    override fun onDestroy() {
        if (running.get()) stopLogging()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder = Binder()
}
