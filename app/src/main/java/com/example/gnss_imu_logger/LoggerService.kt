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
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.PowerManager
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.sensor.ImuCollector
import java.util.concurrent.atomic.AtomicBoolean

class LoggerService : Service(), SensorEventListener, LocationListener {
    companion object {
        const val ACTION_START = "logger.START"
        const val ACTION_STOP = "logger.STOP"
        private const val CHANNEL_ID = "measurement"
        private const val NOTIFICATION_ID = 1001
        private const val SENSOR_PERIOD_US = 5_000
    }

    private lateinit var sensorManager: SensorManager
    private lateinit var locationManager: LocationManager
    private lateinit var writer: LogWriter
    private val mainHandler = Handler(Looper.getMainLooper())
    private var wakeLock: PowerManager.WakeLock? = null
    private var session: SessionContext? = null
    private var imuCollector: ImuCollector? = null
    private val running = AtomicBoolean(false)
    private val stopping = AtomicBoolean(false)

    private val rawCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            event.measurements.forEach { measurement ->
                writer.rawGnss(
                    event.clock.timeNanos,
                    measurement.constellationType,
                    measurement.svid,
                    measurement.cn0DbHz,
                    measurement.pseudorangeRateMetersPerSecond,
                    if (measurement.hasCarrierFrequencyHz()) {
                        measurement.carrierFrequencyHz
                    } else null,
                    measurement.accumulatedDeltaRangeState,
                    measurement.accumulatedDeltaRangeMeters
                )
            }
        }
    }

    private val statusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (index in 0 until status.satelliteCount) {
                if (status.usedInFix(index)) used++
            }
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
            ACTION_START -> startLogging()
        }
        return START_NOT_STICKY
    }

    private fun startLogging() {
        if (!running.compareAndSet(false, true)) return
        stopping.set(false)
        try {
            startForeground(NOTIFICATION_ID, createNotification())
            val currentSession = SessionContext(this, LogMode.DIAGNOSTIC)
            session = currentSession
            currentSession.event(
                EventLevel.INFO,
                EventType.SERVICE_STARTED,
                "計測サービスを開始しました"
            )

            writer = LogWriter(this)
            writer.writeSensorInfo(sensorManager.getSensorList(Sensor.TYPE_ALL))
            acquireWakeLock()

            imuCollector = ImuCollector(
                sensorManager = sensorManager,
                session = currentSession,
                onFatalError = { message, error ->
                    mainHandler.post { handleFatalError(message, error) }
                }
            ).also { it.start() }

            // 比較用センサーと環境センサーは、GNSSログ移行まで旧CSVへ保存する。
            registerLegacySensor(Sensor.TYPE_ACCELEROMETER)
            registerLegacySensor(Sensor.TYPE_GYROSCOPE)
            registerLegacySensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
            registerLegacySensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED, 10_000)
            registerLegacySensor(Sensor.TYPE_PRESSURE, 40_000)
            registerGnss()
        } catch (error: Throwable) {
            handleFatalError("計測を開始できませんでした", error)
        }
    }

    private fun registerLegacySensor(type: Int, periodUs: Int = SENSOR_PERIOD_US) {
        sensorManager.getDefaultSensor(type)?.let { sensor ->
            if (!sensorManager.registerListener(this, sensor, periodUs, 100_000)) {
                throw IllegalStateException("${sensor.name}を登録できませんでした")
            }
        }
    }

    private fun registerGnss() {
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_FINE_LOCATION
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            throw SecurityException("正確な位置情報の権限がありません")
        }
        locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            1_000L,
            0f,
            this,
            Looper.getMainLooper()
        )
        locationManager.registerGnssMeasurementsCallback(rawCallback, mainHandler)
        locationManager.registerGnssStatusCallback(statusCallback, mainHandler)
        session?.event(
            EventLevel.INFO,
            EventType.GNSS_REGISTERED,
            "GNSSを登録しました"
        )
    }

    override fun onSensorChanged(event: SensorEvent) {
        // 未校正加速度と未校正ジャイロはImuCollectorで保存する。
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
        if (!stopping.compareAndSet(false, true)) return

        session?.event(
            EventLevel.INFO,
            EventType.STOP_REQUESTED,
            "計測停止を受け付けました"
        )
        sensorManager.unregisterListener(this)
        locationManager.removeUpdates(this)
        locationManager.unregisterGnssMeasurementsCallback(rawCallback)
        locationManager.unregisterGnssStatusCallback(statusCallback)

        imuCollector?.close()
        imuCollector = null
        if (::writer.isInitialized) writer.close()

        session?.complete()
        session?.close()
        session = null

        wakeLock?.takeIf { it.isHeld }?.release()
        wakeLock = null
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun handleFatalError(message: String, error: Throwable?) {
        if (stopping.get()) return
        val detail = error?.message?.takeIf { it.isNotBlank() }
        val fullMessage = if (detail == null) message else "$message: $detail"
        runCatching { session?.fail(fullMessage) }
        stopLogging()
    }

    private fun acquireWakeLock() {
        val powerManager = getSystemService(POWER_SERVICE) as PowerManager
        wakeLock = powerManager.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "GNSS_IMU_logger:measurement"
        ).apply {
            setReferenceCounted(false)
            acquire()
        }
        session?.event(
            EventLevel.INFO,
            EventType.WAKE_LOCK_ACQUIRED,
            "画面オフ計測用のWakeLockを取得しました"
        )
    }

    private fun createNotification() = NotificationCompat.Builder(this, CHANNEL_ID)
        .setSmallIcon(android.R.drawable.ic_menu_mylocation)
        .setContentTitle("GNSS・IMU計測中")
        .setContentText("画面オフ中も計測を継続しています")
        .setOngoing(true)
        .build()

    private fun createChannel() {
        val channel = NotificationChannel(
            CHANNEL_ID,
            "計測状態",
            NotificationManager.IMPORTANCE_LOW
        )
        getSystemService(NotificationManager::class.java)
            .createNotificationChannel(channel)
    }

    override fun onDestroy() {
        if (running.get()) stopLogging()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder = Binder()
}
