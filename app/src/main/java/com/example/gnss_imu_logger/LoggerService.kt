package com.example.gnss_imu_logger

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.hardware.SensorManager
import android.location.LocationManager
import android.os.Binder
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.PowerManager
import androidx.core.app.NotificationCompat
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import com.example.gnss_imu_logger.model.GnssSummary
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.model.SensorSummary
import com.example.gnss_imu_logger.model.SessionSummary
import com.example.gnss_imu_logger.sensor.EnvironmentCollector
import com.example.gnss_imu_logger.sensor.GnssCollector
import com.example.gnss_imu_logger.sensor.ImuCollector
import com.example.gnss_imu_logger.storage.IncompleteSessionScanner
import com.example.gnss_imu_logger.storage.StorageMonitor
import java.io.File
import java.util.concurrent.atomic.AtomicBoolean

class LoggerService : Service() {
    companion object {
        const val ACTION_START = "logger.START"
        const val ACTION_STOP = "logger.STOP"
        const val EXTRA_LOG_MODE = "logger.LOG_MODE"
        private const val CHANNEL_ID = "measurement"
        private const val NOTIFICATION_ID = 1001
    }

    private lateinit var sensorManager: SensorManager
    private lateinit var locationManager: LocationManager
    private val mainHandler = Handler(Looper.getMainLooper())
    private var wakeLock: PowerManager.WakeLock? = null
    private var session: SessionContext? = null
    private var imuCollector: ImuCollector? = null
    private var gnssCollector: GnssCollector? = null
    private var environmentCollector: EnvironmentCollector? = null
    private var storageMonitor: StorageMonitor? = null
    private val running = AtomicBoolean(false)
    private val stopping = AtomicBoolean(false)

    override fun onCreate() {
        super.onCreate()
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        locationManager = getSystemService(LOCATION_SERVICE) as LocationManager
        createChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> startLogging(parseMode(intent))
            ACTION_STOP -> stopLogging()
        }
        return START_NOT_STICKY
    }

    private fun parseMode(intent: Intent): LogMode = runCatching {
        LogMode.valueOf(intent.getStringExtra(EXTRA_LOG_MODE) ?: LogMode.NORMAL.name)
    }.getOrDefault(LogMode.NORMAL)

    private fun startLogging(mode: LogMode) {
        if (!running.compareAndSet(false, true)) return
        stopping.set(false)
        try {
            startForeground(NOTIFICATION_ID, createNotification())
            val documents = File(getExternalFilesDir(null), "Documents").apply { mkdirs() }
            val incomplete = IncompleteSessionScanner.scan(documents)
            val currentSession = SessionContext(this, mode)
            session = currentSession
            currentSession.event(
                EventLevel.INFO,
                EventType.SERVICE_STARTED,
                "計測サービスを開始しました"
            )
            if (incomplete.isNotEmpty()) {
                currentSession.event(
                    EventLevel.WARNING,
                    EventType.INCOMPLETE_SESSION_FOUND,
                    "未完了セッションを${incomplete.size}件検出しました"
                )
            }
            acquireWakeLock()

            storageMonitor = StorageMonitor(
                currentSession.directory,
                onWarning = { bytes ->
                    currentSession.event(
                        EventLevel.WARNING,
                        EventType.LOW_STORAGE,
                        "保存先の空き容量が少なくなりました: $bytes bytes"
                    )
                },
                onCritical = { bytes ->
                    currentSession.event(
                        EventLevel.ERROR,
                        EventType.LOW_STORAGE,
                        "保存先の空き容量が停止基準を下回りました: $bytes bytes"
                    )
                    stopLogging()
                }
            ).also { it.start() }

            imuCollector = ImuCollector(sensorManager, currentSession, ::postFatalError)
                .also { it.start() }
            gnssCollector = GnssCollector(
                this,
                locationManager,
                currentSession,
                ::postFatalError
            ).also { it.start() }
            environmentCollector = EnvironmentCollector(
                sensorManager,
                currentSession,
                ::postFatalError
            ).also { it.start() }
        } catch (error: Throwable) {
            handleFatalError("計測を開始できませんでした", error)
        }
    }

    private fun postFatalError(message: String, error: Throwable?) {
        mainHandler.post { handleFatalError(message, error) }
    }

    private fun stopLogging() {
        if (!running.compareAndSet(true, false)) {
            stopSelf()
            return
        }
        if (!stopping.compareAndSet(false, true)) return

        val imuStats = imuCollector?.snapshot()
        val gnssStats = gnssCollector?.snapshot()
        val freeBytes = storageMonitor?.freeBytes() ?: 0L
        storageMonitor?.close()
        storageMonitor = null

        runCatching {
            session?.event(EventLevel.INFO, EventType.STOP_REQUESTED, "計測停止を受け付けました")
            environmentCollector?.close()
            environmentCollector = null
            gnssCollector?.close()
            gnssCollector = null
            imuCollector?.close()
            imuCollector = null
            val currentSession = session
            if (currentSession != null && imuStats != null && gnssStats != null) {
                currentSession.complete(
                    SessionSummary(
                        accelerometer = imuStats.accelerometer.toSummary(),
                        gyroscope = imuStats.gyroscope.toSummary(),
                        gnss = GnssSummary(
                            gnssStats.count,
                            gnssStats.meanRateHz,
                            gnssStats.maximumIntervalMs
                        ),
                        totalBytes = directoryBytes(currentSession.directory),
                        freeBytesAtEnd = freeBytes
                    )
                )
            }
        }.onFailure { error ->
            runCatching { session?.fail("計測の終了処理に失敗しました: ${error.message}") }
        }

        runCatching { session?.close() }
        session = null
        wakeLock?.takeIf { it.isHeld }?.release()
        wakeLock = null
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun com.example.gnss_imu_logger.sensor.SensorStats.Snapshot.toSummary() =
        SensorSummary(
            receivedCount,
            writtenCount,
            droppedCount,
            estimatedMissingCount,
            meanRateHz,
            medianIntervalMs,
            maximumIntervalMs
        )

    private fun directoryBytes(directory: File): Long =
        directory.walkTopDown().filter { it.isFile }.sumOf { it.length() }

    private fun handleFatalError(message: String, error: Throwable?) {
        if (stopping.get()) return
        val detail = error?.message?.takeIf { it.isNotBlank() }
        runCatching { session?.fail(if (detail == null) message else "$message: $detail") }
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
        getSystemService(NotificationManager::class.java).createNotificationChannel(channel)
    }

    override fun onDestroy() {
        if (running.get()) stopLogging()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder = Binder()
}
