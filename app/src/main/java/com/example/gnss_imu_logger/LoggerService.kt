package com.example.gnss_imu_logger

import android.app.*
import android.content.Intent
import android.hardware.SensorManager
import android.location.LocationManager
import android.os.*
import androidx.core.app.NotificationCompat
import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.measurement.*
import com.example.gnss_imu_logger.model.*
import com.example.gnss_imu_logger.sensor.*
import com.example.gnss_imu_logger.storage.*
import java.io.File
import java.util.concurrent.atomic.AtomicBoolean

class LoggerService : Service() {
    companion object {
        const val ACTION_START="logger.START"; const val ACTION_STOP="logger.STOP"
        const val EXTRA_LOG_MODE="logger.LOG_MODE"
        private const val CHANNEL_ID="measurement"; private const val NOTIFICATION_ID=1001
        private const val READINESS_CHECK_MS=200L
    }
    private lateinit var sensorManager:SensorManager; private lateinit var locationManager:LocationManager
    private val mainHandler=Handler(Looper.getMainLooper())
    private var wakeLock:PowerManager.WakeLock?=null; private var session:SessionContext?=null
    private var imuCollector:ImuCollector?=null; private var gnssCollector:GnssCollector?=null
    private var environmentCollector:EnvironmentCollector?=null; private var storageMonitor:StorageMonitor?=null
    private var stateMachine:MeasurementStateMachine?=null; private var readinessEvaluator:ReadinessEvaluator?=null
    private var latestLocationNs:Long?=null; private var latestAccuracyM:Float?=null
    private var qualityMonitor: GnssQualityMonitor? = null
    private var measurementTimeline: MeasurementTimeline? = null
    private val stateListeners = mutableSetOf<MeasurementStateListener>()
    @Volatile
    private var latestSnapshot = MeasurementSnapshot()
    private val localBinder = LocalBinder()
    private var latestUsedSatellites=0; private var consecutiveLocationCount=0
    private var readinessCheckRunning=false
    private val running=AtomicBoolean(false); private val stopping=AtomicBoolean(false)
    private val readinessCheck=object:Runnable{override fun run(){if(!readinessCheckRunning)return;evaluateReadiness();mainHandler.postDelayed(this,READINESS_CHECK_MS)}}
    inner class LocalBinder : Binder() {
        fun service(): LoggerService = this@LoggerService
    }

    fun currentSnapshot(): MeasurementSnapshot = latestSnapshot

    fun addStateListener(listener: MeasurementStateListener) {
        stateListeners += listener
        listener.onSnapshotChanged(latestSnapshot)
    }

    fun removeStateListener(listener: MeasurementStateListener) {
        stateListeners -= listener
    }

    private fun publishSnapshot(snapshot: MeasurementSnapshot) {
        latestSnapshot = snapshot
        stateListeners.toList().forEach { it.onSnapshotChanged(snapshot) }
    }

    override fun onCreate(){super.onCreate();sensorManager=getSystemService(SENSOR_SERVICE) as SensorManager;locationManager=getSystemService(LOCATION_SERVICE) as LocationManager;createChannel()}
    override fun onStartCommand(intent:Intent?,flags:Int,startId:Int):Int{when(intent?.action){ACTION_START->startLogging(parseMode(intent));ACTION_STOP->stopLogging()};return START_NOT_STICKY}
    private fun parseMode(i:Intent)=runCatching{LogMode.valueOf(i.getStringExtra(EXTRA_LOG_MODE)?:LogMode.NORMAL.name)}.getOrDefault(LogMode.NORMAL)
    private fun startLogging(mode:LogMode){
        if(!running.compareAndSet(false,true))return;stopping.set(false)
        try{
            startForeground(NOTIFICATION_ID,notification("センサーを初期化しています"))
            val docs=File(getExternalFilesDir(null),"Documents").apply{mkdirs()};val incomplete=IncompleteSessionScanner.scan(docs)
            val s=SessionContext(this,mode);session=s;stateMachine=MeasurementStateMachine(s) {
                updateNotification(it)
                publishSnapshot(latestSnapshot.copy(state = it))
            }
            stateMachine?.transitionTo(MeasurementState.INITIALIZING,"計測開始操作を受け付けました")
            readinessEvaluator=ReadinessEvaluator()
            qualityMonitor=GnssQualityMonitor()
            measurementTimeline=MeasurementTimeline()
            s.event(EventLevel.INFO,EventType.SERVICE_STARTED,"計測サービスを開始しました")
            if(incomplete.isNotEmpty())s.event(EventLevel.WARNING,EventType.INCOMPLETE_SESSION_FOUND,"未完了セッションを${incomplete.size}件検出しました")
            acquireWakeLock();storageMonitor=StorageMonitor(s.directory,{b->s.event(EventLevel.WARNING,EventType.LOW_STORAGE,"保存先の空き容量が少なくなりました: $b bytes")},{b->s.event(EventLevel.ERROR,EventType.LOW_STORAGE,"保存先の空き容量が停止基準を下回りました: $b bytes");stopLogging()}).also{it.start()}
            imuCollector=ImuCollector(sensorManager,s,::postFatalError).also{it.start()}
            gnssCollector=GnssCollector(this,locationManager,s,::postFatalError){t,u,a,c->latestLocationNs=t;latestUsedSatellites=u;latestAccuracyM=a;consecutiveLocationCount=c}.also{it.start()}
            environmentCollector=EnvironmentCollector(sensorManager,s,::postFatalError).also{it.start()}
            stateMachine?.transitionTo(MeasurementState.WAITING_FOR_GNSS,"センサーとGNSSの初期化が完了しました")
            readinessCheckRunning=true;mainHandler.post(readinessCheck)
        }catch(e:Throwable){handleFatalError("計測を開始できませんでした",e)}
    }
    private fun evaluateReadiness() {
        val machine = stateMachine ?: return
        if (machine.state !in setOf(
                MeasurementState.WAITING_FOR_GNSS,
                MeasurementState.READY,
                MeasurementState.DEGRADED
            )
        ) return

        val imu = imuCollector?.snapshot() ?: return
        val quality = qualityMonitor ?: return
        val nowNs = SystemClock.elapsedRealtimeNanos()

        quality.evaluateLocationAge(nowNs, latestLocationNs).forEach { event ->
            if (event == GnssQualityEvent.LOST) {
                session?.event(
                    EventLevel.WARNING,
                    EventType.GNSS_LOST,
                    "GNSS位置を3秒以上受信できませんでした"
                )
            }
        }

        val status = readinessEvaluator?.evaluate(
            nowNs,
            imu.accelerometer.receivedCount,
            imu.gyroscope.receivedCount,
            imu.accelerometer.droppedCount + imu.gyroscope.droppedCount,
            consecutiveLocationCount,
            latestUsedSatellites,
            latestAccuracyM,
            latestLocationNs,
            true
        ) ?: return

        publishSnapshot(
            MeasurementSnapshot(
                state = machine.state,
                reason = status.reason,
                usedSatellites = status.usedSatellites,
                horizontalAccuracyM = status.horizontalAccuracyM,
                lastLocationAgeMs = status.lastLocationAgeMs,
                accelerometerRateHz = imu.accelerometer.meanRateHz,
                gyroscopeRateHz = imu.gyroscope.meanRateHz,
                sessionElapsedMs = session?.time?.sessionElapsedNs(nowNs)?.div(1_000_000L) ?: 0L
            )
        )

        when {
            status.ready && machine.state == MeasurementState.WAITING_FOR_GNSS -> {
                if (machine.transitionTo(MeasurementState.READY, readyMessage(status))) {
                    quality.onReady(nowNs, recovered = false)
                    measurementTimeline?.markReady(nowNs)
                }
            }

            status.ready && machine.state == MeasurementState.DEGRADED -> {
                if (machine.transitionTo(MeasurementState.READY, readyMessage(status))) {
                    quality.onReady(nowNs, recovered = true)
                    measurementTimeline?.markReady(nowNs)
                    session?.event(
                        EventLevel.INFO,
                        EventType.GNSS_RECOVERED,
                        "GNSS位置の受信が安定しました"
                    )
                }
            }

            !status.ready && machine.state == MeasurementState.READY -> {
                if (machine.transitionTo(
                        MeasurementState.DEGRADED,
                        "GNSS品質が低下しました: ${status.reason.name}"
                    )
                ) {
                    quality.onDegraded(nowNs)
                }
            }
        }
    }

    private fun readyMessage(r:ReadinessStatus)="GNSS準備条件が成立しました: 使用衛星${r.usedSatellites}、水平精度${r.horizontalAccuracyM} m、連続位置${r.locationCount}回"
    private fun stopLogging(){
        if(!running.compareAndSet(true,false)){stopSelf();return};if(!stopping.compareAndSet(false,true))return
        readinessCheckRunning=false
        mainHandler.removeCallbacks(readinessCheck)
        stateMachine?.transitionTo(MeasurementState.STOPPING,"計測停止処理を開始します")
        val stoppedNs = SystemClock.elapsedRealtimeNanos()
        measurementTimeline?.markStopped(stoppedNs)
        val qualitySummary = qualityMonitor?.finish(stoppedNs)
        val measurementSummary = if (qualitySummary != null) {
            measurementTimeline?.summary(qualitySummary)
        } else null
        val i=imuCollector?.snapshot();val g=gnssCollector?.snapshot();val free=storageMonitor?.freeBytes()?:0L;storageMonitor?.close();storageMonitor=null
        runCatching{environmentCollector?.close();environmentCollector=null;gnssCollector?.close();gnssCollector=null;imuCollector?.close();imuCollector=null;val s=session;if(s!=null&&i!=null&&g!=null)s.complete(SessionSummary(
                i.accelerometer.toSummary(),
                i.gyroscope.toSummary(),
                GnssSummary(g.count, g.meanRateHz, g.maximumIntervalMs),
                dirBytes(s.directory),
                free,
                requireNotNull(measurementSummary) { "計測状態統計を確定できませんでした" }
            ))}.onFailure{e->runCatching{session?.fail("計測の終了処理に失敗しました: ${e.message}")}}
        stateMachine?.transitionTo(MeasurementState.IDLE,"計測停止処理が完了しました");runCatching{session?.close()};session=null
        stateMachine=null
        readinessEvaluator=null
        qualityMonitor=null
        measurementTimeline=null
        wakeLock?.takeIf{it.isHeld}?.release();wakeLock=null;stopForeground(STOP_FOREGROUND_REMOVE);stopSelf()
    }
    private fun SensorStats.Snapshot.toSummary()=SensorSummary(receivedCount,writtenCount,droppedCount,estimatedMissingCount,meanRateHz,medianIntervalMs,maximumIntervalMs)
    private fun dirBytes(d:File)=d.walkTopDown().filter{it.isFile}.sumOf{it.length()}
    private fun postFatalError(m:String,e:Throwable?){mainHandler.post{handleFatalError(m,e)}}
    private fun handleFatalError(m:String,e:Throwable?){if(stopping.get())return;stateMachine?.transitionTo(MeasurementState.ERROR,m);runCatching{session?.fail(if(e?.message.isNullOrBlank())m else "$m: ${e?.message}")};stopLogging()}
    private fun acquireWakeLock(){val p=getSystemService(POWER_SERVICE) as PowerManager;wakeLock=p.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK,"GNSS_IMU_logger:measurement").apply{setReferenceCounted(false);acquire()};session?.event(EventLevel.INFO,EventType.WAKE_LOCK_ACQUIRED,"画面オフ計測用のWakeLockを取得しました")}
    private fun updateNotification(s:MeasurementState){getSystemService(NotificationManager::class.java).notify(NOTIFICATION_ID,notification(when(s){MeasurementState.WAITING_FOR_GNSS->"GNSS準備待ち";MeasurementState.READY->"GNSS準備完了: 走行可能";MeasurementState.DEGRADED->"GNSS品質低下: IMU記録継続中";else->s.name}))}
    private fun notification(text:String)=NotificationCompat.Builder(this,CHANNEL_ID).setSmallIcon(android.R.drawable.ic_menu_mylocation).setContentTitle("GNSS・IMU計測中").setContentText(text).setOngoing(true).build()
    private fun createChannel(){getSystemService(NotificationManager::class.java).createNotificationChannel(NotificationChannel(CHANNEL_ID,"計測状態",NotificationManager.IMPORTANCE_LOW))}
    override fun onDestroy(){if(running.get())stopLogging();super.onDestroy()};override fun onBind(intent: Intent?): IBinder = localBinder
}
