package com.example.gnss_imu_logger

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import android.os.SystemClock
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.RadioButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.Surface
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import com.example.gnss_imu_logger.measurement.MeasurementSnapshot
import com.example.gnss_imu_logger.measurement.MeasurementState
import com.example.gnss_imu_logger.measurement.MeasurementStateListener
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.playback.PlaybackController
import com.example.gnss_imu_logger.playback.PlaybackState
import com.example.gnss_imu_logger.playback.PlaybackTimeline
import com.example.gnss_imu_logger.playback.SessionCatalog
import com.example.gnss_imu_logger.playback.SessionListItem
import com.example.gnss_imu_logger.playback.SessionLogReader
import com.example.gnss_imu_logger.ui.theme.GNSS_IMU_loggerTheme
import java.io.File
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext

class MainActivity : ComponentActivity(), MeasurementStateListener {
    private var snapshot by mutableStateOf(MeasurementSnapshot())
    private var mode by mutableStateOf(LogMode.DIAGNOSTIC)
    private var selectedScreen by mutableStateOf(AppScreen.MEASUREMENT)
    private var service: LoggerService? = null
    private var bound = false

    private val connection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, binder: IBinder?) {
            service = (binder as? LoggerService.LocalBinder)?.service()
            service?.addStateListener(this@MainActivity)
            bound = service != null
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            service?.removeStateListener(this@MainActivity)
            service = null
            bound = false
        }
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        if (result[Manifest.permission.ACCESS_FINE_LOCATION] == true) startLogger()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            GNSS_IMU_loggerTheme {
                Surface(modifier = Modifier.fillMaxSize()) {
                    AppScreen(
                        snapshot = snapshot,
                        mode = mode,
                        selectedScreen = selectedScreen,
                        documentsDirectory = documentsDirectory(),
                        onScreenChanged = { selectedScreen = it },
                        onModeChanged = { mode = it },
                        onStart = ::requestStart,
                        onStop = ::stopLogger
                    )
                }
            }
        }
    }

    override fun onStart() {
        super.onStart()
        bindService(
            Intent(this, LoggerService::class.java),
            connection,
            Context.BIND_AUTO_CREATE
        )
    }

    override fun onStop() {
        if (bound) {
            service?.removeStateListener(this)
            unbindService(connection)
            bound = false
        }
        super.onStop()
    }

    override fun onSnapshotChanged(snapshot: MeasurementSnapshot) {
        runOnUiThread { this.snapshot = snapshot }
    }

    private fun documentsDirectory(): File =
        File(getExternalFilesDir(null), "Documents").apply { mkdirs() }

    private fun requestStart() {
        val permissions = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions += Manifest.permission.POST_NOTIFICATIONS
        }
        if (permissions.all {
                ContextCompat.checkSelfPermission(this, it) ==
                    PackageManager.PERMISSION_GRANTED
            }
        ) {
            startLogger()
        } else {
            permissionLauncher.launch(permissions.toTypedArray())
        }
    }

    private fun startLogger() {
        ContextCompat.startForegroundService(
            this,
            Intent(this, LoggerService::class.java)
                .setAction(LoggerService.ACTION_START)
                .putExtra(LoggerService.EXTRA_LOG_MODE, mode.name)
        )
    }

    private fun stopLogger() {
        startService(
            Intent(this, LoggerService::class.java)
                .setAction(LoggerService.ACTION_STOP)
        )
    }
}

private enum class AppScreen {
    MEASUREMENT,
    PLAYBACK
}

@Composable
private fun AppScreen(
    snapshot: MeasurementSnapshot,
    mode: LogMode,
    selectedScreen: AppScreen,
    documentsDirectory: File,
    onScreenChanged: (AppScreen) -> Unit,
    onModeChanged: (LogMode) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit
) {
    Scaffold(
        topBar = {
            TabRow(selectedTabIndex = selectedScreen.ordinal) {
                Tab(
                    selected = selectedScreen == AppScreen.MEASUREMENT,
                    onClick = { onScreenChanged(AppScreen.MEASUREMENT) },
                    text = { Text("計測") }
                )
                Tab(
                    selected = selectedScreen == AppScreen.PLAYBACK,
                    onClick = { onScreenChanged(AppScreen.PLAYBACK) },
                    text = { Text("再生") }
                )
            }
        }
    ) { padding ->
        when (selectedScreen) {
            AppScreen.MEASUREMENT -> LoggerScreen(
                snapshot = snapshot,
                mode = mode,
                onModeChanged = onModeChanged,
                onStart = onStart,
                onStop = onStop,
                modifier = Modifier.padding(padding)
            )

            AppScreen.PLAYBACK -> PlaybackScreen(
                documentsDirectory = documentsDirectory,
                modifier = Modifier.padding(padding)
            )
        }
    }
}

@Composable
private fun LoggerScreen(
    snapshot: MeasurementSnapshot,
    mode: LogMode,
    onModeChanged: (LogMode) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit,
    modifier: Modifier = Modifier
) {
    val active = snapshot.state != MeasurementState.IDLE
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(horizontal = 24.dp, vertical = 16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("SOG05 GNSS・IMUロガー", style = MaterialTheme.typography.headlineSmall)
        Text("状態: ${stateText(snapshot.state)}")
        Text("使用衛星: ${snapshot.usedSatellites}")
        Text("水平精度: ${snapshot.horizontalAccuracyM?.let { "%.1f m".format(it) } ?: "未取得"}")
        Text("GNSS更新経過: ${snapshot.lastLocationAgeMs?.let { "$it ms" } ?: "未取得"}")
        Text("加速度: %.1f Hz".format(snapshot.accelerometerRateHz))
        Text("ジャイロ: %.1f Hz".format(snapshot.gyroscopeRateHz))
        Text("静止判定: ${if (snapshot.stationary) "静止" else "未成立"}")
        Text("静止継続: %.1f 秒".format(snapshot.stationaryDurationMs / 1_000.0))
        Text("加速度ノルム: ${snapshot.accelerationNormMps2?.let { "%.3f m/s²".format(it) } ?: "未取得"}")
        Text("角速度ノルム: ${snapshot.gyroscopeNormRadps?.let { "%.4f rad/s".format(it) } ?: "未取得"}")
        Text(
            "初期校正: ${when {
                snapshot.calibrationCompleted -> "完了"
                snapshot.calibrationActive -> "収集中"
                else -> "待機中"
            }}"
        )
        Text("校正収集: 加速度${snapshot.calibrationAccelerometerSamples}件 / ジャイロ${snapshot.calibrationGyroscopeSamples}件")
        Text("校正時間: %.1f秒".format(snapshot.calibrationElapsedMs / 1_000.0))
        Text("補正後角速度: ${snapshot.correctedGyroscopeNormRadps?.let { "%.5f rad/s".format(it) } ?: "未適用"}")
        Text("初期ロール: ${snapshot.initialRollDeg?.let { "%.3f°".format(it) } ?: "未算出"}")
        Text("初期ピッチ: ${snapshot.initialPitchDeg?.let { "%.3f°".format(it) } ?: "未算出"}")
        Text("推定ロール: ${snapshot.estimatedRollDeg?.let { "%.2f°".format(it) } ?: "未推定"}")
        Text("ロール参照: ${snapshot.rollReferenceDeg?.let { "%.2f°".format(it) } ?: "未取得"}")
        Text("ドリフト補正: ${snapshot.rollDriftCorrectionRadps?.let { "%.5f rad/s".format(it) } ?: "未算出"}")
        Text("ロール推定: ${if (snapshot.leanEstimateValid) "有効" else "準備中"}")
        Text(
            "計測時間: %02d:%02d".format(
                snapshot.sessionElapsedMs / 60_000,
                snapshot.sessionElapsedMs / 1_000 % 60
            )
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            RadioButton(
                selected = !active && mode == LogMode.NORMAL,
                onClick = { if (!active) onModeChanged(LogMode.NORMAL) },
                enabled = !active
            )
            Text("通常")
            RadioButton(
                selected = !active && mode == LogMode.DIAGNOSTIC,
                onClick = { if (!active) onModeChanged(LogMode.DIAGNOSTIC) },
                enabled = !active
            )
            Text("診断")
        }
        Button(
            onClick = onStart,
            enabled = !active,
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("計測開始")
        }
        Button(
            onClick = onStop,
            enabled = active,
            modifier = Modifier.fillMaxWidth()
        ) {
            Text("計測停止")
        }
        Spacer(modifier = Modifier.height(16.dp))
    }
}

@Composable
private fun PlaybackScreen(
    documentsDirectory: File,
    modifier: Modifier = Modifier
) {
    var sessions by remember { mutableStateOf<List<SessionListItem>>(emptyList()) }
    var loading by remember { mutableStateOf(true) }
    var errorMessage by remember { mutableStateOf<String?>(null) }
    var selectedSession by remember { mutableStateOf<SessionListItem?>(null) }

    LaunchedEffect(documentsDirectory.absolutePath) {
        loading = true
        errorMessage = null
        runCatching {
            withContext(Dispatchers.IO) {
                SessionCatalog(documentsDirectory).load()
            }
        }.onSuccess {
            sessions = it
        }.onFailure {
            errorMessage = "セッション一覧を読み込めませんでした: ${it.message}"
        }
        loading = false
    }

    selectedSession?.let { item ->
        PlaybackDetailScreen(
            item = item,
            onBack = { selectedSession = null },
            modifier = modifier
        )
        return
    }

    Column(modifier = modifier.fillMaxSize()) {
        Text(
            "保存済みセッション",
            style = MaterialTheme.typography.headlineSmall,
            modifier = Modifier.padding(16.dp)
        )
        when {
            loading -> Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center
            ) {
                CircularProgressIndicator()
            }

            errorMessage != null -> Text(
                errorMessage.orEmpty(),
                color = MaterialTheme.colorScheme.error,
                modifier = Modifier.padding(16.dp)
            )

            sessions.isEmpty() -> Text(
                "保存済みセッションはありません",
                modifier = Modifier.padding(16.dp)
            )

            else -> LazyColumn(
                modifier = Modifier.fillMaxSize(),
                contentPadding = androidx.compose.foundation.layout.PaddingValues(12.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                items(sessions, key = { it.sessionId }) { item ->
                    SessionCard(item = item, onClick = { selectedSession = item })
                }
            }
        }
    }
}

@Composable
private fun SessionCard(
    item: SessionListItem,
    onClick: () -> Unit
) {
    Card(
        modifier = Modifier
            .fillMaxWidth()
            .clickable(onClick = onClick)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            Text(item.sessionId, style = MaterialTheme.typography.titleMedium)
            Text("モード: ${modeText(item.mode)}")
            Text("状態: ${sessionStatusText(item)}")
            Text("記録時間: ${formatDuration(item.durationMs)}")
            Text("GNSS: ${item.gnssCount?.let { "${it}件" } ?: "未集計"}")
            Text("ロールログ: ${if (item.hasLeanAngle) "あり" else "なし"}")
            item.errorMessage?.let {
                Text(it, color = MaterialTheme.colorScheme.error)
            }
        }
    }
}

@Composable
private fun PlaybackDetailScreen(
    item: SessionListItem,
    onBack: () -> Unit,
    modifier: Modifier = Modifier
) {
    var controller by remember(item.sessionId) { mutableStateOf<PlaybackController?>(null) }
    var state by remember(item.sessionId) { mutableStateOf<PlaybackState?>(null) }
    var loading by remember(item.sessionId) { mutableStateOf(true) }
    var errorMessage by remember(item.sessionId) { mutableStateOf<String?>(null) }

    LaunchedEffect(item.sessionId) {
        loading = true
        errorMessage = null
        runCatching {
            withContext(Dispatchers.IO) {
                PlaybackController(PlaybackTimeline(SessionLogReader().read(item)))
            }
        }.onSuccess {
            controller = it
            state = it.state
        }.onFailure {
            errorMessage = "セッションを読み込めませんでした: ${it.message}"
        }
        loading = false
    }

    LaunchedEffect(controller, state?.playing) {
        val playbackController = controller ?: return@LaunchedEffect
        while (playbackController.state.playing) {
            state = playbackController.update(SystemClock.elapsedRealtime())
            delay(PLAYBACK_UPDATE_MS)
        }
    }

    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Button(onClick = onBack) { Text("セッション一覧へ戻る") }
        Text(item.sessionId, style = MaterialTheme.typography.headlineSmall)

        when {
            loading -> CircularProgressIndicator()
            errorMessage != null -> Text(
                errorMessage.orEmpty(),
                color = MaterialTheme.colorScheme.error
            )
            state != null -> PlaybackControls(
                state = requireNotNull(state),
                onPlayPause = {
                    val playbackController = controller ?: return@PlaybackControls
                    if (playbackController.state.playing) {
                        playbackController.pause()
                    } else {
                        playbackController.play(SystemClock.elapsedRealtime())
                    }
                    state = playbackController.state
                },
                onRestart = {
                    state = controller?.restart()
                },
                onSeek = { positionMs ->
                    state = controller?.seek(positionMs)
                },
                onSpeedChanged = { speed ->
                    controller?.setSpeed(speed)
                    state = controller?.state
                }
            )
        }
    }
}

@Composable
private fun PlaybackControls(
    state: PlaybackState,
    onPlayPause: () -> Unit,
    onRestart: () -> Unit,
    onSeek: (Long) -> Unit,
    onSpeedChanged: (Double) -> Unit
) {
    val sample = state.sample
    Text("再生位置: ${formatDuration(state.positionMs)} / ${formatDuration(state.durationMs)}")
    Slider(
        value = state.positionMs.toFloat(),
        onValueChange = { onSeek(it.toLong()) },
        valueRange = 0f..state.durationMs.coerceAtLeast(1L).toFloat(),
        modifier = Modifier.fillMaxWidth()
    )
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Button(onClick = onPlayPause, modifier = Modifier.weight(1f)) {
            Text(if (state.playing) "一時停止" else "再生")
        }
        Button(onClick = onRestart, modifier = Modifier.weight(1f)) {
            Text("先頭へ戻る")
        }
    }
    Text("再生速度")
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        PlaybackController.ALLOWED_SPEEDS.sorted().forEach { speed ->
            Button(
                onClick = { onSpeedChanged(speed) },
                enabled = speed != state.speedMultiplier,
                modifier = Modifier.weight(1f)
            ) {
                Text("${speed}x")
            }
        }
    }
    HorizontalDivider()
    Text("速度: ${sample?.location?.speedMps?.let { "%.1f km/h".format(it * 3.6) } ?: "未取得"}")
    Text("方位: ${sample?.location?.bearingDeg?.let { "%.1f°".format(it) } ?: "未取得"}")
    Text("緯度: ${sample?.location?.latitudeDeg?.let { "%.7f".format(it) } ?: "位置なし"}")
    Text("経度: ${sample?.location?.longitudeDeg?.let { "%.7f".format(it) } ?: "位置なし"}")
    Text("推定ロール: ${sample?.attitude?.rollRad?.let { "%.2f°".format(Math.toDegrees(it)) } ?: "推定なし"}")
    Text("ロール状態: ${if (sample?.attitude?.valid == true) "有効" else "無効"}")
    Text("最新イベント: ${sample?.latestEvent?.message ?: "なし"}")
}

private fun stateText(state: MeasurementState): String = when (state) {
    MeasurementState.IDLE -> "停止中"
    MeasurementState.INITIALIZING -> "センサー初期化中"
    MeasurementState.WAITING_FOR_GNSS -> "GNSS準備待ち"
    MeasurementState.READY -> "GNSS準備完了"
    MeasurementState.DEGRADED -> "GNSS品質低下"
    MeasurementState.STOPPING -> "停止処理中"
    MeasurementState.ERROR -> "エラー"
}

private fun modeText(mode: String): String = when (mode.lowercase()) {
    "normal" -> "通常"
    "diagnostic" -> "診断"
    else -> "不明"
}

private fun sessionStatusText(item: SessionListItem): String = when {
    item.incomplete -> "未完了"
    item.status.equals("completed", ignoreCase = true) -> "完了"
    item.status.equals("recording", ignoreCase = true) -> "記録中"
    item.status.equals("error", ignoreCase = true) -> "エラー"
    else -> "不明"
}

private fun formatDuration(durationMs: Long?): String {
    if (durationMs == null) return "未取得"
    val totalSeconds = durationMs.coerceAtLeast(0L) / 1_000L
    val hours = totalSeconds / 3_600L
    val minutes = totalSeconds / 60L % 60L
    val seconds = totalSeconds % 60L
    return if (hours > 0L) {
        "%02d:%02d:%02d".format(hours, minutes, seconds)
    } else {
        "%02d:%02d".format(minutes, seconds)
    }
}

private const val PLAYBACK_UPDATE_MS = 50L
