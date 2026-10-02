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
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import com.example.gnss_imu_logger.measurement.*
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.ui.theme.GNSS_IMU_loggerTheme

class MainActivity : ComponentActivity(), MeasurementStateListener {
    private var snapshot by mutableStateOf(MeasurementSnapshot())
    private var mode by mutableStateOf(LogMode.DIAGNOSTIC)
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
                    LoggerScreen(snapshot, mode, { mode = it }, ::requestStart, ::stopLogger)
                }
            }
        }
    }

    override fun onStart() {
        super.onStart()
        bindService(Intent(this, LoggerService::class.java), connection, Context.BIND_AUTO_CREATE)
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

    private fun requestStart() {
        val permissions = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions += Manifest.permission.POST_NOTIFICATIONS
        }
        if (permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }) {
            startLogger()
        } else permissionLauncher.launch(permissions.toTypedArray())
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
        startService(Intent(this, LoggerService::class.java).setAction(LoggerService.ACTION_STOP))
    }
}

@Composable
private fun LoggerScreen(
    snapshot: MeasurementSnapshot,
    mode: LogMode,
    onModeChanged: (LogMode) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit
) {
    val active = snapshot.state != MeasurementState.IDLE
    Column(
        modifier = Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp, Alignment.CenterVertically),
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
        Text("初期校正: ${when { snapshot.calibrationCompleted -> "完了"; snapshot.calibrationActive -> "収集中"; else -> "待機中" }}")
        Text("校正収集: 加速度${snapshot.calibrationAccelerometerSamples}件 / ジャイロ${snapshot.calibrationGyroscopeSamples}件")
        Text("校正時間: %.1f秒".format(snapshot.calibrationElapsedMs / 1_000.0))
        Text("計測時間: %02d:%02d".format(snapshot.sessionElapsedMs / 60_000, snapshot.sessionElapsedMs / 1_000 % 60))

        Row(verticalAlignment = Alignment.CenterVertically) {
            RadioButton(!active && mode == LogMode.NORMAL, { if (!active) onModeChanged(LogMode.NORMAL) }, enabled = !active)
            Text("通常")
            RadioButton(!active && mode == LogMode.DIAGNOSTIC, { if (!active) onModeChanged(LogMode.DIAGNOSTIC) }, enabled = !active)
            Text("診断")
        }
        Button(onClick = onStart, enabled = !active) { Text("計測開始") }
        Button(onClick = onStop, enabled = active) { Text("計測停止") }
    }
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
