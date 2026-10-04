package com.example.gnss_imu_logger.ui.measurement

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.RadioButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.measurement.MeasurementSnapshot
import com.example.gnss_imu_logger.measurement.MeasurementState
import com.example.gnss_imu_logger.model.LogMode

@Composable
fun MeasurementScreen(
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


private fun stateText(state: MeasurementState): String = when (state) {
    MeasurementState.IDLE -> "停止中"
    MeasurementState.INITIALIZING -> "センサー初期化中"
    MeasurementState.WAITING_FOR_GNSS -> "GNSS準備待ち"
    MeasurementState.READY -> "GNSS準備完了"
    MeasurementState.DEGRADED -> "GNSS品質低下"
    MeasurementState.STOPPING -> "停止処理中"
    MeasurementState.ERROR -> "エラー"
}

