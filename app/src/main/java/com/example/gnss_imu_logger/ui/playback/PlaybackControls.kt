package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Button
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.playback.PlaybackAnalysisSummaryCalculator
import com.example.gnss_imu_logger.playback.PlaybackController
import com.example.gnss_imu_logger.playback.PlaybackSession
import com.example.gnss_imu_logger.playback.PlaybackState
import com.example.gnss_imu_logger.playback.PlaybackTrack
import com.example.gnss_imu_logger.playback.PlaybackTrackView

@Composable
internal fun PlaybackControls(
    state: PlaybackState,
    session: PlaybackSession,
    onPlayPause: () -> Unit,
    onRestart: () -> Unit,
    onSeek: (Long) -> Unit,
    onSpeedChanged: (Double) -> Unit
) {
    val sample = state.sample
    val track = remember(session) { PlaybackTrack(session.locations) }
    val analysisSummary = remember(session) {
        PlaybackAnalysisSummaryCalculator.calculate(session)
    }
    PlaybackTrackView(
        track = track,
        currentLocation = sample?.location,
        modifier = Modifier.fillMaxWidth(),
        background = {
            OnlinePlaybackMap(
                locations = session.locations,
                currentLocation = sample?.location,
                bounds = track.bounds,
                onStatusChanged = { status ->
                    // 状態表示はOnlinePlaybackMap内で行う。
                    // 次段階で再試行操作へ接続する。
                },
                modifier = Modifier.fillMaxSize()
            )
        },
        overlay = {
            VehicleDynamicsHud(
                sample = sample,
                modifier = Modifier.align(Alignment.TopEnd).padding(8.dp)
            )
        }
    )
    Text("再生位置: ${formatPlaybackDuration(state.positionMs)} / ${formatPlaybackDuration(state.durationMs)}")
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
                Text(
                    text = playbackSpeedText(speed),
                    maxLines = 1,
                    overflow = TextOverflow.Clip
                )
            }
        }
    }
    PlaybackAnalysisSummaryPanel(analysisSummary)
    HorizontalDivider()
    Text("速度: ${sample?.location?.speedMps?.let { "%.1f km/h".format(it * 3.6) } ?: "未取得"}")
    Text(
        "前後加速度: ${sample?.location?.longitudinalAccelerationMps2?.let { "%+.2f m/s²".format(it) } ?: "未取得"}"
    )
    Text("制動状態: ${if (sample?.location?.braking == true) "推定制動中" else "制動なし"}")
    Text("方位: ${sample?.location?.bearingDeg?.let { "%.1f°".format(it) } ?: "未取得"}")
    Text("緯度: ${sample?.location?.latitudeDeg?.let { "%.7f".format(it) } ?: "位置なし"}")
    Text("経度: ${sample?.location?.longitudeDeg?.let { "%.7f".format(it) } ?: "位置なし"}")
    Text("推定ロール: ${sample?.attitude?.rollRad?.let { "%.2f°".format(Math.toDegrees(it)) } ?: "推定なし"}")
    Text(
        "ヨーレート: ${sample?.attitude?.yawRateRadps?.let { "%.3f rad/s".format(it) } ?: "未取得"}"
    )
    Text(
        "横加速度: ${sample?.attitude?.lateralAccelerationMps2?.let { "%.2f m/s²".format(it) } ?: "未取得"}"
    )
    Text("ロール状態: ${if (sample?.attitude?.valid == true) "有効" else "無効"}")
    Text("最新イベント: ${sample?.latestEvent?.message ?: "なし"}")
}




/** 再生倍率を1行で表示する。 */
private fun playbackSpeedText(speed: Double): String = when (speed) {
    0.25 -> "0.25×"
    0.5 -> "0.5×"
    1.0 -> "1×"
    2.0 -> "2×"
    4.0 -> "4×"
    else -> "${speed}×"
}
