package com.example.gnss_imu_logger.ui.playback

import android.os.SystemClock
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
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
import com.example.gnss_imu_logger.playback.PlaybackController
import com.example.gnss_imu_logger.playback.PlaybackSession
import com.example.gnss_imu_logger.playback.PlaybackState
import com.example.gnss_imu_logger.playback.PlaybackTimeline
import com.example.gnss_imu_logger.playback.SessionCatalog
import com.example.gnss_imu_logger.playback.SessionListItem
import com.example.gnss_imu_logger.playback.SessionLogReader
import java.io.File
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext

@Composable
fun PlaybackScreen(
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
            Text("記録時間: ${formatPlaybackDuration(item.durationMs)}")
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
    var playbackSession by remember(item.sessionId) { mutableStateOf<PlaybackSession?>(null) }
    var state by remember(item.sessionId) { mutableStateOf<PlaybackState?>(null) }
    var loading by remember(item.sessionId) { mutableStateOf(true) }
    var errorMessage by remember(item.sessionId) { mutableStateOf<String?>(null) }

    LaunchedEffect(item.sessionId) {
        loading = true
        errorMessage = null
        runCatching {
            withContext(Dispatchers.IO) {
                SessionLogReader().read(item)
            }
        }.onSuccess { session ->
            playbackSession = session
            controller = PlaybackController(PlaybackTimeline(session))
            state = controller?.state
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
            state != null && playbackSession != null -> PlaybackControls(
                state = requireNotNull(state),
                session = requireNotNull(playbackSession),
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

internal fun formatPlaybackDuration(durationMs: Long?): String {
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
