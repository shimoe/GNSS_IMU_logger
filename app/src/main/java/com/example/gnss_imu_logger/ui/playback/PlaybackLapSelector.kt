package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.navigation.LapRecord
import com.example.gnss_imu_logger.navigation.LapType
import com.example.gnss_imu_logger.playback.PlaybackRange

/** ラップタイム、ベスト差、再生対象の選択UIを表示する。 */
@Composable
internal fun PlaybackLapSelector(
    laps: List<LapRecord>,
    selected: PlaybackRange,
    onSelectWhole: () -> Unit,
    onSelectLap: (LapRecord) -> Unit
) {
    if (laps.isEmpty()) return
    val bestSec = laps.filter { it.lapType == LapType.TIMED_LAP }
        .minOfOrNull { it.durationSec }
    Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
        Text("ラップ選択", style = MaterialTheme.typography.titleMedium)
        LapRow("セッション全体", "", selected.lap == null, onSelectWhole)
        laps.forEach { lap ->
            val difference = if (lap.lapType == LapType.TIMED_LAP && bestSec != null) {
                val delta = lap.durationSec - bestSec
                if (delta < 0.0005) "BEST" else "+%.3f".format(delta)
            } else ""
            val warning = if (lap.durationOutlier) "  要確認" else ""
            LapRow(
                label = lapLabel(lap),
                value = "${formatLapTime(lap.durationSec)}  $difference$warning".trimEnd(),
                selected = selected.lap == lap,
                onClick = { onSelectLap(lap) }
            )
        }
    }
}

@Composable
private fun LapRow(label: String, value: String, selected: Boolean, onClick: () -> Unit) {
    Surface(
        modifier = Modifier.fillMaxWidth().clickable(onClick = onClick),
        color = if (selected) MaterialTheme.colorScheme.primaryContainer
        else MaterialTheme.colorScheme.surfaceVariant,
        shape = MaterialTheme.shapes.small
    ) {
        Row(
            modifier = Modifier.padding(horizontal = 12.dp, vertical = 9.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(label)
            Text(value, style = MaterialTheme.typography.bodyMedium)
        }
    }
}

private fun lapLabel(lap: LapRecord): String = when (lap.lapType) {
    LapType.OUT_LAP -> "OUT"
    LapType.IN_LAP -> "IN"
    LapType.TIMED_LAP -> "Lap ${lap.lapIndex}"
    LapType.INVALID_LAP -> "無効"
}

private fun formatLapTime(durationSec: Double): String {
    val totalMs = (durationSec.coerceAtLeast(0.0) * 1_000.0).toLong()
    val minutes = totalMs / 60_000L
    val seconds = totalMs / 1_000L % 60L
    val millis = totalMs % 1_000L
    return "%02d:%02d.%03d".format(minutes, seconds, millis)
}
