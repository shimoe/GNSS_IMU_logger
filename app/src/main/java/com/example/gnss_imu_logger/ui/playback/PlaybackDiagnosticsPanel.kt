package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.playback.PlaybackSession

/**
 * セッションログの読込診断を表示する。
 * 入力: PlaybackSession
 * 出力: GNSS件数、欠損、最大間隔、不正行、時刻逆転の表示
 */
@Composable
internal fun PlaybackDiagnosticsPanel(
    session: PlaybackSession,
    modifier: Modifier = Modifier
) {
    val diagnostics = session.diagnostics
    Surface(
        modifier = modifier.fillMaxWidth(),
        color = MaterialTheme.colorScheme.surfaceVariant,
        shape = MaterialTheme.shapes.small
    ) {
        Column(
            modifier = Modifier.padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            Text("ログ診断", style = MaterialTheme.typography.titleSmall)
            Text("GNSS: ${diagnostics.gnssSamples}件")
            Text(
                "GNSS欠損: ${diagnostics.gnssGapCount}区間 / " +
                    "最大間隔 ${"%.0f ms".format(diagnostics.maximumGnssIntervalMs)}"
            )
            Text(
                "姿勢: ${diagnostics.attitudeSamples}件 / " +
                    "イベント: ${diagnostics.eventSamples}件"
            )
            Text(
                "除外行: ${diagnostics.rejectedRows}件 / " +
                    "時刻逆転: ${diagnostics.reversedTimestamps}件"
            )
        }
    }
}
