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
import com.example.gnss_imu_logger.playback.PlaybackAnalysisSummary

/**
 * セッション全体の走行解析値を表示する。
 * 入力: PlaybackAnalysisSummary
 * 出力: 参考最高速度と各最大値の表示
 */
@Composable
internal fun PlaybackAnalysisSummaryPanel(
    summary: PlaybackAnalysisSummary,
    modifier: Modifier = Modifier
) {
    Surface(
        modifier = modifier.fillMaxWidth(),
        color = MaterialTheme.colorScheme.surfaceVariant,
        shape = MaterialTheme.shapes.small
    ) {
        Column(
            modifier = Modifier.padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            Text("セッション解析", style = MaterialTheme.typography.titleSmall)
            Text("参考最高速度: ${summary.sustainedTopSpeedMps.speedText()}")
            Text("最大加速度: ${summary.maximumAccelerationMps2.accelerationText()}")
            Text("最大減速度: ${summary.maximumDecelerationMps2.accelerationText()}")
            Text("最大横加速度: ${summary.maximumLateralAccelerationMps2.accelerationText()}")
            Text("最大ヨーレート: ${summary.maximumYawRateRadps.rateText()}")
            Text("最大推定ロール: ${summary.maximumRollRad.angleText()}")
        }
    }
}

private fun Double?.speedText(): String =
    this?.let { "%.1f km/h".format(it * 3.6) } ?: "未取得"

private fun Double?.accelerationText(): String =
    this?.let { "%+.2f m/s²".format(it) } ?: "未取得"

private fun Double?.rateText(): String =
    this?.let { "%+.3f rad/s".format(it) } ?: "未取得"

private fun Double?.angleText(): String =
    this?.let { "%+.2f°".format(Math.toDegrees(it)) } ?: "未取得"
