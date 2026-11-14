package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.playback.NavigationPlaybackPoint
import com.example.gnss_imu_logger.playback.NavigationPlaybackResult
import kotlin.math.cos
import kotlin.math.sin

/** 改善済み自己位置推定のENU軌跡と現在位置を再生時刻に同期して表示する。 */
@Composable
fun NavigationPlaybackTrackView(
    navigation: NavigationPlaybackResult,
    currentElapsedRealtimeNs: Long,
    modifier: Modifier = Modifier
) {
    val current = navigation.nearest(currentElapsedRealtimeNs)
    val points = navigation.points
    val minimumEast = points.minOf { it.eastM }
    val maximumEast = points.maxOf { it.eastM }
    val minimumNorth = points.minOf { it.northM }
    val maximumNorth = points.maxOf { it.northM }
    val eastRange = (maximumEast - minimumEast).coerceAtLeast(1.0)
    val northRange = (maximumNorth - minimumNorth).coerceAtLeast(1.0)

    Column(
        modifier = modifier
            .fillMaxWidth()
            .background(MaterialTheme.colorScheme.surfaceVariant)
            .padding(8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Text("改善済み自己位置推定", style = MaterialTheme.typography.titleMedium)
        Text("通常・診断モード共通の推定パイプライン")
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            Text("品質: ${navigation.assessment ?: "未評価"}")
            Text("点数: ${points.size}")
            current?.let { Text("速度: %.1f km/h".format(it.speedMps * 3.6)) }
        }
        Canvas(
            modifier = Modifier
                .fillMaxWidth()
                .height(280.dp)
        ) {
            val padding = 20f
            fun screen(point: NavigationPlaybackPoint): Offset {
                val x = padding + ((point.eastM - minimumEast) / eastRange).toFloat() *
                    (size.width - padding * 2f)
                val y = size.height - padding -
                    ((point.northM - minimumNorth) / northRange).toFloat() *
                    (size.height - padding * 2f)
                return Offset(x, y)
            }
            val path = Path()
            points.forEachIndexed { index, point ->
                val value = screen(point)
                if (index == 0) path.moveTo(value.x, value.y) else path.lineTo(value.x, value.y)
            }
            drawPath(path, Color(0xFF1565C0), style = Stroke(width = 4f))
            current?.let { point ->
                val marker = screen(point)
                drawCircle(Color(0xFFD32F2F), radius = 8f, center = marker)
                point.yawRad?.let { yaw ->
                    val length = 22f
                    val end = Offset(
                        marker.x + (sin(yaw) * length).toFloat(),
                        marker.y - (cos(yaw) * length).toFloat()
                    )
                    drawLine(Color(0xFFD32F2F), marker, end, strokeWidth = 4f)
                }
                point.horizontalPositionStdM?.let { stdM ->
                    val pixelsPerMeter = minOf(
                        (size.width - padding * 2f) / eastRange.toFloat(),
                        (size.height - padding * 2f) / northRange.toFloat()
                    )
                    drawCircle(
                        color = Color(0x331565C0),
                        radius = (stdM * pixelsPerMeter).toFloat().coerceAtMost(120f),
                        center = marker
                    )
                }
            }
        }
        current?.let {
            Text(
                "ENU: E %.1f m / N %.1f m / U %.1f m / 状態 ${it.status}"
                    .format(it.eastM, it.northM, it.upM)
            )
        } ?: Text("現在の再生時刻に対応する推定サンプルがありません。")
    }
}
