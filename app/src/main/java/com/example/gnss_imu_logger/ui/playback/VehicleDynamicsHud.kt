package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.playback.PlaybackSample
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * 現在時刻のロール角、前後加速度、横加速度、推定制動状態を円形HUDで表示する。
 * 入力: PlaybackSample
 * 出力: 重心移動を模した加速度点とロール角マーカー
 */
@Composable
internal fun VehicleDynamicsHud(
    sample: PlaybackSample?,
    modifier: Modifier = Modifier
) {
    val longitudinalMps2 = sample?.location?.longitudinalAccelerationMps2
    val lateralMps2 = sample?.attitude?.lateralAccelerationMps2
    val rollRad = sample?.attitude?.rollRad?.takeIf { sample.attitude.valid }
    val braking = sample?.location?.braking == true
    val axisColor = MaterialTheme.colorScheme.onSurfaceVariant
    val pointColor = if (braking) BRAKING_COLOR else MaterialTheme.colorScheme.primary

    Column(
        modifier = modifier
            .fillMaxWidth()
            .background(
                color = MaterialTheme.colorScheme.surfaceVariant,
                shape = MaterialTheme.shapes.small
            )
            .padding(12.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(6.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text("車体挙動", style = MaterialTheme.typography.titleSmall)
            Text(
                if (braking) "推定制動中" else "制動なし",
                color = if (braking) BRAKING_COLOR else axisColor
            )
        }
        Box(contentAlignment = Alignment.Center) {
            Canvas(modifier = Modifier.size(HUD_SIZE_DP.dp)) {
                val center = Offset(size.width / 2f, size.height / 2f)
                val radius = size.minDimension / 2f - 12.dp.toPx()
                val ringColor = if (braking) BRAKING_COLOR else axisColor

                drawCircle(
                    color = ringColor,
                    radius = radius,
                    center = center,
                    style = Stroke(width = if (braking) 4.dp.toPx() else 2.dp.toPx())
                )
                drawCircle(
                    color = axisColor.copy(alpha = 0.35f),
                    radius = radius * 0.5f,
                    center = center,
                    style = Stroke(width = 1.dp.toPx())
                )
                drawLine(
                    color = axisColor.copy(alpha = 0.55f),
                    start = Offset(center.x - radius, center.y),
                    end = Offset(center.x + radius, center.y),
                    strokeWidth = 1.dp.toPx()
                )
                drawLine(
                    color = axisColor.copy(alpha = 0.55f),
                    start = Offset(center.x, center.y - radius),
                    end = Offset(center.x, center.y + radius),
                    strokeWidth = 1.dp.toPx()
                )

                if (longitudinalMps2 != null || lateralMps2 != null) {
                    val xRatio = ((lateralMps2 ?: 0.0) / HUD_ACCELERATION_LIMIT_MPS2)
                        .coerceIn(-1.0, 1.0)
                    // 重心移動の表示なので、加速は下、減速は上へ動かす。
                    val yRatio = ((longitudinalMps2 ?: 0.0) / HUD_ACCELERATION_LIMIT_MPS2)
                        .coerceIn(-1.0, 1.0)
                    val point = Offset(
                        x = center.x + (radius * xRatio).toFloat(),
                        y = center.y + (radius * yRatio).toFloat()
                    )
                    drawLine(
                        color = pointColor.copy(alpha = 0.55f),
                        start = center,
                        end = point,
                        strokeWidth = 3.dp.toPx(),
                        cap = StrokeCap.Round
                    )
                    drawCircle(
                        color = Color.White,
                        radius = 7.dp.toPx(),
                        center = point
                    )
                    drawCircle(
                        color = pointColor,
                        radius = 5.dp.toPx(),
                        center = point
                    )
                }

                rollRad?.let { roll ->
                    val limitedRoll = roll.coerceIn(-MAXIMUM_ROLL_RAD, MAXIMUM_ROLL_RAD)
                    val angleRad = -PI / 2.0 + limitedRoll
                    val markerCenter = Offset(
                        x = center.x + (cos(angleRad) * radius).toFloat(),
                        y = center.y + (sin(angleRad) * radius).toFloat()
                    )
                    val direction = Offset(
                        x = (center.x - markerCenter.x) / radius,
                        y = (center.y - markerCenter.y) / radius
                    )
                    val tangent = Offset(-direction.y, direction.x)
                    val tip = markerCenter + direction * 12.dp.toPx()
                    val baseCenter = markerCenter - direction * 2.dp.toPx()
                    val path = Path().apply {
                        moveTo(tip.x, tip.y)
                        lineTo(
                            baseCenter.x + tangent.x * 6.dp.toPx(),
                            baseCenter.y + tangent.y * 6.dp.toPx()
                        )
                        lineTo(
                            baseCenter.x - tangent.x * 6.dp.toPx(),
                            baseCenter.y - tangent.y * 6.dp.toPx()
                        )
                        close()
                    }
                    drawPath(path = path, color = ROLL_MARKER_COLOR)
                }
            }
        }
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            Text("前後 ${longitudinalMps2.hudValue("m/s²")}")
            Text("横 ${lateralMps2.hudValue("m/s²")}")
            Text("ロール ${rollRad?.let { "%+.1f°".format(Math.toDegrees(it)) } ?: "未取得"}")
        }
        Text(
            "重心点: 加速で下、減速で上",
            style = MaterialTheme.typography.labelSmall,
            color = axisColor
        )
    }
}

private fun Double?.hudValue(unit: String): String =
    this?.let { "%+.2f %s".format(it, unit) } ?: "未取得"

private const val HUD_SIZE_DP = 210
private const val HUD_ACCELERATION_LIMIT_MPS2 = 10.0
private val MAXIMUM_ROLL_RAD = Math.toRadians(60.0)
private val BRAKING_COLOR = Color(0xFFFF6D00)
private val ROLL_MARKER_COLOR = Color(0xFF00ACC1)
