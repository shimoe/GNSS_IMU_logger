package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.playback.PlaybackSample
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/** 軌跡枠内へ重ねる小型の車体挙動HUD。 */
@Composable
internal fun VehicleDynamicsHud(sample: PlaybackSample?, modifier: Modifier = Modifier) {
    val longitudinal = sample?.location?.longitudinalAccelerationMps2
    val lateral = sample?.attitude?.lateralAccelerationMps2
    val roll = sample?.attitude?.rollRad?.takeIf { sample.attitude.valid }
    val braking = sample?.location?.braking == true
    val axisColor = Color.White
    val pointColor = if (braking) BRAKING_COLOR else MaterialTheme.colorScheme.primary

    Column(
        modifier = modifier
            .size(HUD_SIZE_DP.dp)
            .background(Color.Black.copy(alpha = 0.58f), CircleShape)
            .padding(6.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Canvas(modifier = Modifier.size(HUD_CANVAS_DP.dp)) {
            val center = Offset(size.width / 2f, size.height / 2f)
            val radius = size.minDimension / 2f - 5.dp.toPx()
            val ringColor = if (braking) BRAKING_COLOR else axisColor
            drawCircle(ringColor, radius, center, style = Stroke(if (braking) 3.dp.toPx() else 1.dp.toPx()))
            drawCircle(axisColor.copy(alpha = 0.35f), radius * 0.5f, center, style = Stroke(1.dp.toPx()))
            drawLine(axisColor.copy(alpha = 0.45f), Offset(center.x - radius, center.y), Offset(center.x + radius, center.y), 1.dp.toPx())
            drawLine(axisColor.copy(alpha = 0.45f), Offset(center.x, center.y - radius), Offset(center.x, center.y + radius), 1.dp.toPx())

            if (longitudinal != null || lateral != null) {
                val x = ((lateral ?: 0.0) / ACCELERATION_LIMIT_MPS2).coerceIn(-1.0, 1.0)
                // 重心移動として、加速時は下、減速時は上へ表示する。
                val y = ((longitudinal ?: 0.0) / ACCELERATION_LIMIT_MPS2).coerceIn(-1.0, 1.0)
                val point = Offset(center.x + (radius * x).toFloat(), center.y + (radius * y).toFloat())
                drawLine(pointColor.copy(alpha = 0.55f), center, point, 2.dp.toPx(), StrokeCap.Round)
                drawCircle(Color.White, 5.dp.toPx(), point)
                drawCircle(pointColor, 3.5.dp.toPx(), point)
            }
            roll?.let {
                val angle = -PI / 2.0 + it.coerceIn(-MAXIMUM_ROLL_RAD, MAXIMUM_ROLL_RAD)
                drawCircle(
                    ROLL_COLOR,
                    4.dp.toPx(),
                    Offset(center.x + (cos(angle) * radius).toFloat(), center.y + (sin(angle) * radius).toFloat())
                )
            }
        }
        Text(
            if (braking) "推定制動" else "前後 ${longitudinal?.let { "%+.1f".format(it) } ?: "--"}",
            style = MaterialTheme.typography.labelSmall,
            color = if (braking) BRAKING_COLOR else Color.White,
            maxLines = 1
        )
    }
}

private const val HUD_SIZE_DP = 112
private const val HUD_CANVAS_DP = 86
private const val ACCELERATION_LIMIT_MPS2 = 10.0
private val MAXIMUM_ROLL_RAD = Math.toRadians(60.0)
private val BRAKING_COLOR = Color(0xFFFF6D00)
private val ROLL_COLOR = Color(0xFF00ACC1)
