package com.example.gnss_imu_logger.playback

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import kotlin.math.cos

/**
 * GNSS緯度経度を軌跡表示用の正規化座標へ変換する。
 * 入力: PlaybackLocationの列
 * 出力: 0.0から1.0の範囲に収めた軌跡点と区間情報
 */
class PlaybackTrack(locations: List<PlaybackLocation>) {
    val points: List<TrackPoint>
    val segments: List<TrackSegment>
    val bounds: TrackBounds?
    val xRange: Double
    val yRange: Double
    private val pointByTimestampNs: Map<Long, TrackPoint>

    init {
        val validLocations = locations.filter {
            it.latitudeDeg.isFinite() && it.longitudeDeg.isFinite()
        }
        if (validLocations.size < MINIMUM_POINTS) {
            points = emptyList()
            segments = emptyList()
            bounds = null
            xRange = 1.0
            yRange = 1.0
            pointByTimestampNs = emptyMap()
        } else {
            val latitudeOriginDeg = validLocations.map { it.latitudeDeg }.average()
            val longitudeScale = cos(Math.toRadians(latitudeOriginDeg))
                .coerceAtLeast(MINIMUM_LONGITUDE_SCALE)
            val projected = validLocations.map { location ->
                ProjectedPoint(
                    elapsedRealtimeNs = location.elapsedRealtimeNs,
                    x = location.longitudeDeg * longitudeScale,
                    y = location.latitudeDeg,
                    speedMps = location.speedMps
                )
            }
            val minX = projected.minOf { it.x }
            val maxX = projected.maxOf { it.x }
            val minY = projected.minOf { it.y }
            val maxY = projected.maxOf { it.y }
            val width = (maxX - minX).coerceAtLeast(MINIMUM_RANGE)
            val height = (maxY - minY).coerceAtLeast(MINIMUM_RANGE)
            xRange = width
            yRange = height

            points = projected.map {
                TrackPoint(
                    elapsedRealtimeNs = it.elapsedRealtimeNs,
                    xRatio = (it.x - minX) / width,
                    yRatio = (it.y - minY) / height,
                    speedMps = it.speedMps
                )
            }
            segments = points.zipWithNext { start, end ->
                TrackSegment(
                    start = start,
                    end = end,
                    intervalNs = end.elapsedRealtimeNs - start.elapsedRealtimeNs,
                    speedMps = end.speedMps ?: start.speedMps
                )
            }
            pointByTimestampNs = points.associateBy { it.elapsedRealtimeNs }
            bounds = TrackBounds(
                minimumLatitudeDeg = validLocations.minOf { it.latitudeDeg },
                maximumLatitudeDeg = validLocations.maxOf { it.latitudeDeg },
                minimumLongitudeDeg = validLocations.minOf { it.longitudeDeg },
                maximumLongitudeDeg = validLocations.maxOf { it.longitudeDeg }
            )
        }
    }

    /**
     * 再生カーソルが選択したGNSS位置を表示座標へ変換する。
     * 入力: PlaybackSampleが保持するPlaybackLocation
     * 出力: 現在位置に対応するTrackPoint
     */
    fun pointFor(location: PlaybackLocation?): TrackPoint? {
        val timestampNs = location?.elapsedRealtimeNs ?: return null
        return pointByTimestampNs[timestampNs]
    }

    private data class ProjectedPoint(
        val elapsedRealtimeNs: Long,
        val x: Double,
        val y: Double,
        val speedMps: Double?
    )

    companion object {
        const val GNSS_GAP_NS = 3_000_000_000L
        private const val MINIMUM_POINTS = 2
        private const val MINIMUM_RANGE = 1e-9
        private const val MINIMUM_LONGITUDE_SCALE = 0.1
    }
}

data class TrackPoint(
    val elapsedRealtimeNs: Long,
    val xRatio: Double,
    val yRatio: Double,
    val speedMps: Double?
)

data class TrackSegment(
    val start: TrackPoint,
    val end: TrackPoint,
    val intervalNs: Long,
    val speedMps: Double?
) {
    val gnssGap: Boolean
        get() = intervalNs > PlaybackTrack.GNSS_GAP_NS
}

data class TrackBounds(
    val minimumLatitudeDeg: Double,
    val maximumLatitudeDeg: Double,
    val minimumLongitudeDeg: Double,
    val maximumLongitudeDeg: Double
)

/**
 * GNSS生軌跡、速度区分、GNSS欠損、再生中の現在位置を表示する。
 * 入力: セッション全軌跡、PlaybackSampleが保持する現在位置
 * 出力: オフラインで表示できる簡易軌跡図
 */
@Composable
fun PlaybackTrackView(
    track: PlaybackTrack,
    currentLocation: PlaybackLocation?,
    modifier: Modifier = Modifier
) {
    val currentPoint = remember(track, currentLocation?.elapsedRealtimeNs) {
        track.pointFor(currentLocation)
    }
    var viewport by remember(track) { mutableStateOf(TrackViewport()) }

    Column(modifier = modifier) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(280.dp)
                .background(MaterialTheme.colorScheme.surfaceVariant)
                .clipToBounds(),
            contentAlignment = Alignment.Center
        ) {
            if (track.points.isEmpty()) {
                Text("軌跡を表示できるGNSS位置がありません")
                return@Box
            }

            val traveledColor = MaterialTheme.colorScheme.onSurface
            val markerColor = MaterialTheme.colorScheme.error
            val gapColor = MaterialTheme.colorScheme.error
            Canvas(
                modifier = Modifier
                    .fillMaxSize()
                    .clipToBounds()
                    .pointerInput(track) {
                        detectTransformGestures { _, pan, zoom, _ ->
                            viewport = viewport.updated(zoom = zoom, pan = pan)
                        }
                    }
                    .padding(16.dp)
            ) {
                val content = fittedContentSize(size, track.xRange, track.yRange)
                val left = (size.width - content.width) / 2f
                val top = (size.height - content.height) / 2f
                val center = Offset(size.width / 2f, size.height / 2f)
                fun TrackPoint.toOffset(): Offset {
                    val base = Offset(
                        x = left + (xRatio * content.width).toFloat(),
                        y = top + ((1.0 - yRatio) * content.height).toFloat()
                    )
                    return center + (base - center) * viewport.scale + viewport.translation
                }

                track.segments.forEach { segment ->
                    if (segment.gnssGap) {
                        drawLine(
                            color = gapColor.copy(alpha = 0.85f),
                            start = segment.start.toOffset(),
                            end = segment.end.toOffset(),
                            strokeWidth = 3.dp.toPx(),
                            pathEffect = PathEffect.dashPathEffect(
                                floatArrayOf(10.dp.toPx(), 8.dp.toPx())
                            )
                        )
                    } else {
                        drawLine(
                            color = speedColor(segment.speedMps),
                            start = segment.start.toOffset(),
                            end = segment.end.toOffset(),
                            strokeWidth = 5.dp.toPx(),
                            cap = StrokeCap.Round
                        )
                    }
                }

                val currentNs = currentPoint?.elapsedRealtimeNs
                if (currentNs != null) {
                    track.segments.forEach { segment ->
                        if (segment.end.elapsedRealtimeNs > currentNs) return@forEach
                        if (!segment.gnssGap) {
                            drawLine(
                                color = traveledColor.copy(alpha = 0.35f),
                                start = segment.start.toOffset(),
                                end = segment.end.toOffset(),
                                strokeWidth = 8.dp.toPx(),
                                cap = StrokeCap.Round
                            )
                            drawLine(
                                color = speedColor(segment.speedMps),
                                start = segment.start.toOffset(),
                                end = segment.end.toOffset(),
                                strokeWidth = 5.dp.toPx(),
                                cap = StrokeCap.Round
                            )
                        }
                    }
                }

                currentPoint?.let { point ->
                    val center = point.toOffset()
                    drawCircle(
                        color = Color.White,
                        radius = 9.dp.toPx(),
                        center = center
                    )
                    drawCircle(
                        color = markerColor,
                        radius = 7.dp.toPx(),
                        center = center
                    )
                    drawCircle(
                        color = traveledColor,
                        radius = 11.dp.toPx(),
                        center = center,
                        style = Stroke(width = 2.dp.toPx())
                    )
                }
            }
        }
        Text(
            "2本指で拡大・縮小、ドラッグで移動できます",
            style = MaterialTheme.typography.labelSmall,
            modifier = Modifier.padding(top = 4.dp)
        )
        TrackLegend()
    }
}

@Composable
private fun TrackLegend() {
    Column(
        modifier = Modifier.padding(top = 8.dp),
        verticalArrangement = Arrangement.spacedBy(4.dp)
    ) {
        Text("軌跡色: GNSS速度")
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            LegendItem("0～20", SPEED_LOW_COLOR, Modifier.weight(1f))
            LegendItem("20～50", SPEED_MEDIUM_COLOR, Modifier.weight(1f))
            LegendItem("50～80", SPEED_HIGH_COLOR, Modifier.weight(1f))
            LegendItem("80 km/h～", SPEED_VERY_HIGH_COLOR, Modifier.weight(1f))
        }
        Row(verticalAlignment = Alignment.CenterVertically) {
            Box(
                modifier = Modifier
                    .size(width = 28.dp, height = 3.dp)
                    .background(Color(0xFFD32F2F))
            )
            Text("  GNSS欠損区間（3秒超）")
        }
    }
}

@Composable
private fun LegendItem(
    label: String,
    color: Color,
    modifier: Modifier = Modifier
) {
    Column(modifier = modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(5.dp)
                .background(color)
        )
        Text(label, style = MaterialTheme.typography.labelSmall)
    }
}

private data class TrackViewport(
    val scale: Float = 1f,
    val translation: Offset = Offset.Zero
) {
    /**
     * 拡大率と移動量を更新する。
     * 入力: 拡大率、移動量[px]
     * 出力: 1倍から5倍へ制限した表示状態
     */
    fun updated(zoom: Float, pan: Offset): TrackViewport {
        val nextScale = (scale * zoom).coerceIn(MINIMUM_SCALE, MAXIMUM_SCALE)
        val scaleRatio = nextScale / scale
        return copy(
            scale = nextScale,
            translation = translation * scaleRatio + pan
        )
    }

    companion object {
        private const val MINIMUM_SCALE = 1f
        private const val MAXIMUM_SCALE = 5f
    }
}

/** 軌跡の縦横比を保ち、Canvas中央へ収める。 */
private fun fittedContentSize(canvas: Size, xRange: Double, yRange: Double): Size {
    val trackAspect = (xRange / yRange).coerceAtLeast(1e-6)
    val canvasAspect = canvas.width / canvas.height.coerceAtLeast(1f)
    return if (trackAspect > canvasAspect) {
        Size(canvas.width, (canvas.width / trackAspect).toFloat())
    } else {
        Size((canvas.height * trackAspect).toFloat(), canvas.height)
    }
}

private fun speedColor(speedMps: Double?): Color {
    val speedKmh = speedMps?.times(MPS_TO_KMH) ?: return SPEED_UNKNOWN_COLOR
    return when {
        speedKmh < 20.0 -> SPEED_LOW_COLOR
        speedKmh < 50.0 -> SPEED_MEDIUM_COLOR
        speedKmh < 80.0 -> SPEED_HIGH_COLOR
        else -> SPEED_VERY_HIGH_COLOR
    }
}

private val SPEED_LOW_COLOR = Color(0xFF1976D2)
private val SPEED_MEDIUM_COLOR = Color(0xFF2E7D32)
private val SPEED_HIGH_COLOR = Color(0xFFF9A825)
private val SPEED_VERY_HIGH_COLOR = Color(0xFFC62828)
private val SPEED_UNKNOWN_COLOR = Color(0xFF757575)
private const val MPS_TO_KMH = 3.6
