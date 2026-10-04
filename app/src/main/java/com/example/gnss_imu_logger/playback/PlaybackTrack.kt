package com.example.gnss_imu_logger.playback

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import kotlin.math.cos

/**
 * GNSS緯度経度を軌跡表示用の正規化座標へ変換する。
 * 入力: PlaybackLocationの列
 * 出力: 0.0から1.0の範囲に収めた軌跡点
 */
class PlaybackTrack(locations: List<PlaybackLocation>) {
    val points: List<TrackPoint>
    val bounds: TrackBounds?

    init {
        val validLocations = locations.filter {
            it.latitudeDeg.isFinite() && it.longitudeDeg.isFinite()
        }
        if (validLocations.size < MINIMUM_POINTS) {
            points = emptyList()
            bounds = null
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

            points = projected.map {
                TrackPoint(
                    elapsedRealtimeNs = it.elapsedRealtimeNs,
                    xRatio = (it.x - minX) / width,
                    yRatio = (it.y - minY) / height,
                    speedMps = it.speedMps
                )
            }
            bounds = TrackBounds(
                minimumLatitudeDeg = validLocations.minOf { it.latitudeDeg },
                maximumLatitudeDeg = validLocations.maxOf { it.latitudeDeg },
                minimumLongitudeDeg = validLocations.minOf { it.longitudeDeg },
                maximumLongitudeDeg = validLocations.maxOf { it.longitudeDeg }
            )
        }
    }

    /**
     * 指定時刻以前の最新軌跡点を返す。
     * 入力: elapsedRealtimeNs[ns]
     * 出力: 現在位置に対応するTrackPoint
     */
    fun pointAt(elapsedRealtimeNs: Long): TrackPoint? {
        if (points.isEmpty()) return null
        var low = 0
        var high = points.lastIndex
        var result = -1
        while (low <= high) {
            val middle = (low + high).ushr(1)
            if (points[middle].elapsedRealtimeNs <= elapsedRealtimeNs) {
                result = middle
                low = middle + 1
            } else {
                high = middle - 1
            }
        }
        return points.getOrNull(result)
    }

    private data class ProjectedPoint(
        val elapsedRealtimeNs: Long,
        val x: Double,
        val y: Double,
        val speedMps: Double?
    )

    companion object {
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

data class TrackBounds(
    val minimumLatitudeDeg: Double,
    val maximumLatitudeDeg: Double,
    val minimumLongitudeDeg: Double,
    val maximumLongitudeDeg: Double
)

/**
 * GNSS生軌跡と再生中の現在位置を表示する。
 * 入力: セッション全軌跡、現在のelapsedRealtimeNs[ns]
 * 出力: オフラインで表示できる簡易軌跡図
 */
@Composable
fun PlaybackTrackView(
    track: PlaybackTrack,
    currentElapsedRealtimeNs: Long?,
    modifier: Modifier = Modifier
) {
    val currentPoint = remember(track, currentElapsedRealtimeNs) {
        currentElapsedRealtimeNs?.let(track::pointAt)
    }
    val points = track.points

    Box(
        modifier = modifier
            .height(280.dp)
            .background(MaterialTheme.colorScheme.surfaceVariant),
        contentAlignment = Alignment.Center
    ) {
        if (points.isEmpty()) {
            Text("軌跡を表示できるGNSS位置がありません")
            return@Box
        }

        val routeColor = MaterialTheme.colorScheme.primary
        val traveledColor = MaterialTheme.colorScheme.tertiary
        val markerColor = MaterialTheme.colorScheme.error
        Canvas(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp)
        ) {
            fun TrackPoint.toOffset(): Offset = Offset(
                x = (xRatio * size.width).toFloat(),
                y = ((1.0 - yRatio) * size.height).toFloat()
            )

            for (index in 1 until points.size) {
                val previous = points[index - 1]
                val current = points[index]
                drawLine(
                    color = routeColor.copy(alpha = 0.35f),
                    start = previous.toOffset(),
                    end = current.toOffset(),
                    strokeWidth = 4.dp.toPx(),
                    cap = StrokeCap.Round
                )
            }

            val currentNs = currentPoint?.elapsedRealtimeNs
            if (currentNs != null) {
                for (index in 1 until points.size) {
                    val current = points[index]
                    if (current.elapsedRealtimeNs > currentNs) break
                    drawLine(
                        color = traveledColor,
                        start = points[index - 1].toOffset(),
                        end = current.toOffset(),
                        strokeWidth = 6.dp.toPx(),
                        cap = StrokeCap.Round
                    )
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
                    color = routeColor,
                    radius = 11.dp.toPx(),
                    center = center,
                    style = Stroke(width = 2.dp.toPx())
                )
            }
        }
    }
}
