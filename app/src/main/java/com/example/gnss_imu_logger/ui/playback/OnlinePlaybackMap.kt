package com.example.gnss_imu_logger.ui.playback

import android.graphics.Color
import android.os.Bundle
import android.view.ViewGroup
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleOwner
import com.example.gnss_imu_logger.playback.PlaybackLocation
import com.example.gnss_imu_logger.playback.TrackBounds
import kotlinx.coroutines.delay
import org.maplibre.android.MapLibre
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLngBounds
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.Property
import org.maplibre.android.style.layers.PropertyFactory.lineCap
import org.maplibre.android.style.layers.PropertyFactory.lineColor
import org.maplibre.android.style.layers.PropertyFactory.lineJoin
import org.maplibre.android.style.layers.PropertyFactory.lineWidth
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.geojson.Feature
import org.maplibre.geojson.FeatureCollection
import org.maplibre.geojson.LineString
import org.maplibre.geojson.Point

/** オンライン背景地図の読込状態。 */
internal sealed interface OnlineMapStatus {
    data object Loading : OnlineMapStatus
    data object Ready : OnlineMapStatus
    data class Error(val message: String) : OnlineMapStatus
}

/**
 * OpenFreeMapへ速度別GNSS軌跡を表示する。
 * 入力: GNSS位置列、軌跡範囲、状態通知
 * 出力: 緯度経度へ一致した速度別軌跡と背景地図
 */
@Composable
internal fun OnlinePlaybackMap(
    locations: List<PlaybackLocation>,
    bounds: TrackBounds?,
    onStatusChanged: (OnlineMapStatus) -> Unit,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraPaddingPx = with(LocalDensity.current) { CAMERA_PADDING_DP.dp.roundToPx() }
    val trackData = remember(locations) { OnlineTrackData.from(locations) }
    val mapView = remember {
        MapLibre.getInstance(context)
        MapView(context).apply {
            layoutParams = ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
            // MapLibreへの操作移行前なので、既存Canvasと競合しないようにする。
            isClickable = false
            isFocusable = false
        }
    }
    var map by remember { mutableStateOf<MapLibreMap?>(null) }
    var status by remember { mutableStateOf<OnlineMapStatus>(OnlineMapStatus.Loading) }

    LaunchedEffect(status) { onStatusChanged(status) }

    DisposableEffect(mapView, lifecycleOwner, trackData) {
        mapView.onCreate(Bundle())
        val observer = object : DefaultLifecycleObserver {
            override fun onStart(owner: LifecycleOwner) = mapView.onStart()
            override fun onResume(owner: LifecycleOwner) = mapView.onResume()
            override fun onPause(owner: LifecycleOwner) = mapView.onPause()
            override fun onStop(owner: LifecycleOwner) = mapView.onStop()
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        mapView.getMapAsync { readyMap ->
            map = readyMap
            readyMap.uiSettings.apply {
                isCompassEnabled = false
                isLogoEnabled = false
                isAttributionEnabled = false
                isRotateGesturesEnabled = false
                isTiltGesturesEnabled = false
                isScrollGesturesEnabled = false
                isZoomGesturesEnabled = false
            }
            readyMap.setStyle(OPEN_FREE_MAP_STYLE_URL) { style ->
                addSpeedTrackLayers(style, trackData)
                status = OnlineMapStatus.Ready
            }
        }
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            map = null
            mapView.onDestroy()
        }
    }

    LaunchedEffect(map, status, bounds, cameraPaddingPx) {
        val readyMap = map ?: return@LaunchedEffect
        val trackBounds = bounds ?: return@LaunchedEffect
        if (status !is OnlineMapStatus.Ready) return@LaunchedEffect
        mapView.post {
            if (mapView.width <= 0 || mapView.height <= 0) return@post
            val mapBounds = LatLngBounds.from(
                trackBounds.maximumLatitudeDeg,
                trackBounds.maximumLongitudeDeg,
                trackBounds.minimumLatitudeDeg,
                trackBounds.minimumLongitudeDeg
            )
            readyMap.moveCamera(
                CameraUpdateFactory.newLatLngBounds(mapBounds, cameraPaddingPx)
            )
        }
    }

    LaunchedEffect(mapView) {
        delay(MAP_LOAD_TIMEOUT_MS)
        if (status is OnlineMapStatus.Loading) {
            status = OnlineMapStatus.Error(
                "オンライン地図を読み込めないため簡易表示を使用しています"
            )
        }
    }

    Box(modifier = modifier.fillMaxSize()) {
        AndroidView(factory = { mapView }, modifier = Modifier.fillMaxSize())
        MapStatusText(status)
    }
}

/** 速度区分ごとのGeoJSON SourceとLineLayerを追加する。 */
private fun addSpeedTrackLayers(
    style: org.maplibre.android.maps.Style,
    trackData: OnlineTrackData
) {
    SpeedBand.entries.forEach { band ->
        val sourceId = "playback-speed-source-${band.id}"
        val layerId = "playback-speed-layer-${band.id}"
        style.addSource(
            GeoJsonSource(
                sourceId,
                FeatureCollection.fromFeatures(trackData.features[band].orEmpty())
            )
        )
        style.addLayer(
            LineLayer(layerId, sourceId).withProperties(
                lineColor(band.color),
                lineWidth(TRACK_WIDTH_PX),
                lineCap(Property.LINE_CAP_ROUND),
                lineJoin(Property.LINE_JOIN_ROUND)
            )
        )
    }
}

/** 地図用の速度別線分。3秒超のGNSS欠損は線を生成しない。 */
private data class OnlineTrackData(
    val features: Map<SpeedBand, List<Feature>>
) {
    companion object {
        fun from(locations: List<PlaybackLocation>): OnlineTrackData {
            val result = SpeedBand.entries.associateWith { mutableListOf<Feature>() }
            locations.zipWithNext().forEach { (start, end) ->
                if (!start.hasValidCoordinate() || !end.hasValidCoordinate()) return@forEach
                if (end.elapsedRealtimeNs - start.elapsedRealtimeNs > GNSS_GAP_NS) return@forEach
                val speedMps = listOfNotNull(start.speedMps, end.speedMps).averageOrNull()
                    ?: return@forEach
                val band = SpeedBand.from(speedMps * MPS_TO_KMH)
                val line = LineString.fromLngLats(
                    listOf(
                        Point.fromLngLat(start.longitudeDeg, start.latitudeDeg),
                        Point.fromLngLat(end.longitudeDeg, end.latitudeDeg)
                    )
                )
                result.getValue(band) += Feature.fromGeometry(line)
            }
            return OnlineTrackData(result)
        }
    }
}

private enum class SpeedBand(val id: String, val color: Int) {
    LOW("low", Color.rgb(30, 136, 229)),
    MEDIUM("medium", Color.rgb(67, 160, 71)),
    HIGH("high", Color.rgb(251, 140, 0)),
    VERY_HIGH("very-high", Color.rgb(216, 27, 96));

    companion object {
        fun from(speedKmh: Double): SpeedBand = when {
            speedKmh < 20.0 -> LOW
            speedKmh < 50.0 -> MEDIUM
            speedKmh < 80.0 -> HIGH
            else -> VERY_HIGH
        }
    }
}

private fun PlaybackLocation.hasValidCoordinate(): Boolean =
    latitudeDeg.isFinite() && longitudeDeg.isFinite() &&
        latitudeDeg in -90.0..90.0 && longitudeDeg in -180.0..180.0

private fun List<Double>.averageOrNull(): Double? =
    takeIf { it.isNotEmpty() }?.average()?.takeIf { it.isFinite() }

@Composable
private fun MapStatusText(status: OnlineMapStatus) {
    val text = when (status) {
        OnlineMapStatus.Loading -> "地図を読み込んでいます"
        OnlineMapStatus.Ready -> "© OpenFreeMap  © OpenStreetMap contributors"
        is OnlineMapStatus.Error -> status.message
    }
    Text(
        text = text,
        modifier = Modifier
            .background(MaterialTheme.colorScheme.surface.copy(alpha = 0.84f))
            .padding(horizontal = 6.dp, vertical = 2.dp),
        color = if (status is OnlineMapStatus.Error) {
            MaterialTheme.colorScheme.error
        } else {
            MaterialTheme.colorScheme.onSurface
        },
        style = MaterialTheme.typography.labelSmall
    )
}

private const val OPEN_FREE_MAP_STYLE_URL = "https://tiles.openfreemap.org/styles/liberty"
private const val MAP_LOAD_TIMEOUT_MS = 15_000L
private const val CAMERA_PADDING_DP = 24
private const val TRACK_WIDTH_PX = 4f
private const val GNSS_GAP_NS = 3_000_000_000L
private const val MPS_TO_KMH = 3.6
