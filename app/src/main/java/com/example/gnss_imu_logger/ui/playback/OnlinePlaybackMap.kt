package com.example.gnss_imu_logger.ui.playback

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.os.Bundle
import android.view.ViewGroup
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.TextButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
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
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.geometry.LatLngBounds
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.style.layers.CircleLayer
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.SymbolLayer
import org.maplibre.android.style.layers.Property
import org.maplibre.android.style.layers.PropertyFactory.circleColor
import org.maplibre.android.style.layers.PropertyFactory.circleRadius
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeColor
import org.maplibre.android.style.layers.PropertyFactory.circleStrokeWidth
import org.maplibre.android.style.layers.PropertyFactory.iconAllowOverlap
import org.maplibre.android.style.layers.PropertyFactory.iconIgnorePlacement
import org.maplibre.android.style.layers.PropertyFactory.iconImage
import org.maplibre.android.style.layers.PropertyFactory.iconOpacity
import org.maplibre.android.style.layers.PropertyFactory.iconRotate
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
    currentLocation: PlaybackLocation?,
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
            // 地図のドラッグとピンチ操作を受け付ける。
            isClickable = true
            isFocusable = true
        }
    }
    var map by remember { mutableStateOf<MapLibreMap?>(null) }
    var style by remember { mutableStateOf<Style?>(null) }
    var status by remember { mutableStateOf<OnlineMapStatus>(OnlineMapStatus.Loading) }
    var followCurrentPosition by remember { mutableStateOf(false) }
    var mapOperationEnabled by remember { mutableStateOf(false) }
    var resetRequest by remember { mutableStateOf(0) }

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
                isDoubleTapGesturesEnabled = false
            }
            readyMap.setStyle(OPEN_FREE_MAP_STYLE_URL) { readyStyle ->
                addSpeedTrackLayers(readyStyle, trackData)
                addGnssGapLayer(readyStyle, trackData.gnssGapFeatures)
                addBrakingLayers(
                    style = readyStyle,
                    intervalFeatures = trackData.brakingIntervalFeatures,
                    startFeatures = trackData.brakingStartFeatures
                )
                addCurrentPositionLayers(readyStyle)
                style = readyStyle
                updateCurrentPosition(readyStyle, currentLocation)
                status = OnlineMapStatus.Ready
            }
        }
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            map = null
            style = null
            mapView.onDestroy()
        }
    }

    // 通常は画面スクロールを優先し、地図操作モード中だけMapViewへタッチを渡す。
    LaunchedEffect(map, mapOperationEnabled) {
        map?.uiSettings?.apply {
            isScrollGesturesEnabled = mapOperationEnabled
            isZoomGesturesEnabled = mapOperationEnabled
            isDoubleTapGesturesEnabled = mapOperationEnabled
        }
        mapView.isClickable = mapOperationEnabled
        mapView.isFocusable = mapOperationEnabled
    }

    LaunchedEffect(map, status, bounds, cameraPaddingPx, resetRequest) {
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

    // 追従中は現在位置を画面中央へ移動し、利用者が選択した縮尺を維持する。
    LaunchedEffect(map, status, followCurrentPosition, currentLocation?.elapsedRealtimeNs) {
        val readyMap = map ?: return@LaunchedEffect
        val location = currentLocation?.takeIf { it.hasValidCoordinate() }
            ?: return@LaunchedEffect
        if (status !is OnlineMapStatus.Ready || !followCurrentPosition) return@LaunchedEffect
        readyMap.animateCamera(
            CameraUpdateFactory.newLatLng(
                LatLng(location.latitudeDeg, location.longitudeDeg)
            )
        )
    }

    // 再生中は現在位置Sourceと表示状態だけを更新し、軌跡全体は再生成しない。
    LaunchedEffect(style, currentLocation) {
        style?.let { readyStyle ->
            updateCurrentPosition(readyStyle, currentLocation)
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
        AndroidView(
            factory = { mapView },
            update = { view ->
                view.setOnTouchListener { touchedView, event ->
                    if (mapOperationEnabled) {
                        touchedView.parent?.requestDisallowInterceptTouchEvent(true)
                    }
                    if (event.actionMasked == android.view.MotionEvent.ACTION_UP ||
                        event.actionMasked == android.view.MotionEvent.ACTION_CANCEL
                    ) {
                        touchedView.parent?.requestDisallowInterceptTouchEvent(false)
                    }
                    false
                }
            },
            modifier = Modifier.fillMaxSize()
        )
        MapStatusText(status)
        Row(
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .fillMaxWidth()
                .background(MaterialTheme.colorScheme.surface.copy(alpha = 0.84f))
                .padding(horizontal = 4.dp),
            horizontalArrangement = Arrangement.SpaceEvenly
        ) {
            TextButton(
                onClick = {
                    mapOperationEnabled = !mapOperationEnabled
                    if (mapOperationEnabled) followCurrentPosition = false
                }
            ) {
                Text(if (mapOperationEnabled) "操作終了" else "地図操作")
            }
            TextButton(
                onClick = {
                    followCurrentPosition = false
                    mapOperationEnabled = false
                    resetRequest++
                }
            ) { Text("全体表示") }
            TextButton(
                onClick = {
                    followCurrentPosition = !followCurrentPosition
                    if (followCurrentPosition) mapOperationEnabled = false
                },
                enabled = currentLocation?.hasValidCoordinate() == true
            ) {
                Text(if (followCurrentPosition) "追従中" else "現在位置を追従")
            }
        }
    }
}

/** 現在位置の円と円内の進行方向矢印を追加する。 */
private fun addCurrentPositionLayers(style: Style) {
    style.addImage(CURRENT_POSITION_ARROW_IMAGE_ID, createHeadingArrowBitmap())
    style.addSource(
        GeoJsonSource(
            CURRENT_POSITION_SOURCE_ID,
            FeatureCollection.fromFeatures(emptyArray())
        )
    )
    style.addLayer(
        CircleLayer(CURRENT_POSITION_CIRCLE_LAYER_ID, CURRENT_POSITION_SOURCE_ID)
            .withProperties(
                circleColor(CURRENT_POSITION_UNAVAILABLE_COLOR),
                circleRadius(CURRENT_POSITION_RADIUS_PX),
                circleStrokeColor(Color.WHITE),
                circleStrokeWidth(CURRENT_POSITION_STROKE_WIDTH_PX)
            )
    )
    style.addLayer(
        SymbolLayer(CURRENT_POSITION_ARROW_LAYER_ID, CURRENT_POSITION_SOURCE_ID)
            .withProperties(
                iconImage(CURRENT_POSITION_ARROW_IMAGE_ID),
                iconAllowOverlap(true),
                iconIgnorePlacement(true),
                iconOpacity(0f)
            )
    )
}

/**
 * 現在位置Sourceと表示状態を更新する。
 * 入力: 再生カーソルが選択したGNSS位置
 * 出力: 現在位置、状態色、円内の方位矢印
 */
private fun updateCurrentPosition(
    style: Style,
    location: PlaybackLocation?
) {
    val source = style.getSourceAs<GeoJsonSource>(CURRENT_POSITION_SOURCE_ID) ?: return
    if (location == null || !location.hasValidCoordinate()) {
        source.setGeoJson(FeatureCollection.fromFeatures(emptyArray()))
        return
    }

    source.setGeoJson(
        Feature.fromGeometry(
            Point.fromLngLat(location.longitudeDeg, location.latitudeDeg)
        )
    )
    style.getLayerAs<CircleLayer>(CURRENT_POSITION_CIRCLE_LAYER_ID)
        ?.setProperties(circleColor(location.currentPositionColor()))

    val bearingDeg = location.bearingDeg?.takeIf { it.isFinite() }
    style.getLayerAs<SymbolLayer>(CURRENT_POSITION_ARROW_LAYER_ID)
        ?.setProperties(
            iconRotate((bearingDeg ?: 0.0).toFloat()),
            iconOpacity(if (bearingDeg == null) 0f else 1f)
        )
}

/** 現在位置円内へ収まる北向き三角矢印を生成する。 */
private fun createHeadingArrowBitmap(): Bitmap {
    val bitmap = Bitmap.createBitmap(
        CURRENT_POSITION_ICON_SIZE_PX,
        CURRENT_POSITION_ICON_SIZE_PX,
        Bitmap.Config.ARGB_8888
    )
    val canvas = Canvas(bitmap)
    val center = CURRENT_POSITION_ICON_SIZE_PX / 2f
    val paint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.FILL
    }
    val path = Path().apply {
        moveTo(center, center - CURRENT_POSITION_ARROW_LENGTH_PX)
        lineTo(center - CURRENT_POSITION_ARROW_HALF_WIDTH_PX, center + CURRENT_POSITION_ARROW_REAR_PX)
        lineTo(center + CURRENT_POSITION_ARROW_HALF_WIDTH_PX, center + CURRENT_POSITION_ARROW_REAR_PX)
        close()
    }
    canvas.drawPath(path, paint)
    return bitmap
}

/** 加減速状態から現在位置円の色を返す。 */
private fun PlaybackLocation.currentPositionColor(): Int = when {
    braking -> CURRENT_POSITION_BRAKING_COLOR
    longitudinalAccelerationMps2 == null -> CURRENT_POSITION_UNAVAILABLE_COLOR
    longitudinalAccelerationMps2 >= ACCELERATION_DISPLAY_THRESHOLD_MPS2 ->
        CURRENT_POSITION_ACCELERATING_COLOR
    longitudinalAccelerationMps2 <= DECELERATION_DISPLAY_THRESHOLD_MPS2 ->
        CURRENT_POSITION_DECELERATING_COLOR
    else -> CURRENT_POSITION_STEADY_COLOR
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


/** GNSS欠損前後をシアン色の専用線として表示する。 */
private fun addGnssGapLayer(
    style: org.maplibre.android.maps.Style,
    features: List<Feature>
) {
    style.addSource(
        GeoJsonSource(
            GNSS_GAP_SOURCE_ID,
            FeatureCollection.fromFeatures(features)
        )
    )
    style.addLayer(
        LineLayer(GNSS_GAP_LAYER_ID, GNSS_GAP_SOURCE_ID).withProperties(
            lineColor(GNSS_GAP_COLOR),
            lineWidth(GNSS_GAP_WIDTH_PX),
            lineCap(Property.LINE_CAP_ROUND),
            lineJoin(Property.LINE_JOIN_ROUND)
        )
    )
}

/** 推定制動区間と推定制動開始位置を速度別軌跡より前面へ表示する。 */
private fun addBrakingLayers(
    style: org.maplibre.android.maps.Style,
    intervalFeatures: List<Feature>,
    startFeatures: List<Feature>
) {
    style.addSource(
        GeoJsonSource(
            BRAKING_INTERVAL_SOURCE_ID,
            FeatureCollection.fromFeatures(intervalFeatures)
        )
    )
    style.addLayer(
        LineLayer(BRAKING_INTERVAL_LAYER_ID, BRAKING_INTERVAL_SOURCE_ID).withProperties(
            lineColor(BRAKING_INTERVAL_COLOR),
            lineWidth(BRAKING_INTERVAL_WIDTH_PX),
            lineCap(Property.LINE_CAP_ROUND),
            lineJoin(Property.LINE_JOIN_ROUND)
        )
    )
    style.addSource(
        GeoJsonSource(
            BRAKING_START_SOURCE_ID,
            FeatureCollection.fromFeatures(startFeatures)
        )
    )
    style.addLayer(
        CircleLayer(BRAKING_START_LAYER_ID, BRAKING_START_SOURCE_ID).withProperties(
            circleColor(BRAKING_START_COLOR),
            circleRadius(BRAKING_START_RADIUS_PX),
            circleStrokeColor(Color.WHITE),
            circleStrokeWidth(BRAKING_START_STROKE_WIDTH_PX)
        )
    )
}

/** 地図用の速度別線分。3秒超のGNSS欠損は線を生成しない。 */
private data class OnlineTrackData(
    val features: Map<SpeedBand, List<Feature>>,
    val gnssGapFeatures: List<Feature>,
    val brakingIntervalFeatures: List<Feature>,
    val brakingStartFeatures: List<Feature>
) {
    companion object {
        fun from(locations: List<PlaybackLocation>): OnlineTrackData {
            val speedFeatures = SpeedBand.entries.associateWith { mutableListOf<Feature>() }
            val gapFeatures = mutableListOf<Feature>()
            val brakingFeatures = mutableListOf<Feature>()
            val brakingStartFeatures = mutableListOf<Feature>()
            var brakingActive = false

            locations.zipWithNext().forEach { (start, end) ->
                if (!start.hasValidCoordinate() || !end.hasValidCoordinate()) return@forEach
                val line = LineString.fromLngLats(
                    listOf(
                        Point.fromLngLat(start.longitudeDeg, start.latitudeDeg),
                        Point.fromLngLat(end.longitudeDeg, end.latitudeDeg)
                    )
                )
                val gap = end.elapsedRealtimeNs - start.elapsedRealtimeNs > GNSS_GAP_NS
                if (gap) {
                    gapFeatures += Feature.fromGeometry(line)
                    brakingActive = false
                    return@forEach
                }

                val speedMps = listOfNotNull(start.speedMps, end.speedMps).averageOrNull()
                if (speedMps != null) {
                    val band = SpeedBand.from(speedMps * MPS_TO_KMH)
                    speedFeatures.getValue(band) += Feature.fromGeometry(line)
                }

                val braking = start.braking && end.braking
                if (braking) {
                    brakingFeatures += Feature.fromGeometry(line)
                    if (!brakingActive) {
                        brakingStartFeatures += Feature.fromGeometry(
                            Point.fromLngLat(start.longitudeDeg, start.latitudeDeg)
                        )
                    }
                }
                brakingActive = braking
            }
            return OnlineTrackData(
                features = speedFeatures,
                gnssGapFeatures = gapFeatures,
                brakingIntervalFeatures = brakingFeatures,
                brakingStartFeatures = brakingStartFeatures
            )
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
private const val GNSS_GAP_SOURCE_ID = "playback-gnss-gap-source"
private const val GNSS_GAP_LAYER_ID = "playback-gnss-gap-layer"
private const val BRAKING_INTERVAL_SOURCE_ID = "playback-braking-interval-source"
private const val BRAKING_INTERVAL_LAYER_ID = "playback-braking-interval-layer"
private const val BRAKING_START_SOURCE_ID = "playback-braking-start-source"
private const val BRAKING_START_LAYER_ID = "playback-braking-start-layer"
private const val GNSS_GAP_WIDTH_PX = 5f
private const val BRAKING_INTERVAL_WIDTH_PX = 8f
private const val BRAKING_START_RADIUS_PX = 6f
private const val BRAKING_START_STROKE_WIDTH_PX = 2f
private val GNSS_GAP_COLOR = Color.rgb(0, 188, 212)
private val BRAKING_INTERVAL_COLOR = Color.rgb(230, 81, 0)
private val BRAKING_START_COLOR = Color.rgb(255, 109, 0)
private const val CURRENT_POSITION_SOURCE_ID = "playback-current-position-source"
private const val CURRENT_POSITION_CIRCLE_LAYER_ID = "playback-current-position-circle-layer"
private const val CURRENT_POSITION_ARROW_LAYER_ID = "playback-current-position-arrow-layer"
private const val CURRENT_POSITION_ARROW_IMAGE_ID = "playback-current-position-arrow-image"
private const val CURRENT_POSITION_RADIUS_PX = 11f
private const val CURRENT_POSITION_STROKE_WIDTH_PX = 2f
private const val CURRENT_POSITION_ICON_SIZE_PX = 22
private const val CURRENT_POSITION_ARROW_LENGTH_PX = 7f
private const val CURRENT_POSITION_ARROW_REAR_PX = 3.15f
private const val CURRENT_POSITION_ARROW_HALF_WIDTH_PX = 3.15f
private const val ACCELERATION_DISPLAY_THRESHOLD_MPS2 = 0.5
private const val DECELERATION_DISPLAY_THRESHOLD_MPS2 = -0.5
private val CURRENT_POSITION_ACCELERATING_COLOR = Color.rgb(0, 188, 212)
private val CURRENT_POSITION_DECELERATING_COLOR = Color.rgb(255, 214, 0)
private val CURRENT_POSITION_BRAKING_COLOR = Color.rgb(255, 109, 0)
private val CURRENT_POSITION_STEADY_COLOR = Color.WHITE
private val CURRENT_POSITION_UNAVAILABLE_COLOR = Color.GRAY
