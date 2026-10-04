package com.example.gnss_imu_logger.ui.playback

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
import com.example.gnss_imu_logger.playback.TrackBounds
import kotlinx.coroutines.delay
import org.maplibre.android.MapLibre
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLngBounds
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView

/** オンライン背景地図の読込状態。 */
internal sealed interface OnlineMapStatus {
    data object Loading : OnlineMapStatus
    data object Ready : OnlineMapStatus
    data class Error(val message: String) : OnlineMapStatus
}

/**
 * OpenFreeMapを軌跡枠の背景へ表示し、GNSS軌跡範囲へカメラを合わせる。
 * 入力: GNSS軌跡の緯度経度範囲、状態通知
 * 出力: 軌跡範囲へ同期したMapLibre MapView、読込状態、帰属表示
 */
@Composable
internal fun OnlinePlaybackMap(
    bounds: TrackBounds?,
    onStatusChanged: (OnlineMapStatus) -> Unit,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val cameraPaddingPx = with(LocalDensity.current) { CAMERA_PADDING_DP.dp.roundToPx() }
    val mapView = remember {
        MapLibre.getInstance(context)
        MapView(context).apply {
            layoutParams = ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
            // Canvas側の操作と競合しないよう、地図操作は次段階まで無効にする。
            isClickable = false
            isFocusable = false
        }
    }
    var map by remember { mutableStateOf<MapLibreMap?>(null) }
    var status by remember { mutableStateOf<OnlineMapStatus>(OnlineMapStatus.Loading) }

    LaunchedEffect(status) { onStatusChanged(status) }

    DisposableEffect(mapView, lifecycleOwner) {
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
            readyMap.setStyle(OPEN_FREE_MAP_STYLE_URL) {
                status = OnlineMapStatus.Ready
            }
        }
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            map = null
            mapView.onDestroy()
        }
    }

    // Style読込後かつMapViewの寸法確定後に、軌跡全体が表示されるカメラへ移動する。
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
