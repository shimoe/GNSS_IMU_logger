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
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.delay
import org.maplibre.android.MapLibre
import org.maplibre.android.maps.MapView

/** オンライン背景地図の読込状態。 */
internal sealed interface OnlineMapStatus {
    data object Loading : OnlineMapStatus
    data object Ready : OnlineMapStatus
    data class Error(val message: String) : OnlineMapStatus
}

/**
 * OpenFreeMapを軌跡枠の背景へ表示する。
 * 入力: 状態通知
 * 出力: MapLibre MapView、読込状態、帰属表示
 */
@Composable
internal fun OnlinePlaybackMap(
    onStatusChanged: (OnlineMapStatus) -> Unit,
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    val mapView = remember {
        MapLibre.getInstance(context)
        MapView(context).apply {
            layoutParams = ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT
            )
            // 既存の簡易軌跡操作を維持し、地図操作は次段階で有効化する。
            isClickable = false
            isFocusable = false
        }
    }
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
        mapView.getMapAsync { map ->
            map.uiSettings.apply {
                isCompassEnabled = false
                isLogoEnabled = false
                isAttributionEnabled = false
                isRotateGesturesEnabled = false
                isTiltGesturesEnabled = false
                isScrollGesturesEnabled = false
                isZoomGesturesEnabled = false
            }
            map.setStyle(OPEN_FREE_MAP_STYLE_URL) {
                status = OnlineMapStatus.Ready
            }
        }
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            mapView.onDestroy()
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
