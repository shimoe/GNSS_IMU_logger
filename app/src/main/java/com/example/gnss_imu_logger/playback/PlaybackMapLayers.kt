package com.example.gnss_imu_logger.playback

/** 軌跡画面を構成する表示レイヤー。 */
enum class PlaybackMapLayer {
    BACKGROUND_MAP,
    SPEED_TRACK,
    GNSS_GAP,
    BRAKING_INTERVAL,
    BRAKING_START,
    CURRENT_POSITION,
    VEHICLE_DYNAMICS_HUD
}

/** 軌跡画面のレイヤー表示設定。 */
data class PlaybackMapLayerState(
    val visibleLayers: Set<PlaybackMapLayer> = PlaybackMapLayer.entries.toSet()
) {
    fun isVisible(layer: PlaybackMapLayer): Boolean = layer in visibleLayers
}
