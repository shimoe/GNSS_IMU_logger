package com.example.gnss_imu_logger.navigation

/**
 * 地理座標。
 * 入力: 緯度[deg]、経度[deg]、楕円体高[m]
 * 出力: WGS84座標
 */
data class GeodeticPosition(
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val altitudeM: Double
) {
    init {
        require(latitudeDeg.isFinite() && latitudeDeg in -90.0..90.0) {
            "緯度は-90 degから90 degの範囲にしてください。"
        }
        require(longitudeDeg.isFinite() && longitudeDeg in -180.0..180.0) {
            "経度は-180 degから180 degの範囲にしてください。"
        }
        require(altitudeM.isFinite()) { "楕円体高は有限値にしてください。" }
    }
}

/**
 * セッション原点を基準にしたENU座標。
 * 入力: 東[m]、北[m]、上[m]
 * 出力: 局所直交座標
 */
data class EnuPosition(
    val eastM: Double,
    val northM: Double,
    val upM: Double
)

/**
 * GNSS位置・速度観測。
 * 入力: gnss.csvの1行
 * 出力: オフライン位置速度推定で使用する観測値
 */
data class GnssObservation(
    val elapsedRealtimeNs: Long,
    val position: GeodeticPosition,
    val speedMps: Double?,
    val bearingDeg: Double?,
    val horizontalAccuracyM: Double?,
    val verticalAccuracyM: Double?,
    val speedAccuracyMps: Double?,
    val bearingAccuracyDeg: Double?,
    val elapsedRealtimeUncertaintyNs: Double?,
    val totalSatellites: Int?,
    val usedSatellites: Int?,
    val usedSatelliteMeanCn0DbHz: Double?,
    val statusAgeMs: Long?,
    val mock: Boolean
)

/** GNSS観測を推定へ適用できるかを示す判定結果。 */
data class GnssObservationDecision(
    val accepted: Boolean,
    val positionStdM: Double,
    val verticalStdM: Double,
    val speedStdMps: Double,
    val bearingUsable: Boolean,
    val reason: String
)
