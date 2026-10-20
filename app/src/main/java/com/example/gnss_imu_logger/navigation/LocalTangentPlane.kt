package com.example.gnss_imu_logger.navigation

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * WGS84座標とセッション原点基準のENU座標を相互変換する。
 * 入力: 原点および変換対象の緯度[deg]、経度[deg]、楕円体高[m]
 * 出力: ENU座標[m]またはWGS84座標
 */
class LocalTangentPlane(val origin: GeodeticPosition) {
    private val originLatitudeRad = Math.toRadians(origin.latitudeDeg)
    private val originLongitudeRad = Math.toRadians(origin.longitudeDeg)
    private val originEcef = geodeticToEcef(origin)

    /** WGS84座標をセッション原点基準のENU座標へ変換する。 */
    fun toEnu(position: GeodeticPosition): EnuPosition {
        val ecef = geodeticToEcef(position)
        val dx = ecef.x - originEcef.x
        val dy = ecef.y - originEcef.y
        val dz = ecef.z - originEcef.z
        val sinLat = sin(originLatitudeRad)
        val cosLat = cos(originLatitudeRad)
        val sinLon = sin(originLongitudeRad)
        val cosLon = cos(originLongitudeRad)
        return EnuPosition(
            eastM = -sinLon * dx + cosLon * dy,
            northM = -sinLat * cosLon * dx - sinLat * sinLon * dy + cosLat * dz,
            upM = cosLat * cosLon * dx + cosLat * sinLon * dy + sinLat * dz
        )
    }

    /** セッション原点基準のENU座標をWGS84座標へ変換する。 */
    fun toGeodetic(position: EnuPosition): GeodeticPosition {
        val sinLat = sin(originLatitudeRad)
        val cosLat = cos(originLatitudeRad)
        val sinLon = sin(originLongitudeRad)
        val cosLon = cos(originLongitudeRad)
        val dx = -sinLon * position.eastM - sinLat * cosLon * position.northM +
            cosLat * cosLon * position.upM
        val dy = cosLon * position.eastM - sinLat * sinLon * position.northM +
            cosLat * sinLon * position.upM
        val dz = cosLat * position.northM + sinLat * position.upM
        return ecefToGeodetic(
            EcefPosition(originEcef.x + dx, originEcef.y + dy, originEcef.z + dz)
        )
    }

    private fun geodeticToEcef(position: GeodeticPosition): EcefPosition {
        val latitudeRad = Math.toRadians(position.latitudeDeg)
        val longitudeRad = Math.toRadians(position.longitudeDeg)
        val sinLat = sin(latitudeRad)
        val cosLat = cos(latitudeRad)
        val radius = SEMI_MAJOR_AXIS_M / sqrt(1.0 - ECCENTRICITY_SQUARED * sinLat * sinLat)
        return EcefPosition(
            x = (radius + position.altitudeM) * cosLat * cos(longitudeRad),
            y = (radius + position.altitudeM) * cosLat * sin(longitudeRad),
            z = (radius * (1.0 - ECCENTRICITY_SQUARED) + position.altitudeM) * sinLat
        )
    }

    private fun ecefToGeodetic(position: EcefPosition): GeodeticPosition {
        val longitudeRad = kotlin.math.atan2(position.y, position.x)
        val horizontalM = sqrt(position.x * position.x + position.y * position.y)
        var latitudeRad = kotlin.math.atan2(position.z, horizontalM * (1.0 - ECCENTRICITY_SQUARED))
        var altitudeM = 0.0
        repeat(8) {
            val sinLat = sin(latitudeRad)
            val radius = SEMI_MAJOR_AXIS_M / sqrt(1.0 - ECCENTRICITY_SQUARED * sinLat * sinLat)
            altitudeM = horizontalM / cos(latitudeRad) - radius
            latitudeRad = kotlin.math.atan2(
                position.z,
                horizontalM * (1.0 - ECCENTRICITY_SQUARED * radius / (radius + altitudeM))
            )
        }
        return GeodeticPosition(
            latitudeDeg = Math.toDegrees(latitudeRad),
            longitudeDeg = Math.toDegrees(longitudeRad),
            altitudeM = altitudeM
        )
    }

    private data class EcefPosition(val x: Double, val y: Double, val z: Double)

    companion object {
        private const val SEMI_MAJOR_AXIS_M = 6_378_137.0
        private const val ECCENTRICITY_SQUARED = 6.69437999014e-3
    }
}
