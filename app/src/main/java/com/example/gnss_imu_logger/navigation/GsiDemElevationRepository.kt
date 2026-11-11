package com.example.gnss_imu_logger.navigation

import android.graphics.BitmapFactory
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import kotlin.math.PI
import kotlin.math.floor
import kotlin.math.ln
import kotlin.math.tan

/**
 * 国土地理院のPNG標高タイルを取得し、指定地点の地表標高を返す。
 * 入力: 緯度・経度[deg]
 * 出力: 標高[MSL m]、DEM種別、評価用標準偏差[m]
 */
class GsiDemElevationRepository(
    private val cacheDirectory: File,
    private val connectTimeoutMs: Int = 10_000,
    private val readTimeoutMs: Int = 15_000
) {
    fun elevation(latitudeDeg: Double, longitudeDeg: Double): GsiElevationResult {
        if (!latitudeDeg.isFinite() || !longitudeDeg.isFinite() ||
            latitudeDeg !in -WEB_MERCATOR_LIMIT_DEG..WEB_MERCATOR_LIMIT_DEG ||
            longitudeDeg !in -180.0..180.0
        ) {
            return GsiElevationResult.invalid("COORDINATE")
        }
        SOURCES.forEach { source ->
            val coordinate = tileCoordinate(latitudeDeg, longitudeDeg, source.zoom)
            val file = cacheFile(source, coordinate)
            val downloaded = file.isFile || download(source, coordinate, file)
            if (!downloaded) return@forEach
            val bitmap = BitmapFactory.decodeFile(file.path) ?: run {
                file.delete()
                return@forEach
            }
            val pixel = bitmap.getPixel(coordinate.pixelX, coordinate.pixelY)
            bitmap.recycle()
            val red = pixel shr 16 and 0xff
            val green = pixel shr 8 and 0xff
            val blue = pixel and 0xff
            val elevation = decodeElevation(red, green, blue) ?: return@forEach
            return GsiElevationResult(
                valid = true,
                elevationMslM = elevation,
                source = source.id,
                observationStdM = source.observationStdM,
                zoom = source.zoom,
                tileX = coordinate.tileX,
                tileY = coordinate.tileY,
                pixelX = coordinate.pixelX,
                pixelY = coordinate.pixelY,
                rejectionReason = ""
            )
        }
        return GsiElevationResult.invalid("TILE_UNAVAILABLE")
    }

    private fun download(source: GsiDemSource, coordinate: TileCoordinate, file: File): Boolean {
        file.parentFile?.mkdirs()
        val url = URL(
            "$BASE_URL/${source.id}/${source.zoom}/${coordinate.tileX}/${coordinate.tileY}.png"
        )
        val connection = url.openConnection() as HttpURLConnection
        return try {
            connection.connectTimeout = connectTimeoutMs
            connection.readTimeout = readTimeoutMs
            connection.requestMethod = "GET"
            connection.setRequestProperty("User-Agent", USER_AGENT)
            if (connection.responseCode != HttpURLConnection.HTTP_OK) return false
            val temporary = File(file.path + ".tmp")
            connection.inputStream.use { input ->
                temporary.outputStream().use { output -> input.copyTo(output) }
            }
            if (!temporary.renameTo(file)) {
                temporary.copyTo(file, overwrite = true)
                temporary.delete()
            }
            true
        } catch (_: Exception) {
            false
        } finally {
            connection.disconnect()
        }
    }

    private fun cacheFile(source: GsiDemSource, coordinate: TileCoordinate): File =
        cacheDirectory.resolve(
            "${source.id}/${source.zoom}/${coordinate.tileX}/${coordinate.tileY}.png"
        )

    private fun tileCoordinate(
        latitudeDeg: Double,
        longitudeDeg: Double,
        zoom: Int
    ): TileCoordinate {
        val scale = 1 shl zoom
        val worldX = (longitudeDeg + 180.0) / 360.0 * scale
        val latitudeRad = latitudeDeg * PI / 180.0
        val worldY = (1.0 - ln(tan(latitudeRad) + 1.0 / kotlin.math.cos(latitudeRad)) / PI) *
            0.5 * scale
        val tileX = floor(worldX).toInt().coerceIn(0, scale - 1)
        val tileY = floor(worldY).toInt().coerceIn(0, scale - 1)
        return TileCoordinate(
            tileX = tileX,
            tileY = tileY,
            pixelX = floor((worldX - tileX) * TILE_SIZE).toInt().coerceIn(0, TILE_SIZE - 1),
            pixelY = floor((worldY - tileY) * TILE_SIZE).toInt().coerceIn(0, TILE_SIZE - 1)
        )
    }

    private fun decodeElevation(red: Int, green: Int, blue: Int): Double? {
        val value = (red shl 16) or (green shl 8) or blue
        if (value == NO_DATA_VALUE) return null
        val signed = if (value < SIGN_BIT) value else value - VALUE_RANGE
        return signed * ELEVATION_RESOLUTION_M
    }

    private data class TileCoordinate(
        val tileX: Int,
        val tileY: Int,
        val pixelX: Int,
        val pixelY: Int
    )

    companion object {
        private const val BASE_URL = "https://cyberjapandata.gsi.go.jp/xyz"
        private const val USER_AGENT = "GNSS_IMU_logger/1.0 (GSI DEM evaluation)"
        private const val TILE_SIZE = 256
        private const val NO_DATA_VALUE = 1 shl 23
        private const val SIGN_BIT = 1 shl 23
        private const val VALUE_RANGE = 1 shl 24
        private const val ELEVATION_RESOLUTION_M = 0.01
        private const val WEB_MERCATOR_LIMIT_DEG = 85.05112878

        private val SOURCES = listOf(
            GsiDemSource("dem1a_png", 17, 2.0),
            GsiDemSource("dem5a_png", 15, 2.0),
            GsiDemSource("dem5b_png", 15, 3.0),
            GsiDemSource("dem5c_png", 15, 4.0),
            GsiDemSource("dem_png", 14, 8.0)
        )
    }
}

data class GsiDemSource(
    val id: String,
    val zoom: Int,
    val observationStdM: Double
)

data class GsiElevationResult(
    val valid: Boolean,
    val elevationMslM: Double?,
    val source: String,
    val observationStdM: Double?,
    val zoom: Int?,
    val tileX: Int?,
    val tileY: Int?,
    val pixelX: Int?,
    val pixelY: Int?,
    val rejectionReason: String
) {
    companion object {
        fun invalid(reason: String) = GsiElevationResult(
            valid = false,
            elevationMslM = null,
            source = "",
            observationStdM = null,
            zoom = null,
            tileX = null,
            tileY = null,
            pixelX = null,
            pixelY = null,
            rejectionReason = reason
        )
    }
}
