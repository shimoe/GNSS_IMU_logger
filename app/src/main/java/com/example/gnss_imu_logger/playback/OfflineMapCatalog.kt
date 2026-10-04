package com.example.gnss_imu_logger.playback

import java.io.File

/** 端末内へ事前配置したオフライン地図一式。 */
data class OfflineMapPackage(
    val directory: File,
    val tilesFile: File,
    val styleFile: File
)

/** Documents/maps内のMBTilesとstyle.jsonを検索する。 */
object OfflineMapCatalog {
    fun find(documentsDirectory: File): OfflineMapPackage? {
        val mapDirectory = File(documentsDirectory, "maps")
        if (!mapDirectory.isDirectory) return null
        val tiles = mapDirectory.listFiles()
            ?.filter { it.isFile && it.extension.equals("mbtiles", ignoreCase = true) }
            ?.sortedBy { it.name }
            ?.firstOrNull()
            ?: return null
        val style = File(mapDirectory, "style.json").takeIf { it.isFile } ?: return null
        return OfflineMapPackage(mapDirectory, tiles, style)
    }
}
