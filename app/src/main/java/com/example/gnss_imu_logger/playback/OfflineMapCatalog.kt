package com.example.gnss_imu_logger.playback

import java.io.File

/** 端末内へ事前配置したオフライン地図一式。 */
data class OfflineMapPackage(
    val directory: File,
    val tilesFile: File,
    val styleFile: File
)

/** オフライン地図の検出結果。 */
sealed interface OfflineMapStatus {
    data class Available(val mapPackage: OfflineMapPackage) : OfflineMapStatus
    data class Unavailable(val message: String) : OfflineMapStatus
}

/**
 * Documents/maps内のMBTilesとstyle.jsonを検索する。
 * 入力: アプリのDocumentsディレクトリ
 * 出力: 地図一式、または不足内容を示す日本語メッセージ
 */
object OfflineMapCatalog {
    fun inspect(documentsDirectory: File): OfflineMapStatus {
        val mapDirectory = File(documentsDirectory, MAP_DIRECTORY_NAME)
        if (!mapDirectory.isDirectory) {
            return OfflineMapStatus.Unavailable(
                "オフライン地図フォルダーがありません: Documents/maps"
            )
        }
        val tiles = mapDirectory.listFiles()
            ?.filter { it.isFile && it.extension.equals(MBTILES_EXTENSION, ignoreCase = true) }
            ?.sortedBy { it.name }
            ?.firstOrNull()
            ?: return OfflineMapStatus.Unavailable(
                "MBTilesがありません: Documents/maps/*.mbtiles"
            )
        val style = File(mapDirectory, STYLE_FILE_NAME)
        if (!style.isFile) {
            return OfflineMapStatus.Unavailable(
                "地図スタイルがありません: Documents/maps/style.json"
            )
        }
        return OfflineMapStatus.Available(
            OfflineMapPackage(mapDirectory, tiles, style)
        )
    }

    fun find(documentsDirectory: File): OfflineMapPackage? =
        (inspect(documentsDirectory) as? OfflineMapStatus.Available)?.mapPackage

    private const val MAP_DIRECTORY_NAME = "maps"
    private const val MBTILES_EXTENSION = "mbtiles"
    private const val STYLE_FILE_NAME = "style.json"
}
