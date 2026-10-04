package com.example.gnss_imu_logger.playback

import org.json.JSONObject
import java.io.File

/** MapLibreへ渡す端末内地図スタイル。 */
data class OfflineMapStyle(
    val json: String,
    val tilesUri: String
)

/**
 * style.jsonのMBTilesプレースホルダーを端末内URIへ置き換える。
 * 入力: OfflineMapPackage
 * 出力: MapLibreへ渡せるstyle.json文字列とmbtiles URI
 */
object OfflineMapStyleLoader {
    fun load(mapPackage: OfflineMapPackage): OfflineMapStyle {
        require(mapPackage.tilesFile.isFile) {
            "MBTilesを読み込めません: ${mapPackage.tilesFile.absolutePath}"
        }
        require(mapPackage.styleFile.isFile) {
            "地図スタイルを読み込めません: ${mapPackage.styleFile.absolutePath}"
        }

        val tilesUri = mapPackage.tilesFile.toMbtilesUri()
        val source = mapPackage.styleFile.readText(Charsets.UTF_8)
        require(source.contains(MBTILES_PLACEHOLDER)) {
            "style.jsonに${MBTILES_PLACEHOLDER}を指定してください"
        }
        val resolved = source.replace(MBTILES_PLACEHOLDER, tilesUri)
        val json = runCatching { JSONObject(resolved) }.getOrElse { error ->
            throw IllegalArgumentException(
                "style.jsonのJSON形式が不正です: ${error.message}",
                error
            )
        }
        require(json.optInt("version") == MAPLIBRE_STYLE_VERSION) {
            "style.jsonのversionは${MAPLIBRE_STYLE_VERSION}を指定してください"
        }
        require(json.optJSONObject("sources") != null) {
            "style.jsonにsourcesがありません"
        }
        require(json.optJSONArray("layers") != null) {
            "style.jsonにlayersがありません"
        }
        return OfflineMapStyle(
            json = json.toString(),
            tilesUri = tilesUri
        )
    }

    private fun File.toMbtilesUri(): String =
        "mbtiles://${absolutePath.replace('\\', '/')}"

    const val MBTILES_PLACEHOLDER = "__MBTILES_URI__"
    private const val MAPLIBRE_STYLE_VERSION = 8
}
