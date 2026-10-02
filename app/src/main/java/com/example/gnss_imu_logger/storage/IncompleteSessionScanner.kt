package com.example.gnss_imu_logger.storage

import java.io.File

/**
 * 正常終了していない過去セッションを検出する。
 * 入力: Documentsディレクトリ
 * 出力: session.incompleteを含むセッションディレクトリ一覧
 */
object IncompleteSessionScanner {
    fun scan(documentsDirectory: File): List<File> =
        documentsDirectory.listFiles()
            ?.filter { it.isDirectory && File(it, "session.incomplete").exists() }
            ?.sortedBy { it.name }
            .orEmpty()
}
