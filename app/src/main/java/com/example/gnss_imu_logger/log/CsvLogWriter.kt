package com.example.gnss_imu_logger.log

import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileWriter

/**
 * CSVをバッファ付きで追記する。
 * 入力: 1行分の値
 * 出力: UTF-8 CSV
 */
class CsvLogWriter(file: File, header: List<String>) : Closeable {
    private val lock = Any()
    private val out = BufferedWriter(FileWriter(file, true), BUFFER_SIZE)

    init {
        if (file.length() == 0L) {
            out.appendLine(header.joinToString(",", transform = ::escape))
            out.flush()
        }
    }

    fun write(values: List<Any?>) = synchronized(lock) {
        out.appendLine(values.joinToString(",") { escape(it?.toString().orEmpty()) })
    }

    fun flush() = synchronized(lock) {
        out.flush()
    }

    override fun close() = synchronized(lock) {
        out.flush()
        out.close()
    }

    private fun escape(value: String): String {
        val escaped = value.replace("\"", "\"\"")
        return "\"$escaped\""
    }

    companion object {
        private const val BUFFER_SIZE = 64 * 1024
    }
}
