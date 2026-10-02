package com.example.gnss_imu_logger.log

import com.example.gnss_imu_logger.model.SessionEvent
import java.io.BufferedWriter
import java.io.Closeable
import java.io.File
import java.io.FileWriter

/**
 * セッションイベントをCSVへ追記する。
 * 入力: SessionEvent
 * 出力: events.csv
 */
class EventWriter(file: File) : Closeable {
    private val lock = Any()
    private val out = BufferedWriter(FileWriter(file, true), 16 * 1024)

    init {
        if (file.length() == 0L) {
            out.appendLine(
                "elapsed_realtime_ns,session_elapsed_ns,utc_ns," +
                    "level,event_type,message"
            )
            out.flush()
        }
    }

    fun write(event: SessionEvent) = synchronized(lock) {
        val fields = listOf(
            event.elapsedRealtimeNs.toString(),
            event.sessionElapsedNs.toString(),
            event.utcNs.toString(),
            event.level.name,
            event.type.name,
            event.message
        )
        out.appendLine(fields.joinToString(",", transform = ::escapeCsv))
    }

    fun flush() = synchronized(lock) {
        out.flush()
    }

    override fun close() = synchronized(lock) {
        out.flush()
        out.close()
    }

    private fun escapeCsv(value: String): String {
        val escaped = value.replace("\"", "\"\"")
        return "\"$escaped\""
    }
}
