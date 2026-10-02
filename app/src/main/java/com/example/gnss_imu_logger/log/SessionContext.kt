package com.example.gnss_imu_logger.log

import android.content.Context
import android.os.Build
import android.os.SystemClock
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.model.SessionEvent
import com.example.gnss_imu_logger.model.SessionMetadata
import com.example.gnss_imu_logger.model.SessionStatus
import com.example.gnss_imu_logger.time.TimeSynchronizer
import java.io.Closeable
import java.io.File
import java.time.Instant
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter

/**
 * 1回の計測セッションで共有する保存先と時刻基準を管理する。
 * 入力: Android Context、計測モード
 * 出力: セッションディレクトリ、session.json、events.csv
 */
class SessionContext(context: Context, val mode: LogMode) : Closeable {
    val time = TimeSynchronizer()
    val sessionId: String = SESSION_FORMAT.format(Instant.now())
    val directory = File(context.getExternalFilesDir(null), "Documents/$sessionId")
    private val incompleteFile = File(directory, "session.incomplete")
    private val metadataFile = File(directory, "session.json")
    private val metadataWriter: SessionMetadataWriter
    val events: EventWriter
    private var metadata: SessionMetadata

    init {
        if (!directory.mkdirs() && !directory.isDirectory) {
            throw IllegalStateException("セッション保存先を作成できませんでした")
        }
        incompleteFile.createNewFile()
        metadataWriter = SessionMetadataWriter(metadataFile)
        events = EventWriter(File(directory, "events.csv"))
        metadata = SessionMetadata(
            sessionId = sessionId,
            status = SessionStatus.RECORDING,
            mode = mode,
            manufacturer = Build.MANUFACTURER,
            model = Build.MODEL,
            androidVersion = Build.VERSION.RELEASE,
            apiLevel = Build.VERSION.SDK_INT,
            startElapsedNs = time.startElapsedNs,
            startUtcNs = time.startUtcNs,
            initialUtcOffsetNs = time.initialUtcOffsetNs
        )
        metadataWriter.write(metadata)
        event(EventLevel.INFO, EventType.SESSION_CREATED, "計測セッションを作成しました")
    }

    fun event(level: EventLevel, type: EventType, message: String) {
        val elapsedNs = SystemClock.elapsedRealtimeNanos()
        events.write(
            SessionEvent(
                elapsedRealtimeNs = elapsedNs,
                sessionElapsedNs = time.sessionElapsedNs(elapsedNs),
                utcNs = time.estimateUtcNs(elapsedNs),
                level = level,
                type = type,
                message = message
            )
        )
    }

    fun complete() {
        val endElapsedNs = SystemClock.elapsedRealtimeNanos()
        event(EventLevel.INFO, EventType.SESSION_COMPLETED, "計測セッションを正常終了しました")
        events.flush()
        metadata = metadata.copy(
            status = SessionStatus.COMPLETED,
            endElapsedNs = endElapsedNs,
            endUtcNs = time.estimateUtcNs(endElapsedNs)
        )
        metadataWriter.write(metadata)
        if (!incompleteFile.delete() && incompleteFile.exists()) {
            throw IllegalStateException("未完了マーカーを削除できませんでした")
        }
    }

    fun fail(message: String) {
        val endElapsedNs = SystemClock.elapsedRealtimeNanos()
        event(EventLevel.ERROR, EventType.WRITE_ERROR, message)
        events.flush()
        metadata = metadata.copy(
            status = SessionStatus.ERROR,
            endElapsedNs = endElapsedNs,
            endUtcNs = time.estimateUtcNs(endElapsedNs),
            errorMessage = message
        )
        metadataWriter.write(metadata)
    }

    override fun close() = events.close()

    companion object {
        private val SESSION_FORMAT = DateTimeFormatter
            .ofPattern("yyyyMMdd_HHmmss_SSS")
            .withZone(ZoneOffset.UTC)
    }
}
