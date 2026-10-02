package com.example.gnss_imu_logger.log

import java.io.BufferedOutputStream
import java.io.Closeable
import java.io.DataOutputStream
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * 6要素のセンサーデータを固定長バイナリへ保存する。
 * 入力: elapsedRealtimeNs、測定3軸、推定バイアス3軸、精度
 * 出力: 16バイトのヘッダーと36バイトの固定長レコード
 */
class BinarySensorWriter(
    file: File,
    private val sensorType: Int,
    private val onError: (String, Throwable?) -> Unit
) : Closeable {
    private val queue = ArrayBlockingQueue<Record>(MAX_QUEUE_SIZE)
    private val accepting = AtomicBoolean(true)
    private val closed = AtomicBoolean(false)
    private val written = AtomicLong(0L)
    private val dropped = AtomicLong(0L)
    private val output = DataOutputStream(
        BufferedOutputStream(FileOutputStream(file), OUTPUT_BUFFER_SIZE)
    )
    private val writerThread = Thread(::writeLoop, "sensor-writer-$sensorType").apply {
        start()
    }

    init {
        writeHeader()
    }

    fun offer(timestampNs: Long, values: FloatArray, accuracy: Int): Boolean {
        if (!accepting.get()) return false
        val record = Record(
            timestampNs = timestampNs,
            x = values.getOrElse(0) { Float.NaN },
            y = values.getOrElse(1) { Float.NaN },
            z = values.getOrElse(2) { Float.NaN },
            biasX = values.getOrElse(3) { Float.NaN },
            biasY = values.getOrElse(4) { Float.NaN },
            biasZ = values.getOrElse(5) { Float.NaN },
            accuracy = accuracy
        )
        val accepted = queue.offer(record)
        if (!accepted) dropped.incrementAndGet()
        return accepted
    }

    fun writtenCount(): Long = written.get()

    fun droppedCount(): Long = dropped.get()

    private fun writeLoop() {
        try {
            while (accepting.get() || queue.isNotEmpty()) {
                val record = queue.poll(POLL_TIMEOUT_MS, TimeUnit.MILLISECONDS)
                if (record != null) {
                    writeRecord(record)
                    written.incrementAndGet()
                }
            }
            output.flush()
        } catch (error: InterruptedException) {
            Thread.currentThread().interrupt()
            reportError("センサーログの書き込み処理が中断されました", error)
        } catch (error: Throwable) {
            reportError("センサーログの書き込みに失敗しました", error)
        } finally {
            runCatching {
                output.flush()
                output.close()
            }.onFailure {
                reportError("センサーログを閉じられませんでした", it)
            }
            closed.set(true)
        }
    }

    private fun writeHeader() {
        val header = ByteBuffer.allocate(HEADER_SIZE)
            .order(ByteOrder.LITTLE_ENDIAN)
            .put(MAGIC)
            .putInt(FORMAT_VERSION)
            .putInt(RECORD_SIZE)
            .putInt(sensorType)
            .array()
        output.write(header)
        output.flush()
    }

    private fun writeRecord(record: Record) {
        val bytes = ByteBuffer.allocate(RECORD_SIZE)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putLong(record.timestampNs)
            .putFloat(record.x)
            .putFloat(record.y)
            .putFloat(record.z)
            .putFloat(record.biasX)
            .putFloat(record.biasY)
            .putFloat(record.biasZ)
            .putInt(record.accuracy)
            .array()
        output.write(bytes)
    }

    private fun reportError(message: String, error: Throwable?) {
        accepting.set(false)
        onError(message, error)
    }

    override fun close() {
        if (!accepting.getAndSet(false) && closed.get()) return
        if (Thread.currentThread() === writerThread) return

        writerThread.join(CLOSE_TIMEOUT_MS)
        if (writerThread.isAlive) {
            writerThread.interrupt()
            onError("センサーログの終了処理が完了しませんでした", null)
        }
    }

    private data class Record(
        val timestampNs: Long,
        val x: Float,
        val y: Float,
        val z: Float,
        val biasX: Float,
        val biasY: Float,
        val biasZ: Float,
        val accuracy: Int
    )

    companion object {
        private val MAGIC = byteArrayOf(
            'G'.code.toByte(), 'I'.code.toByte(),
            'L'.code.toByte(), 'S'.code.toByte()
        )
        private const val FORMAT_VERSION = 1
        private const val HEADER_SIZE = 16
        private const val RECORD_SIZE = 36
        private const val MAX_QUEUE_SIZE = 4096
        private const val OUTPUT_BUFFER_SIZE = 64 * 1024
        private const val POLL_TIMEOUT_MS = 20L
        private const val CLOSE_TIMEOUT_MS = 5_000L
    }
}
