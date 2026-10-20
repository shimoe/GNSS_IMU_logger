package com.example.gnss_imu_logger.navigation

import java.io.BufferedInputStream
import java.io.Closeable
import java.io.DataInputStream
import java.io.EOFException
import java.io.File
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * BinarySensorWriter形式の固定長センサーログを逐次読み込む。
 * 入力: accel.bin、gyro.bin、gyro_corrected.bin
 * 出力: elapsedRealtimeNs[ns]順の6要素センサーサンプル
 */
class BinarySensorReader(file: File) : Closeable, Iterable<BinarySensorSample> {
    private val input = DataInputStream(BufferedInputStream(FileInputStream(file), BUFFER_SIZE))
    val header: BinarySensorHeader = readHeader()
    private var iteratorCreated = false

    override fun iterator(): Iterator<BinarySensorSample> {
        check(!iteratorCreated) { "同じセンサーログを複数回読み込むことはできません。" }
        iteratorCreated = true
        return BinarySensorIterator(input, header.recordSize)
    }

    override fun close() = input.close()

    private fun readHeader(): BinarySensorHeader {
        val bytes = ByteArray(HEADER_SIZE)
        input.readFully(bytes)
        require(bytes.copyOfRange(0, MAGIC.size).contentEquals(MAGIC)) {
            "センサーログの識別子が不正です。"
        }
        val buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
        buffer.position(MAGIC.size)
        val version = buffer.int
        val recordSize = buffer.int
        val sensorType = buffer.int
        require(version == SUPPORTED_FORMAT_VERSION) {
            "未対応のセンサーログ形式です: version=$version"
        }
        require(recordSize == RECORD_SIZE) {
            "センサーログのレコード長が不正です: recordSize=$recordSize"
        }
        return BinarySensorHeader(version, recordSize, sensorType)
    }

    private class BinarySensorIterator(
        private val input: DataInputStream,
        private val recordSize: Int
    ) : Iterator<BinarySensorSample> {
        private var nextValue: BinarySensorSample? = readNext()

        override fun hasNext(): Boolean = nextValue != null

        override fun next(): BinarySensorSample {
            val value = nextValue ?: throw NoSuchElementException("センサーログの終端です。")
            nextValue = readNext()
            return value
        }

        private fun readNext(): BinarySensorSample? {
            val bytes = ByteArray(recordSize)
            return try {
                input.readFully(bytes)
                val buffer = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
                BinarySensorSample(
                    elapsedRealtimeNs = buffer.long,
                    x = buffer.float.toDouble(),
                    y = buffer.float.toDouble(),
                    z = buffer.float.toDouble(),
                    biasX = buffer.float.toDouble(),
                    biasY = buffer.float.toDouble(),
                    biasZ = buffer.float.toDouble(),
                    accuracy = buffer.int
                )
            } catch (_: EOFException) {
                null
            }
        }
    }

    companion object {
        private val MAGIC = byteArrayOf(
            'G'.code.toByte(), 'I'.code.toByte(),
            'L'.code.toByte(), 'S'.code.toByte()
        )
        private const val SUPPORTED_FORMAT_VERSION = 1
        private const val HEADER_SIZE = 16
        private const val RECORD_SIZE = 36
        private const val BUFFER_SIZE = 64 * 1024
    }
}

data class BinarySensorHeader(
    val formatVersion: Int,
    val recordSize: Int,
    val sensorType: Int
)

data class BinarySensorSample(
    val elapsedRealtimeNs: Long,
    val x: Double,
    val y: Double,
    val z: Double,
    val biasX: Double,
    val biasY: Double,
    val biasZ: Double,
    val accuracy: Int
) {
    val correctedX: Double get() = x - biasX.takeIf { it.isFinite() }.orZero()
    val correctedY: Double get() = y - biasY.takeIf { it.isFinite() }.orZero()
    val correctedZ: Double get() = z - biasZ.takeIf { it.isFinite() }.orZero()
}

private fun Double?.orZero(): Double = this ?: 0.0
