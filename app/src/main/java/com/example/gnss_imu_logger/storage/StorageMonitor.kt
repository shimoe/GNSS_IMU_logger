package com.example.gnss_imu_logger.storage

import android.os.Handler
import android.os.Looper
import android.os.StatFs
import java.io.Closeable
import java.io.File

/**
 * セッション保存先の空き容量を定期監視する。
 * 入力: 保存先、警告・停止コールバック
 * 出力: 10秒周期の空き容量判定
 */
class StorageMonitor(
    private val directory: File,
    private val onWarning: (Long) -> Unit,
    private val onCritical: (Long) -> Unit
) : Closeable {
    private val handler = Handler(Looper.getMainLooper())
    private var warned = false
    private var running = false
    private val checkTask = object : Runnable {
        override fun run() {
            if (!running) return
            val freeBytes = freeBytes()
            when {
                freeBytes < CRITICAL_FREE_BYTES -> onCritical(freeBytes)
                freeBytes < WARNING_FREE_BYTES && !warned -> {
                    warned = true
                    onWarning(freeBytes)
                }
            }
            handler.postDelayed(this, CHECK_PERIOD_MS)
        }
    }

    fun start() {
        val freeBytes = freeBytes()
        if (freeBytes < START_FREE_BYTES) {
            throw IllegalStateException("保存先の空き容量が500 MiB未満です")
        }
        running = true
        handler.post(checkTask)
    }

    fun freeBytes(): Long = StatFs(directory.absolutePath).availableBytes

    override fun close() {
        running = false
        handler.removeCallbacks(checkTask)
    }

    companion object {
        private const val CHECK_PERIOD_MS = 10_000L
        private const val MIB = 1024L * 1024L
        private const val START_FREE_BYTES = 500L * MIB
        private const val WARNING_FREE_BYTES = 250L * MIB
        private const val CRITICAL_FREE_BYTES = 100L * MIB
    }
}
