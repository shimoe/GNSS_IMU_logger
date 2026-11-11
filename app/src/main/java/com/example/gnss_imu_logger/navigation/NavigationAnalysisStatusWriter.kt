package com.example.gnss_imu_logger.navigation

import java.io.File

/**
 * オフライン解析の世代と完了状態を保存する。
 * 入力: 解析状態、処理段階、失敗理由
 * 出力: navigation_analysis_status.csv
 */
class NavigationAnalysisStatusWriter(private val sessionDirectory: File) {
    private val statusFile = sessionDirectory.resolve(FILE_NAME)
    private var startedUtcMs: Long = 0L

    fun start() {
        startedUtcMs = System.currentTimeMillis()
        write(
            result = NavigationAnalysisResult.RUNNING,
            stage = NavigationAnalysisStage.STARTING,
            completedUtcMs = null,
            reason = ""
        )
    }

    fun update(stage: NavigationAnalysisStage) {
        write(
            result = NavigationAnalysisResult.RUNNING,
            stage = stage,
            completedUtcMs = null,
            reason = ""
        )
    }

    fun complete() {
        write(
            result = NavigationAnalysisResult.COMPLETED,
            stage = NavigationAnalysisStage.COMPLETED,
            completedUtcMs = System.currentTimeMillis(),
            reason = ""
        )
    }

    fun fail(stage: NavigationAnalysisStage, error: Throwable) {
        write(
            result = NavigationAnalysisResult.FAILED,
            stage = stage,
            completedUtcMs = System.currentTimeMillis(),
            reason = error.message.orEmpty().replace(',', ' ')
        )
    }

    private fun write(
        result: NavigationAnalysisResult,
        stage: NavigationAnalysisStage,
        completedUtcMs: Long?,
        reason: String
    ) {
        statusFile.bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "analysis_schema_version,implementation_id,started_utc_ms," +
                    "completed_utc_ms,result,stage,failure_reason"
            )
            writer.appendLine(
                listOf(
                    ANALYSIS_SCHEMA_VERSION,
                    IMPLEMENTATION_ID,
                    startedUtcMs,
                    completedUtcMs ?: "",
                    result.name,
                    stage.name,
                    reason
                ).joinToString(",")
            )
        }
    }

    companion object {
        const val ANALYSIS_SCHEMA_VERSION = 3
        const val IMPLEMENTATION_ID = "baro_bias_gsi_v1"
        private const val FILE_NAME = "navigation_analysis_status.csv"
    }
}

enum class NavigationAnalysisResult {
    RUNNING,
    COMPLETED,
    FAILED
}

enum class NavigationAnalysisStage {
    STARTING,
    INPUT,
    BAROMETRIC_ALTITUDE,
    GSI_TERRAIN,
    NAVIGATION,
    OUTPUT,
    COMPLETED
}
