package com.example.gnss_imu_logger.navigation

import java.io.File

/**
 * 保存済みセッションのオフラインナビゲーション解析を一括実行する。
 * 入力: セッションディレクトリ
 * 出力: 予測、残差、サマリー、GNSS更新、重力補正のCSV
 *
 * 再生画面でセッションを開いた時点の実装を使って毎回再計算し、
 * セッションディレクトリ内の既存解析CSVを上書きする。
 */
class OfflineNavigationProcessor {
    fun process(sessionDirectory: File): OfflineNavigationProcessResult {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        val input = OfflineNavigationInputReader(sessionDirectory).read()
        val prepared = NavigationInputPreprocessor().prepare(input)
        writeInputMetadata(sessionDirectory, prepared)
        val result = NavigationAttitudePredictionWriter(sessionDirectory).use { writer ->
            GnssAidedOfflineNavigationPredictor(
                attitudePredictionSink = writer
            ).run(prepared)
        }
        NavigationInitializationWriter(sessionDirectory).write(result.initialization)
        val summary = OfflineNavigationPredictionSummarizer().summarize(
            result.prediction
        )
        OfflineNavigationPredictionWriter(sessionDirectory).write(
            result.prediction,
            summary
        )
        NavigationMeasurementUpdateWriter(sessionDirectory).write(
            result.measurementUpdates
        )
        NavigationGravityUpdateWriter(sessionDirectory).write(
            result.gravityUpdates
        )
        NavigationHeadingDirectionWriter(sessionDirectory).write(
            result.headingDirections
        )
        return OfflineNavigationProcessResult(
            deviceMount = prepared.deviceMount.name,
            predictionSampleCount = result.prediction.samples.size,
            gnssResidualCount = result.prediction.gnssResiduals.size,
            measurementUpdateCount = result.measurementUpdates.size,
            gravityUpdateCount = result.gravityUpdates.size,
            attitudePredictionCount = result.attitudePredictionCount,
            assessment = summary.assessment
        )
    }

    /**
     * オフライン解析へ使用した取付方向を保存する。
     * 入力: セッションディレクトリ、前処理済み入力
     * 出力: navigation_input_metadata.csv
     */
    private fun writeInputMetadata(
        sessionDirectory: File,
        input: PreparedNavigationInput
    ) {
        sessionDirectory.resolve("navigation_input_metadata.csv")
            .bufferedWriter(Charsets.UTF_8)
            .use { writer ->
                writer.appendLine("metric,value")
                writer.appendLine("device_mount,${input.deviceMount.name}")
                writer.appendLine("device_mount_source,session.json")
            }
    }
}

data class OfflineNavigationProcessResult(
    val deviceMount: String,
    val predictionSampleCount: Int,
    val gnssResidualCount: Int,
    val measurementUpdateCount: Int,
    val gravityUpdateCount: Int,
    val attitudePredictionCount: Long,
    val assessment: NavigationPredictionAssessment
)
