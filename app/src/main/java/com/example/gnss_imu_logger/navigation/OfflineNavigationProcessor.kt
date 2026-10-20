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
        val result = GnssAidedOfflineNavigationPredictor().run(prepared)
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
        return OfflineNavigationProcessResult(
            predictionSampleCount = result.prediction.samples.size,
            gnssResidualCount = result.prediction.gnssResiduals.size,
            measurementUpdateCount = result.measurementUpdates.size,
            gravityUpdateCount = result.gravityUpdates.size,
            assessment = summary.assessment
        )
    }
}

data class OfflineNavigationProcessResult(
    val predictionSampleCount: Int,
    val gnssResidualCount: Int,
    val measurementUpdateCount: Int,
    val gravityUpdateCount: Int,
    val assessment: NavigationPredictionAssessment
)
