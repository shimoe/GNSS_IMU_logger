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
        clearPreviousOutputs(sessionDirectory)
        val status = NavigationAnalysisStatusWriter(sessionDirectory)
        status.start()
        var stage = NavigationAnalysisStage.INPUT
        return try {
            status.update(stage)
            val input = OfflineNavigationInputReader(sessionDirectory).read()
            val prepared = NavigationInputPreprocessor().prepare(input)
            writeInputMetadata(sessionDirectory, prepared)

            stage = NavigationAnalysisStage.BAROMETRIC_ALTITUDE
            status.update(stage)
            val barometricAltitude = BarometricAltitudePreprocessor().prepare(sessionDirectory)
            BarometricAltitudeWriter(sessionDirectory).write(barometricAltitude)

            stage = NavigationAnalysisStage.GSI_TERRAIN
            status.update(stage)
            val terrainReference = runCatching {
                GsiTerrainElevationEvaluator(sessionDirectory).evaluate(barometricAltitude)
            }.onFailure { error ->
                sessionDirectory.resolve("navigation_gsi_terrain_error.txt")
                    .writeText("地理院DEM評価に失敗しました: ${error.message}", Charsets.UTF_8)
            }.getOrNull()
            val verticalObservations = NavigationVerticalObservationFactory().create(
                barometric = barometricAltitude,
                reference = terrainReference
            )

            stage = NavigationAnalysisStage.NAVIGATION
            status.update(stage)
            val result = NavigationAttitudePredictionWriter(sessionDirectory).use { attitudeWriter ->
                NavigationVerticalUpdateWriter(sessionDirectory).use { verticalWriter ->
                    GnssAidedOfflineNavigationPredictor(
                        attitudePredictionSink = attitudeWriter,
                        verticalObservations = verticalObservations,
                        verticalUpdateWriter = verticalWriter
                    ).run(prepared)
                }
            }
            NavigationInitializationWriter(sessionDirectory).write(result.initialization)
            val summary = OfflineNavigationPredictionSummarizer().summarize(result.prediction)

            stage = NavigationAnalysisStage.OUTPUT
            status.update(stage)
            OfflineNavigationPredictionWriter(sessionDirectory).write(result.prediction, summary)
            NavigationMeasurementUpdateWriter(sessionDirectory).write(result.measurementUpdates)
            NavigationGravityUpdateWriter(sessionDirectory).write(result.gravityUpdates)
            NavigationHeadingDirectionWriter(sessionDirectory).write(result.headingDirections)
            val processResult = OfflineNavigationProcessResult(
                deviceMount = prepared.deviceMount.name,
                predictionSampleCount = result.prediction.samples.size,
                gnssResidualCount = result.prediction.gnssResiduals.size,
                measurementUpdateCount = result.measurementUpdates.size,
                gravityUpdateCount = result.gravityUpdates.size,
                attitudePredictionCount = result.attitudePredictionCount,
                assessment = summary.assessment
            )
            status.complete()
            processResult
        } catch (error: Throwable) {
            status.fail(stage, error)
            throw error
        }
    }

    /** 前回の解析結果を削除し、異なる実装世代のCSVが混在しないようにする。 */
    private fun clearPreviousOutputs(sessionDirectory: File) {
        OUTPUT_FILE_NAMES.forEach { name -> sessionDirectory.resolve(name).delete() }
        sessionDirectory.resolve("navigation_gsi_dem_cache")
            .takeIf { it.isDirectory }
            ?.deleteRecursively()
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

    companion object {
        private val OUTPUT_FILE_NAMES = listOf(
            "navigation_analysis_status.csv",
            "navigation_attitude_prediction.csv",
            "navigation_barometric_altitude.csv",
            "navigation_gnss_residual.csv",
            "navigation_gravity_update.csv",
            "navigation_gsi_terrain_elevation.csv",
            "navigation_gsi_terrain_error.txt",
            "navigation_heading_direction.csv",
            "navigation_initialization.csv",
            "navigation_input_metadata.csv",
            "navigation_measurement_update.csv",
            "navigation_prediction.csv",
            "navigation_prediction_summary.csv",
            "navigation_vertical_reference.csv",
            "navigation_vertical_update.csv"
        )
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
