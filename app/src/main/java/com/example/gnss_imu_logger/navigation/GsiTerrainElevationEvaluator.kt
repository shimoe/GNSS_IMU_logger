package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.Locale
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * GNSS軌跡上の気圧高度・GNSS高度・地理院DEM標高を比較し、
 * 基準面の定常差を除いた鉛直基準候補を生成する。
 */
class GsiTerrainElevationEvaluator(private val sessionDirectory: File) {
    fun evaluate(barometric: BarometricAltitudeResult): TerrainVerticalReferenceResult {
        val repository = GsiDemElevationRepository(
            sessionDirectory.resolve("navigation_gsi_dem_cache")
        )
        val gnss = readGnss(sessionDirectory.resolve("gnss.csv"))
        val barometricTimes = barometric.records.map { it.elapsedRealtimeNs }
        val records = gnss.map { point ->
            val barometer = nearestBarometer(point.elapsedRealtimeNs, barometric, barometricTimes)
            val dem = repository.elevation(point.latitudeDeg, point.longitudeDeg)
            val reason = when {
                !dem.valid -> dem.rejectionReason
                point.horizontalAccuracyM > MAXIMUM_HORIZONTAL_ACCURACY_M ->
                    "HORIZONTAL_ACCURACY"
                barometer == null -> "BAROMETER_UNAVAILABLE"
                else -> ""
            }
            TerrainEvaluationRecord(point, barometer, dem, reason)
        }
        val valid = records.filter { it.rejectionReason.isEmpty() }
        val gnssOffset = robustOffset(valid.mapNotNull {
            difference(it.gnss.altitudeM, it.dem.elevationMslM)
        })
        val barometerOffset = robustOffset(valid.mapNotNull {
            difference(it.barometer?.calibratedAltitudeM, it.dem.elevationMslM)
        })
        val result = TerrainVerticalReferenceResult(
            validSampleCount = valid.size,
            totalSampleCount = records.size,
            gnssToMslOffsetM = gnssOffset.offsetM,
            gnssCenteredRmseM = gnssOffset.centeredRmseM,
            gnssInlierCount = gnssOffset.inlierCount,
            barometerToMslOffsetM = barometerOffset.offsetM,
            barometerCenteredRmseM = barometerOffset.centeredRmseM,
            barometerInlierCount = barometerOffset.inlierCount,
            recommendedAltitudeSource = if (
                barometerOffset.centeredRmseM < gnssOffset.centeredRmseM
            ) "BAROMETER_TERRAIN_ALIGNED" else "GNSS_TERRAIN_ALIGNED"
        )
        writeRecords(records, result)
        TerrainVerticalReferenceWriter(sessionDirectory).write(result)
        return result
    }

    private fun writeRecords(
        records: List<TerrainEvaluationRecord>,
        reference: TerrainVerticalReferenceResult
    ) {
        sessionDirectory.resolve(FILE_NAME).bufferedWriter(Charsets.UTF_8).use { writer ->
            writer.appendLine(
                "elapsed_realtime_ns,latitude_deg,longitude_deg,horizontal_accuracy_m," +
                    "gnss_altitude_m,gnss_vertical_accuracy_m,barometric_altitude_raw_m," +
                    "barometric_altitude_30s_m,barometric_altitude_60s_m," +
                    "barometric_altitude_120s_m,gsi_elevation_msl_m,gsi_dem_source," +
                    "gsi_observation_std_m,gsi_zoom,gsi_tile_x,gsi_tile_y," +
                    "gsi_pixel_x,gsi_pixel_y,gnss_minus_gsi_m," +
                    "barometer_raw_minus_gsi_m,barometer_30s_minus_gsi_m," +
                    "barometer_60s_minus_gsi_m,barometer_120s_minus_gsi_m," +
                    "gnss_terrain_aligned_m,barometer_terrain_aligned_m," +
                    "gnss_aligned_minus_gsi_m,barometer_aligned_minus_gsi_m," +
                    "valid,rejection_reason,attribution"
            )
            records.forEach { value ->
                val point = value.gnss
                val barometer = value.barometer
                val dem = value.dem
                val gnssAligned = point.altitudeM - reference.gnssToMslOffsetM
                val barometerAligned = barometer?.calibratedAltitudeM?.minus(
                    reference.barometerToMslOffsetM
                )
                writer.appendLine(listOf(
                    point.elapsedRealtimeNs,
                    point.latitudeDeg.format(),
                    point.longitudeDeg.format(),
                    point.horizontalAccuracyM.format(),
                    point.altitudeM.format(),
                    point.verticalAccuracyM?.format().orEmpty(),
                    barometer?.calibratedAltitudeM?.format().orEmpty(),
                    barometer?.correctedAltitude30sM?.format().orEmpty(),
                    barometer?.correctedAltitude60sM?.format().orEmpty(),
                    barometer?.correctedAltitude120sM?.format().orEmpty(),
                    dem.elevationMslM?.format().orEmpty(),
                    dem.source,
                    dem.observationStdM?.format().orEmpty(),
                    dem.zoom ?: "",
                    dem.tileX ?: "",
                    dem.tileY ?: "",
                    dem.pixelX ?: "",
                    dem.pixelY ?: "",
                    difference(point.altitudeM, dem.elevationMslM).formatOrEmpty(),
                    difference(barometer?.calibratedAltitudeM, dem.elevationMslM).formatOrEmpty(),
                    difference(barometer?.correctedAltitude30sM, dem.elevationMslM).formatOrEmpty(),
                    difference(barometer?.correctedAltitude60sM, dem.elevationMslM).formatOrEmpty(),
                    difference(barometer?.correctedAltitude120sM, dem.elevationMslM).formatOrEmpty(),
                    gnssAligned.format(),
                    barometerAligned?.format().orEmpty(),
                    difference(gnssAligned, dem.elevationMslM).formatOrEmpty(),
                    difference(barometerAligned, dem.elevationMslM).formatOrEmpty(),
                    value.rejectionReason.isEmpty(),
                    value.rejectionReason,
                    ATTRIBUTION
                ).joinToString(","))
            }
        }
    }

    private fun robustOffset(values: List<Double>): RobustOffset {
        require(values.size >= MINIMUM_REFERENCE_SAMPLES) {
            "鉛直基準差の計算に必要な地理院DEMサンプルが不足しています: ${values.size}"
        }
        val median = values.median()
        val mad = values.map { abs(it - median) }.median()
        val threshold = max(MINIMUM_INLIER_THRESHOLD_M, MAD_SCALE * mad)
        val inliers = values.filter { abs(it - median) <= threshold }
        val offset = inliers.median()
        val rmse = sqrt(inliers.map { (it - offset) * (it - offset) }.average())
        return RobustOffset(offset, rmse, inliers.size)
    }

    private fun nearestBarometer(
        elapsedRealtimeNs: Long,
        result: BarometricAltitudeResult,
        times: List<Long>
    ): BarometricAltitudeRecord? {
        val index = times.binarySearch(elapsedRealtimeNs).let { found ->
            if (found >= 0) found else (-found - 1).coerceIn(0, times.lastIndex)
        }
        val candidates = listOf(index, index - 1).filter { it in times.indices }
        val nearest = candidates.minByOrNull { abs(times[it] - elapsedRealtimeNs) } ?: return null
        return result.records[nearest].takeIf {
            abs(it.elapsedRealtimeNs - elapsedRealtimeNs) <= MAXIMUM_BAROMETER_AGE_NS
        }
    }

    private fun readGnss(file: File): List<GsiGnssPoint> {
        val lines = file.readLines(Charsets.UTF_8)
        if (lines.isEmpty()) return emptyList()
        val header = lines.first().split(',').map { it.trim().trim('"') }
        fun index(name: String) = header.indexOf(name)
        val time = index("elapsed_realtime_ns")
        val latitude = index("latitude_deg")
        val longitude = index("longitude_deg")
        val altitude = index("altitude_m")
        val horizontalAccuracy = index("horizontal_accuracy_m")
        val verticalAccuracy = index("vertical_accuracy_m")
        require(listOf(time, latitude, longitude, altitude, horizontalAccuracy).all { it >= 0 }) {
            "gnss.csvに地理院DEM評価に必要な列がありません。"
        }
        return lines.drop(1).mapNotNull { line ->
            val values = line.split(',').map { it.trim().trim('"') }
            GsiGnssPoint(
                elapsedRealtimeNs = values.getOrNull(time)?.toLongOrNull() ?: return@mapNotNull null,
                latitudeDeg = values.getOrNull(latitude)?.toDoubleOrNull() ?: return@mapNotNull null,
                longitudeDeg = values.getOrNull(longitude)?.toDoubleOrNull() ?: return@mapNotNull null,
                altitudeM = values.getOrNull(altitude)?.toDoubleOrNull() ?: return@mapNotNull null,
                horizontalAccuracyM = values.getOrNull(horizontalAccuracy)?.toDoubleOrNull()
                    ?: return@mapNotNull null,
                verticalAccuracyM = values.getOrNull(verticalAccuracy)?.toDoubleOrNull()
            )
        }
    }

    private fun difference(first: Double?, second: Double?): Double? =
        if (first != null && second != null) first - second else null
    private fun Double?.formatOrEmpty(): String = this?.format().orEmpty()
    private fun Double.format(): String =
        if (isFinite()) String.format(Locale.US, "%.9f", this) else ""
    private fun List<Double>.median(): Double {
        val sorted = sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) {
            (sorted[middle - 1] + sorted[middle]) * 0.5
        } else sorted[middle]
    }

    private data class TerrainEvaluationRecord(
        val gnss: GsiGnssPoint,
        val barometer: BarometricAltitudeRecord?,
        val dem: GsiElevationResult,
        val rejectionReason: String
    )
    private data class RobustOffset(
        val offsetM: Double,
        val centeredRmseM: Double,
        val inlierCount: Int
    )
    private data class GsiGnssPoint(
        val elapsedRealtimeNs: Long,
        val latitudeDeg: Double,
        val longitudeDeg: Double,
        val altitudeM: Double,
        val horizontalAccuracyM: Double,
        val verticalAccuracyM: Double?
    )

    companion object {
        private const val FILE_NAME = "navigation_gsi_terrain_elevation.csv"
        private const val MAXIMUM_HORIZONTAL_ACCURACY_M = 15.0
        private const val MAXIMUM_BAROMETER_AGE_NS = 500_000_000L
        private const val MINIMUM_REFERENCE_SAMPLES = 30
        private const val MINIMUM_INLIER_THRESHOLD_M = 10.0
        private const val MAD_SCALE = 4.4478
        private const val ATTRIBUTION = "国土地理院 地理院タイル 標高タイルを加工して作成"
    }
}

data class TerrainVerticalReferenceResult(
    val validSampleCount: Int,
    val totalSampleCount: Int,
    val gnssToMslOffsetM: Double,
    val gnssCenteredRmseM: Double,
    val gnssInlierCount: Int,
    val barometerToMslOffsetM: Double,
    val barometerCenteredRmseM: Double,
    val barometerInlierCount: Int,
    val recommendedAltitudeSource: String
)
