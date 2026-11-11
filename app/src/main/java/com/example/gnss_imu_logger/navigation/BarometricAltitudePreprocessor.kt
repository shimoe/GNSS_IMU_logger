package com.example.gnss_imu_logger.navigation

import java.io.File
import java.util.ArrayDeque
import kotlin.math.abs
import kotlin.math.pow

/**
 * BMP390の気圧ログから相対高度と鉛直速度を生成する。
 * 入力: pressure.bin、gnss.csv
 * 出力: 気圧高度診断レコード
 */
class BarometricAltitudePreprocessor(
    private val parameters: BarometricAltitudeParameters =
        BarometricAltitudeParameters.DEFAULT
) {
    fun prepare(sessionDirectory: File): BarometricAltitudeResult {
        val pressureFile = sessionDirectory.resolve(PRESSURE_FILE_NAME)
        require(pressureFile.isFile) {
            "気圧ログが存在しません: ${pressureFile.path}"
        }
        val gnssAltitude = readGnssAltitude(sessionDirectory.resolve(GNSS_FILE_NAME))
        require(gnssAltitude.isNotEmpty()) {
            "気圧高度の基準に使用できるGNSS高度がありません。"
        }
        val samples = BinarySensorReader(pressureFile).use { reader ->
            reader.toList()
        }
        require(samples.isNotEmpty()) { "pressure.binに気圧サンプルがありません。" }

        val startNs = samples.first().elapsedRealtimeNs
        val calibrationEndNs = startNs + parameters.initialPressureWindowNs
        val initialPressureValues = samples.asSequence()
            .filter { it.elapsedRealtimeNs <= calibrationEndNs }
            .map { it.x }
            .filter { isValidPressure(it) }
            .sorted()
            .toList()
        require(initialPressureValues.size >= parameters.minimumInitialSamples) {
            "初期気圧の計算に必要なサンプルが不足しています: ${initialPressureValues.size}"
        }
        val referencePressureHpa = initialPressureValues.median()
        val referenceGnss = gnssAltitude.minBy {
            abs(it.elapsedRealtimeNs - calibrationEndNs)
        }
        val records = mutableListOf<BarometricAltitudeRecord>()
        val altitudeHistory = ArrayDeque<AltitudePoint>()
        var filteredPressureHpa = referencePressureHpa
        var previousPressureHpa: Double? = null
        var previousTimestampNs: Long? = null
        var gnssIndex = 0
        val bias30 = AltitudeBiasFilter(30.0)
        val bias60 = AltitudeBiasFilter(60.0)
        val bias120 = AltitudeBiasFilter(120.0)
        var lastBiasGnssNs: Long? = null

        samples.forEach { sample ->
            val pressureHpa = sample.x
            val dtSec = previousTimestampNs?.let {
                (sample.elapsedRealtimeNs - it) / NS_PER_SEC
            }
            val rejectionReason = when {
                !isValidPressure(pressureHpa) -> "PRESSURE_RANGE"
                previousPressureHpa != null &&
                    abs(pressureHpa - previousPressureHpa!!) > parameters.maximumPressureJumpHpa ->
                    "PRESSURE_JUMP"
                dtSec != null && (dtSec <= 0.0 || dtSec > parameters.maximumIntervalSec) ->
                    "TIMESTAMP_INTERVAL"
                else -> null
            }
            if (rejectionReason == null) {
                val alpha = if (dtSec == null) 1.0 else {
                    dtSec / (parameters.lowPassTimeConstantSec + dtSec)
                }
                filteredPressureHpa += alpha * (pressureHpa - filteredPressureHpa)
            }
            val relativeAltitudeM = pressureToRelativeAltitude(
                referencePressureHpa,
                filteredPressureHpa
            )
            altitudeHistory.addLast(
                AltitudePoint(sample.elapsedRealtimeNs, relativeAltitudeM)
            )
            while (
                altitudeHistory.size > 2 &&
                sample.elapsedRealtimeNs - altitudeHistory.first().elapsedRealtimeNs >
                parameters.verticalSpeedWindowNs + parameters.verticalSpeedHistoryMarginNs
            ) {
                altitudeHistory.removeFirst()
            }
            val verticalSpeedMps = verticalSpeed(
                sample.elapsedRealtimeNs,
                relativeAltitudeM,
                altitudeHistory
            )
            while (
                gnssIndex + 1 < gnssAltitude.size &&
                abs(gnssAltitude[gnssIndex + 1].elapsedRealtimeNs - sample.elapsedRealtimeNs) <=
                abs(gnssAltitude[gnssIndex].elapsedRealtimeNs - sample.elapsedRealtimeNs)
            ) {
                gnssIndex++
            }
            val nearestGnss = gnssAltitude[gnssIndex].takeIf {
                abs(it.elapsedRealtimeNs - sample.elapsedRealtimeNs) <= parameters.maximumGnssAgeNs
            }
            val speedAccepted = verticalSpeedMps == null ||
                abs(verticalSpeedMps) <= parameters.maximumAbsoluteVerticalSpeedMps
            val finalReason = rejectionReason ?: if (!speedAccepted) "VERTICAL_SPEED" else null
            val calibratedAltitudeM = referenceGnss.altitudeM + relativeAltitudeM
            val rawResidualM = nearestGnss?.let { calibratedAltitudeM - it.altitudeM }
            val biasRejectionReason = when {
                nearestGnss == null -> "GNSS_UNAVAILABLE"
                nearestGnss.verticalAccuracyM == null -> "VERTICAL_ACCURACY_UNAVAILABLE"
                nearestGnss.verticalAccuracyM > parameters.maximumGnssVerticalAccuracyM ->
                    "VERTICAL_ACCURACY"
                finalReason != null -> "BAROMETER_INVALID"
                nearestGnss.elapsedRealtimeNs == lastBiasGnssNs -> "GNSS_ALREADY_USED"
                else -> null
            }
            val biasUpdateApplied = biasRejectionReason == null && rawResidualM != null
            if (biasUpdateApplied) {
                bias30.update(nearestGnss!!.elapsedRealtimeNs, rawResidualM!!)
                bias60.update(nearestGnss.elapsedRealtimeNs, rawResidualM)
                bias120.update(nearestGnss.elapsedRealtimeNs, rawResidualM)
                lastBiasGnssNs = nearestGnss.elapsedRealtimeNs
            }
            records += BarometricAltitudeRecord(
                elapsedRealtimeNs = sample.elapsedRealtimeNs,
                pressureHpa = pressureHpa,
                filteredPressureHpa = filteredPressureHpa,
                relativeAltitudeM = relativeAltitudeM,
                calibratedAltitudeM = calibratedAltitudeM,
                verticalSpeedMps = verticalSpeedMps?.takeIf { speedAccepted },
                referencePressureHpa = referencePressureHpa,
                referenceGnssAltitudeM = referenceGnss.altitudeM,
                gnssAltitudeM = nearestGnss?.altitudeM,
                gnssVerticalAccuracyM = nearestGnss?.verticalAccuracyM,
                rawAltitudeResidualM = rawResidualM,
                barometerBias30sM = bias30.valueM,
                barometerBias60sM = bias60.valueM,
                barometerBias120sM = bias120.valueM,
                correctedAltitude30sM = calibratedAltitudeM - bias30.valueM,
                correctedAltitude60sM = calibratedAltitudeM - bias60.valueM,
                correctedAltitude120sM = calibratedAltitudeM - bias120.valueM,
                gnssBiasUpdateApplied = biasUpdateApplied,
                gnssBiasRejectionReason = biasRejectionReason.orEmpty(),
                valid = finalReason == null,
                rejectionReason = finalReason.orEmpty()
            )
            if (pressureHpa.isFinite()) previousPressureHpa = pressureHpa
            previousTimestampNs = sample.elapsedRealtimeNs
        }
        return BarometricAltitudeResult(
            records = records,
            referencePressureHpa = referencePressureHpa,
            referenceGnssAltitudeM = referenceGnss.altitudeM
        )
    }

    private fun verticalSpeed(
        elapsedRealtimeNs: Long,
        relativeAltitudeM: Double,
        history: ArrayDeque<AltitudePoint>
    ): Double? {
        val targetNs = elapsedRealtimeNs - parameters.verticalSpeedWindowNs
        val base = history.lastOrNull { it.elapsedRealtimeNs <= targetNs } ?: return null
        val dtSec = (elapsedRealtimeNs - base.elapsedRealtimeNs) / NS_PER_SEC
        return if (dtSec > 0.0) (relativeAltitudeM - base.altitudeM) / dtSec else null
    }

    private fun pressureToRelativeAltitude(
        referencePressureHpa: Double,
        pressureHpa: Double
    ): Double = STANDARD_ATMOSPHERE_HEIGHT_M *
        (1.0 - (pressureHpa / referencePressureHpa).pow(PRESSURE_EXPONENT))

    private fun isValidPressure(value: Double): Boolean =
        value.isFinite() && value in parameters.minimumPressureHpa..parameters.maximumPressureHpa

    private fun readGnssAltitude(file: File): List<GnssAltitudePoint> {
        require(file.isFile) { "gnss.csvが存在しません: ${file.path}" }
        val lines = file.readLines(Charsets.UTF_8)
        require(lines.isNotEmpty()) { "gnss.csvが空です。" }
        val header = lines.first().split(',').map { it.trim().trim('"') }
        val timeIndex = header.indexOf("elapsed_realtime_ns")
        val altitudeIndex = header.indexOf("altitude_m")
        val accuracyIndex = header.indexOf("vertical_accuracy_m")
        require(timeIndex >= 0 && altitudeIndex >= 0) {
            "gnss.csvに時刻または高度列がありません。"
        }
        return lines.drop(1).mapNotNull { line ->
            val values = line.split(',').map { it.trim().trim('"') }
            val time = values.getOrNull(timeIndex)?.toLongOrNull() ?: return@mapNotNull null
            val altitude = values.getOrNull(altitudeIndex)?.toDoubleOrNull()
                ?.takeIf { it.isFinite() } ?: return@mapNotNull null
            GnssAltitudePoint(
                elapsedRealtimeNs = time,
                altitudeM = altitude,
                verticalAccuracyM = values.getOrNull(accuracyIndex)?.toDoubleOrNull()
            )
        }
    }

    private fun List<Double>.median(): Double {
        val middle = size / 2
        return if (size % 2 == 0) (this[middle - 1] + this[middle]) * 0.5 else this[middle]
    }

    private class AltitudeBiasFilter(private val timeConstantSec: Double) {
        var valueM: Double = 0.0
            private set
        private var previousElapsedRealtimeNs: Long? = null

        fun update(elapsedRealtimeNs: Long, observedBiasM: Double) {
            val previous = previousElapsedRealtimeNs
            val alpha = if (previous == null) 1.0 else {
                val dtSec = (elapsedRealtimeNs - previous) / NS_PER_SEC
                (dtSec / (timeConstantSec + dtSec)).coerceIn(0.0, 1.0)
            }
            valueM += alpha * (observedBiasM - valueM)
            previousElapsedRealtimeNs = elapsedRealtimeNs
        }
    }

    private data class AltitudePoint(val elapsedRealtimeNs: Long, val altitudeM: Double)
    private data class GnssAltitudePoint(
        val elapsedRealtimeNs: Long,
        val altitudeM: Double,
        val verticalAccuracyM: Double?
    )

    companion object {
        private const val PRESSURE_FILE_NAME = "pressure.bin"
        private const val GNSS_FILE_NAME = "gnss.csv"
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val STANDARD_ATMOSPHERE_HEIGHT_M = 44330.0
        private const val PRESSURE_EXPONENT = 0.190294957
    }
}

data class BarometricAltitudeParameters(
    val initialPressureWindowNs: Long,
    val minimumInitialSamples: Int,
    val lowPassTimeConstantSec: Double,
    val verticalSpeedWindowNs: Long,
    val verticalSpeedHistoryMarginNs: Long,
    val minimumPressureHpa: Double,
    val maximumPressureHpa: Double,
    val maximumPressureJumpHpa: Double,
    val maximumIntervalSec: Double,
    val maximumAbsoluteVerticalSpeedMps: Double,
    val maximumGnssAgeNs: Long,
    val maximumGnssVerticalAccuracyM: Double
) {
    companion object {
        val DEFAULT = BarometricAltitudeParameters(
            initialPressureWindowNs = 5_000_000_000L,
            minimumInitialSamples = 50,
            lowPassTimeConstantSec = 1.0,
            verticalSpeedWindowNs = 2_000_000_000L,
            verticalSpeedHistoryMarginNs = 1_000_000_000L,
            minimumPressureHpa = 300.0,
            maximumPressureHpa = 1100.0,
            maximumPressureJumpHpa = 0.5,
            maximumIntervalSec = 1.0,
            maximumAbsoluteVerticalSpeedMps = 15.0,
            maximumGnssAgeNs = 1_500_000_000L,
            maximumGnssVerticalAccuracyM = 15.0
        )
    }
}

data class BarometricAltitudeRecord(
    val elapsedRealtimeNs: Long,
    val pressureHpa: Double,
    val filteredPressureHpa: Double,
    val relativeAltitudeM: Double,
    val calibratedAltitudeM: Double,
    val verticalSpeedMps: Double?,
    val referencePressureHpa: Double,
    val referenceGnssAltitudeM: Double,
    val gnssAltitudeM: Double?,
    val gnssVerticalAccuracyM: Double?,
    val rawAltitudeResidualM: Double?,
    val barometerBias30sM: Double,
    val barometerBias60sM: Double,
    val barometerBias120sM: Double,
    val correctedAltitude30sM: Double,
    val correctedAltitude60sM: Double,
    val correctedAltitude120sM: Double,
    val gnssBiasUpdateApplied: Boolean,
    val gnssBiasRejectionReason: String,
    val valid: Boolean,
    val rejectionReason: String
)

data class BarometricAltitudeResult(
    val records: List<BarometricAltitudeRecord>,
    val referencePressureHpa: Double,
    val referenceGnssAltitudeM: Double
)
