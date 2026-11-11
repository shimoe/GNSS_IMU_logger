package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.DeviceMount
import org.json.JSONObject
import java.io.File
import java.util.PriorityQueue

/**
 * 保存セッションからIMUとGNSSを読み込み、時刻順の推定入力へ変換する。
 * 入力: セッションディレクトリ
 * 出力: elapsedRealtimeNs[ns]順の加速度、角速度、GNSS観測イベント
 */
class OfflineNavigationInputReader(
    private val sessionDirectory: File,
    private val gate: GnssObservationGate = GnssObservationGate()
) {
    /**
     * セッション原点と時刻順イベントを読み込む。
     * 入力: accel.bin、gyro_corrected.binまたはgyro.bin、gnss.csv
     * 出力: OfflineNavigationInput
     */
    fun read(): OfflineNavigationInput {
        require(sessionDirectory.isDirectory) {
            "セッションディレクトリが存在しません: ${sessionDirectory.path}"
        }
        val sessionFile = requiredFile("session.json")
        val sessionJson = readSessionJson(sessionFile)
        val deviceMount = readDeviceMount(sessionJson)
        val calibration = readSessionCalibration(sessionJson)
        val accelerometerFile = requiredFile("accel.bin")
        val angularRateInput = selectAngularRateInput()
        val observations = readGnssCsv(requiredFile("gnss.csv"))
        require(observations.isNotEmpty()) { "gnss.csvに有効な位置観測がありません。" }
        val originObservation = observations.firstOrNull { gate.evaluate(it).accepted }
            ?: error("原点に使用できるGNSS観測がありません。")
        return OfflineNavigationInput(
            origin = originObservation.position,
            deviceMount = deviceMount,
            calibration = calibration,
            angularRateSource = angularRateInput.source,
            events = mergeEvents(accelerometerFile, angularRateInput.file, observations)
        )
    }

    /**
     * 計測時に保存された端末取付方向を読み込む。
     * 入力: session.json
     * 出力: セッションで使用したDeviceMount
     */
    /** session.jsonをJSONとして読み込む。 */
    private fun readSessionJson(file: File): JSONObject =
        runCatching { JSONObject(file.readText(Charsets.UTF_8)) }
            .getOrElse { error("session.jsonを読み込めません: ${it.message}") }

    /**
     * 計測時に保存された端末取付方向を読み込む。
     * 入力: session.jsonのJSON
     * 出力: セッションで使用したDeviceMount
     */
    private fun readDeviceMount(json: JSONObject): DeviceMount {
        val formatVersion = json.optInt("format_version", -1)
        require(formatVersion >= DEVICE_MOUNT_FORMAT_VERSION) {
            "端末取付方向を確認できないセッション形式です: format_version=$formatVersion"
        }
        val storedName = json.optString("device_mount", "")
        return DeviceMount.fromStoredName(storedName)
            ?: error("session.jsonの端末取付方向が不正です: device_mount=$storedName")
    }

    /**
     * 計測時の初期校正結果を読み込む。
     * 入力: session.jsonのJSON
     * 出力: 校正結果。旧形式などで存在しない場合はnull
     */
    private fun readSessionCalibration(json: JSONObject): SessionCalibration? {
        val calibration = json.optJSONObject("statistics")
            ?.optJSONObject("calibration") ?: return null
        val gravity = calibration.optJSONObject("gravity_mps2") ?: return null
        val gravityStd = calibration.optJSONObject("gravity_std_mps2") ?: return null
        val gyroscopeBias = calibration.optJSONObject("gyroscope_bias_radps") ?: return null
        val gyroscopeStd = calibration.optJSONObject("gyroscope_std_radps") ?: return null
        return SessionCalibration(
            startedElapsedRealtimeNs = calibration.optLong("started_elapsed_ns", -1L),
            completedElapsedRealtimeNs = calibration.optLong("completed_elapsed_ns", -1L),
            accelerationSampleCount = calibration.optInt("accelerometer_samples", 0),
            angularRateSampleCount = calibration.optInt("gyroscope_samples", 0),
            gravityDeviceMps2 = gravity.toBodyVector(),
            gravityStdDeviceMps2 = gravityStd.toBodyVector(),
            gyroscopeBiasDeviceRadps = gyroscopeBias.toBodyVector(),
            gyroscopeStdDeviceRadps = gyroscopeStd.toBodyVector()
        )
    }

    /** 補正済み角速度を優先し、入力元を明示する。 */
    private fun selectAngularRateInput(): AngularRateInput {
        val corrected = sessionDirectory.resolve("gyro_corrected.bin")
        if (corrected.isFile && corrected.length() > BINARY_SENSOR_HEADER_BYTES) {
            return AngularRateInput(corrected, AngularRateSource.CORRECTED)
        }
        return AngularRateInput(
            requiredFile("gyro.bin"),
            AngularRateSource.RAW_WITH_RECORDED_BIAS
        )
    }

    private fun JSONObject.toBodyVector() =
        com.example.gnss_imu_logger.attitude.BodyVector(
            x = optDouble("x", Double.NaN),
            y = optDouble("y", Double.NaN),
            z = optDouble("z", Double.NaN)
        )

    private fun mergeEvents(
        accelerometerFile: File,
        gyroscopeFile: File,
        observations: List<GnssObservation>
    ): Sequence<OfflineNavigationEvent> = sequence {
        BinarySensorReader(accelerometerFile).use { accelerometer ->
            BinarySensorReader(gyroscopeFile).use { gyroscope ->
                val sources = listOfNotNull(
                    EventSource.accelerometer(accelerometer.iterator()),
                    EventSource.gyroscope(gyroscope.iterator()),
                    EventSource.gnss(observations.iterator(), gate)
                )
                val queue = PriorityQueue<EventSource>(
                    compareBy<EventSource> { it.current.elapsedRealtimeNs }
                        .thenBy { it.priority }
                )
                queue.addAll(sources)
                while (queue.isNotEmpty()) {
                    val source = queue.remove()
                    yield(source.current)
                    if (source.advance()) queue.add(source)
                }
            }
        }
    }

    private fun readGnssCsv(file: File): List<GnssObservation> {
        val lines = file.bufferedReader().useLines { it.toList() }
        if (lines.isEmpty()) return emptyList()
        val header = parseCsvLine(lines.first())
        return lines.drop(1).mapNotNull { line ->
            runCatching {
                val values = parseCsvLine(line)
                val row = header.zip(values).toMap()
                GnssObservation(
                    elapsedRealtimeNs = row.requiredLong("elapsed_realtime_ns"),
                    position = GeodeticPosition(
                        latitudeDeg = row.requiredDouble("latitude_deg"),
                        longitudeDeg = row.requiredDouble("longitude_deg"),
                        altitudeM = row.optionalDouble("altitude_m") ?: 0.0
                    ),
                    speedMps = row.optionalDouble("speed_mps"),
                    bearingDeg = row.optionalDouble("bearing_deg"),
                    horizontalAccuracyM = row.optionalDouble("horizontal_accuracy_m"),
                    verticalAccuracyM = row.optionalDouble("vertical_accuracy_m"),
                    speedAccuracyMps = row.optionalDouble("speed_accuracy_mps"),
                    bearingAccuracyDeg = row.optionalDouble("bearing_accuracy_deg"),
                    elapsedRealtimeUncertaintyNs = row.optionalDouble("elapsed_realtime_uncertainty_ns"),
                    totalSatellites = row.optionalInt("total_satellites"),
                    usedSatellites = row.optionalInt("used_satellites"),
                    usedSatelliteMeanCn0DbHz = row.optionalDouble("used_satellite_mean_cn0_dbhz"),
                    statusAgeMs = row.optionalLong("status_age_ms"),
                    mock = row["mock"]?.toBooleanStrictOrNull() ?: false
                )
            }.getOrNull()
        }.sortedBy { it.elapsedRealtimeNs }
    }

    private fun requiredFile(name: String): File = sessionDirectory.resolve(name).also {
        require(it.isFile) { "必要なログファイルがありません: $name" }
    }

    private fun parseCsvLine(line: String): List<String> {
        val values = mutableListOf<String>()
        val value = StringBuilder()
        var quoted = false
        var index = 0
        while (index < line.length) {
            val char = line[index]
            when {
                char == '"' && quoted && index + 1 < line.length && line[index + 1] == '"' -> {
                    value.append('"')
                    index++
                }
                char == '"' -> quoted = !quoted
                char == ',' && !quoted -> {
                    values += value.toString()
                    value.clear()
                }
                else -> value.append(char)
            }
            index++
        }
        values += value.toString()
        return values
    }

    private fun Map<String, String>.requiredLong(name: String): Long =
        optionalLong(name) ?: error("${name}が不正です。")

    private fun Map<String, String>.requiredDouble(name: String): Double =
        optionalDouble(name) ?: error("${name}が不正です。")

    private fun Map<String, String>.optionalLong(name: String): Long? =
        get(name)?.takeIf { it.isNotBlank() }?.toLongOrNull()

    private fun Map<String, String>.optionalInt(name: String): Int? =
        get(name)?.takeIf { it.isNotBlank() }?.toIntOrNull()

    private fun Map<String, String>.optionalDouble(name: String): Double? =
        get(name)?.takeIf { it.isNotBlank() }?.toDoubleOrNull()

    companion object {
        private const val DEVICE_MOUNT_FORMAT_VERSION = 2
        private const val BINARY_SENSOR_HEADER_BYTES = 16L
    }

    private data class AngularRateInput(
        val file: File,
        val source: AngularRateSource
    )

    private class EventSource private constructor(
        val priority: Int,
        private val iterator: Iterator<OfflineNavigationEvent>
    ) {
        var current: OfflineNavigationEvent = iterator.next()
            private set
        fun advance(): Boolean {
            if (!iterator.hasNext()) return false
            current = iterator.next()
            return true
        }

        companion object {
            fun accelerometer(values: Iterator<BinarySensorSample>): EventSource? = create(
                0,
                values.asSequence().map { OfflineNavigationEvent.Acceleration(it) }.iterator()
            )

            fun gyroscope(values: Iterator<BinarySensorSample>): EventSource? = create(
                1,
                values.asSequence().map { OfflineNavigationEvent.AngularRate(it) }.iterator()
            )

            fun gnss(
                values: Iterator<GnssObservation>,
                gate: GnssObservationGate
            ): EventSource? = create(
                2,
                values.asSequence().map {
                    OfflineNavigationEvent.Gnss(it, gate.evaluate(it))
                }.iterator()
            )

            private fun create(
                priority: Int,
                iterator: Iterator<OfflineNavigationEvent>
            ): EventSource? = if (iterator.hasNext()) EventSource(priority, iterator) else null
        }
    }
}

data class OfflineNavigationInput(
    val origin: GeodeticPosition,
    val deviceMount: DeviceMount,
    val calibration: SessionCalibration?,
    val angularRateSource: AngularRateSource,
    val events: Sequence<OfflineNavigationEvent>
)

data class SessionCalibration(
    val startedElapsedRealtimeNs: Long,
    val completedElapsedRealtimeNs: Long,
    val accelerationSampleCount: Int,
    val angularRateSampleCount: Int,
    val gravityDeviceMps2: com.example.gnss_imu_logger.attitude.BodyVector,
    val gravityStdDeviceMps2: com.example.gnss_imu_logger.attitude.BodyVector,
    val gyroscopeBiasDeviceRadps: com.example.gnss_imu_logger.attitude.BodyVector,
    val gyroscopeStdDeviceRadps: com.example.gnss_imu_logger.attitude.BodyVector
)

enum class AngularRateSource {
    CORRECTED,
    RAW_WITH_RECORDED_BIAS
}

sealed interface OfflineNavigationEvent {
    val elapsedRealtimeNs: Long

    data class Acceleration(val sample: BinarySensorSample) : OfflineNavigationEvent {
        override val elapsedRealtimeNs: Long = sample.elapsedRealtimeNs
    }

    data class AngularRate(val sample: BinarySensorSample) : OfflineNavigationEvent {
        override val elapsedRealtimeNs: Long = sample.elapsedRealtimeNs
    }

    data class Gnss(
        val observation: GnssObservation,
        val decision: GnssObservationDecision
    ) : OfflineNavigationEvent {
        override val elapsedRealtimeNs: Long = observation.elapsedRealtimeNs
    }
}
