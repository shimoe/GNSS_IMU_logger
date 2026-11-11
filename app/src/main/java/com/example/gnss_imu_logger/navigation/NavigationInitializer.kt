package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * 安定したIMU区間からオフライン推定の初期姿勢を求める。
 * 入力: 車体座標へ変換済みのIMUイベントとGNSS初期速度
 * 出力: 初期姿勢、積分開始時刻、初期化診断
 */
class NavigationInitializer(
    private val parameters: NavigationInitializationParameters =
        NavigationInitializationParameters.DEFAULT
) {
    fun initialize(input: PreparedNavigationInput): NavigationInitializationResult {
        input.calibration?.let { calibration ->
            if (isValidCalibration(calibration)) {
                return initializeFromCalibration(input, calibration)
            }
        }
        require(input.angularRateSource == AngularRateSource.CORRECTED) {
            "未補正ジャイロを使用するには有効な初期校正結果が必要です。"
        }
        val searchEndNs = input.initialElapsedRealtimeNs + parameters.searchDurationNs
        val imuEvents = input.events.asSequence()
            .filterIsInstance<PreparedNavigationEvent.Imu>()
            .filter { it.elapsedRealtimeNs in input.initialElapsedRealtimeNs..searchEndNs }
            .toList()
        require(imuEvents.isNotEmpty()) { "初期姿勢の計算に使用できるIMUサンプルがありません。" }

        var startNs = input.initialElapsedRealtimeNs
        var lastRejected: NavigationInitializationDiagnostics? = null
        while (startNs + parameters.windowDurationNs <= searchEndNs) {
            val endNs = startNs + parameters.windowDurationNs
            val window = evaluateWindow(input, imuEvents, startNs, endNs)
            if (window.result == NavigationInitializationResultType.ACCEPTED) {
                return createResult(input, window)
            }
            lastRejected = window
            startNs += parameters.windowStepNs
        }
        val reason = lastRejected?.result ?: NavigationInitializationResultType.INSUFFICIENT_SAMPLES
        error("安定した初期姿勢を計算できません: ${reason.name}")
    }

    /** session.jsonの校正結果から初期状態を生成する。 */
    private fun initializeFromCalibration(
        input: PreparedNavigationInput,
        calibration: SessionCalibration
    ): NavigationInitializationResult {
        val transform = input.deviceMount.transform()
        val gravityBody = transform.deviceToBody(calibration.gravityDeviceMps2)
        val gyroscopeBiasBody = transform.deviceToBody(calibration.gyroscopeBiasDeviceRadps)
        val integrationStartNs = maxOf(
            calibration.completedElapsedRealtimeNs,
            input.initialElapsedRealtimeNs
        )
        val initialGnss = latestGnssAtOrBefore(input, integrationStartNs)
        val yawRad = initialYaw(initialGnss.velocityEnuMps)
        val rollRad = atan2(gravityBody.y, gravityBody.z)
        val pitchRad = atan2(
            -gravityBody.x,
            sqrt(gravityBody.y * gravityBody.y + gravityBody.z * gravityBody.z)
        )
        val attitude = AttitudeQuaternion.fromEulerAngles(rollRad, pitchRad, yawRad)
        val gravityRemoved = attitude.rotate(gravityBody) - GRAVITY_ENU_MPS2
        val gravityRemovedNorm = gravityRemoved.norm()
        require(gravityRemovedNorm <= parameters.maximumGravityRemovedAccelerationMps2) {
            "保存された校正結果の重力除去後加速度が大きすぎます: $gravityRemovedNorm m/s^2"
        }
        val initialBias = if (input.angularRateSource == AngularRateSource.CORRECTED) {
            BodyVector(0.0, 0.0, 0.0)
        } else {
            gyroscopeBiasBody
        }
        val diagnostics = NavigationInitializationDiagnostics(
            startElapsedRealtimeNs = calibration.startedElapsedRealtimeNs,
            endElapsedRealtimeNs = integrationStartNs,
            accelerationSampleCount = calibration.accelerationSampleCount,
            angularRateSampleCount = calibration.angularRateSampleCount,
            meanAccelerationBodyMps2 = gravityBody,
            accelerationNormMeanMps2 = gravityBody.norm(),
            accelerationNormStdMps2 = calibration.gravityStdDeviceMps2.norm(),
            angularRateRmsRadps = calibration.gyroscopeStdDeviceRadps.norm(),
            initialRollRad = rollRad,
            initialPitchRad = pitchRad,
            initialYawRad = yawRad,
            gravityRemovedAccelerationNormMps2 = gravityRemovedNorm,
            result = NavigationInitializationResultType.ACCEPTED,
            source = NavigationInitializationSource.SESSION_CALIBRATION,
            angularRateSource = input.angularRateSource,
            calibrationCompletedElapsedRealtimeNs = calibration.completedElapsedRealtimeNs,
            positionSourceElapsedRealtimeNs = initialGnss.elapsedRealtimeNs,
            gyroscopeBiasBodyRadps = initialBias
        )
        return NavigationInitializationResult(
            attitudeBodyToEnu = attitude,
            endElapsedRealtimeNs = integrationStartNs,
            initialPositionEnuM = initialGnss.positionEnuM,
            initialVelocityEnuMps = initialGnss.velocityEnuMps ?: EnuVelocity.ZERO,
            initialGyroscopeBiasBodyRadps = initialBias,
            diagnostics = diagnostics
        )
    }

    private fun isValidCalibration(value: SessionCalibration): Boolean =
        value.startedElapsedRealtimeNs >= 0L &&
            value.completedElapsedRealtimeNs > value.startedElapsedRealtimeNs &&
            value.accelerationSampleCount >= parameters.minimumAccelerationSamples &&
            value.angularRateSampleCount >= parameters.minimumAngularRateSamples &&
            value.gravityDeviceMps2.isFinite() &&
            value.gyroscopeBiasDeviceRadps.isFinite() &&
            value.gravityDeviceMps2.norm() in parameters.minimumAccelerationNormMps2..
                parameters.maximumAccelerationNormMps2

    private fun latestGnssAtOrBefore(
        input: PreparedNavigationInput,
        elapsedRealtimeNs: Long
    ): NavigationGnssUpdate = input.events.asSequence()
        .filterIsInstance<PreparedNavigationEvent.Gnss>()
        .map { it.update }
        .filter { it.elapsedRealtimeNs <= elapsedRealtimeNs }
        .lastOrNull()
        ?: error("初期化時刻以前に採用できるGNSS観測がありません。")

    private fun initialYaw(velocity: EnuVelocity?): Double {
        if (velocity == null) return 0.0
        val horizontalSpeedMps = sqrt(
            velocity.eastMps * velocity.eastMps + velocity.northMps * velocity.northMps
        )
        return if (horizontalSpeedMps >= parameters.minimumInitialYawSpeedMps) {
            atan2(velocity.eastMps, velocity.northMps)
        } else 0.0
    }

    private fun evaluateWindow(
        input: PreparedNavigationInput,
        events: List<PreparedNavigationEvent.Imu>,
        startNs: Long,
        endNs: Long
    ): NavigationInitializationDiagnostics {
        val acceleration = events.asSequence()
            .filter { it.elapsedRealtimeNs in startNs..endNs }
            .mapNotNull { it.sample.accelerationBodyMps2 }
            .toList()
        val angularRate = events.asSequence()
            .filter { it.elapsedRealtimeNs in startNs..endNs }
            .mapNotNull { it.sample.angularRateBodyRadps }
            .toList()
        val accelerationMean = acceleration.meanVector()
        val norms = acceleration.map { it.norm() }
        val normMean = norms.averageOrNaN()
        val normStd = norms.standardDeviation(normMean)
        val angularRateRms = sqrt(angularRate.map { it.normSquared() }.averageOrNaN())
        val result = when {
            acceleration.size < parameters.minimumAccelerationSamples ||
                angularRate.size < parameters.minimumAngularRateSamples ->
                NavigationInitializationResultType.INSUFFICIENT_SAMPLES
            normMean !in parameters.minimumAccelerationNormMps2..
                parameters.maximumAccelerationNormMps2 ->
                NavigationInitializationResultType.ACCELERATION_NORM
            normStd > parameters.maximumAccelerationNormStdMps2 ->
                NavigationInitializationResultType.ACCELERATION_VARIATION
            angularRateRms > parameters.maximumAngularRateRmsRadps ->
                NavigationInitializationResultType.ANGULAR_RATE
            else -> NavigationInitializationResultType.ACCEPTED
        }
        return NavigationInitializationDiagnostics(
            startElapsedRealtimeNs = startNs,
            endElapsedRealtimeNs = endNs,
            accelerationSampleCount = acceleration.size,
            angularRateSampleCount = angularRate.size,
            meanAccelerationBodyMps2 = accelerationMean,
            accelerationNormMeanMps2 = normMean,
            accelerationNormStdMps2 = normStd,
            angularRateRmsRadps = angularRateRms,
            initialRollRad = null,
            initialPitchRad = null,
            initialYawRad = null,
            gravityRemovedAccelerationNormMps2 = null,
            result = result
        )
    }

    private fun createResult(
        input: PreparedNavigationInput,
        diagnostics: NavigationInitializationDiagnostics
    ): NavigationInitializationResult {
        val acceleration = diagnostics.meanAccelerationBodyMps2
        val rollRad = atan2(acceleration.y, acceleration.z)
        val pitchRad = atan2(
            -acceleration.x,
            sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z)
        )
        val horizontalSpeedMps = sqrt(
            input.initialVelocityEnuMps.eastMps * input.initialVelocityEnuMps.eastMps +
                input.initialVelocityEnuMps.northMps * input.initialVelocityEnuMps.northMps
        )
        val yawRad = if (horizontalSpeedMps >= parameters.minimumInitialYawSpeedMps) {
            atan2(input.initialVelocityEnuMps.eastMps, input.initialVelocityEnuMps.northMps)
        } else 0.0
        val attitude = AttitudeQuaternion.fromEulerAngles(rollRad, pitchRad, yawRad)
        val gravityRemoved = attitude.rotate(acceleration) - GRAVITY_ENU_MPS2
        val completed = diagnostics.copy(
            initialRollRad = rollRad,
            initialPitchRad = pitchRad,
            initialYawRad = yawRad,
            gravityRemovedAccelerationNormMps2 = gravityRemoved.norm()
        )
        require(completed.gravityRemovedAccelerationNormMps2!! <=
            parameters.maximumGravityRemovedAccelerationMps2) {
            "初期姿勢の重力除去後加速度が大きすぎます: " +
                "${completed.gravityRemovedAccelerationNormMps2} m/s^2"
        }
        val initialGnss = latestGnssAtOrBefore(input, diagnostics.endElapsedRealtimeNs)
        return NavigationInitializationResult(
            attitudeBodyToEnu = attitude,
            endElapsedRealtimeNs = diagnostics.endElapsedRealtimeNs,
            initialPositionEnuM = initialGnss.positionEnuM,
            initialVelocityEnuMps = initialGnss.velocityEnuMps ?: EnuVelocity.ZERO,
            initialGyroscopeBiasBodyRadps = BodyVector(0.0, 0.0, 0.0),
            diagnostics = completed.copy(
                source = NavigationInitializationSource.STABLE_WINDOW,
                angularRateSource = input.angularRateSource,
                positionSourceElapsedRealtimeNs = initialGnss.elapsedRealtimeNs
            )
        )
    }

    private fun List<BodyVector>.meanVector(): BodyVector = if (isEmpty()) {
        BodyVector(Double.NaN, Double.NaN, Double.NaN)
    } else {
        BodyVector(sumOf { it.x } / size, sumOf { it.y } / size, sumOf { it.z } / size)
    }

    private fun List<Double>.averageOrNaN(): Double = if (isEmpty()) Double.NaN else average()
    private fun List<Double>.standardDeviation(mean: Double): Double =
        if (isEmpty() || !mean.isFinite()) Double.NaN else sqrt(sumOf { (it - mean) * (it - mean) } / size)
    private fun BodyVector.normSquared(): Double = x * x + y * y + z * z
    private fun BodyVector.norm(): Double = sqrt(normSquared())
    private fun BodyVector.isFinite(): Boolean = x.isFinite() && y.isFinite() && z.isFinite()
    private operator fun BodyVector.minus(other: BodyVector) =
        BodyVector(x - other.x, y - other.y, z - other.z)

    companion object {
        private val GRAVITY_ENU_MPS2 = BodyVector(0.0, 0.0, 9.80665)
    }
}

data class NavigationInitializationParameters(
    val windowDurationNs: Long,
    val windowStepNs: Long,
    val searchDurationNs: Long,
    val minimumAccelerationSamples: Int,
    val minimumAngularRateSamples: Int,
    val minimumAccelerationNormMps2: Double,
    val maximumAccelerationNormMps2: Double,
    val maximumAccelerationNormStdMps2: Double,
    val maximumAngularRateRmsRadps: Double,
    val minimumInitialYawSpeedMps: Double,
    val maximumGravityRemovedAccelerationMps2: Double
) {
    companion object {
        val DEFAULT = NavigationInitializationParameters(
            windowDurationNs = 2_000_000_000L,
            windowStepNs = 500_000_000L,
            searchDurationNs = 10_000_000_000L,
            minimumAccelerationSamples = 100,
            minimumAngularRateSamples = 100,
            minimumAccelerationNormMps2 = 8.8,
            maximumAccelerationNormMps2 = 10.8,
            maximumAccelerationNormStdMps2 = 1.0,
            maximumAngularRateRmsRadps = 0.15,
            minimumInitialYawSpeedMps = 5.0,
            maximumGravityRemovedAccelerationMps2 = 1.0
        )
    }
}

data class NavigationInitializationResult(
    val attitudeBodyToEnu: AttitudeQuaternion,
    val endElapsedRealtimeNs: Long,
    val initialPositionEnuM: EnuPosition,
    val initialVelocityEnuMps: EnuVelocity,
    val initialGyroscopeBiasBodyRadps: BodyVector,
    val diagnostics: NavigationInitializationDiagnostics
)

enum class NavigationInitializationSource {
    SESSION_CALIBRATION,
    STABLE_WINDOW
}

data class NavigationInitializationDiagnostics(
    val startElapsedRealtimeNs: Long,
    val endElapsedRealtimeNs: Long,
    val accelerationSampleCount: Int,
    val angularRateSampleCount: Int,
    val meanAccelerationBodyMps2: BodyVector,
    val accelerationNormMeanMps2: Double,
    val accelerationNormStdMps2: Double,
    val angularRateRmsRadps: Double,
    val initialRollRad: Double?,
    val initialPitchRad: Double?,
    val initialYawRad: Double?,
    val gravityRemovedAccelerationNormMps2: Double?,
    val result: NavigationInitializationResultType,
    val source: NavigationInitializationSource = NavigationInitializationSource.STABLE_WINDOW,
    val angularRateSource: AngularRateSource = AngularRateSource.CORRECTED,
    val calibrationCompletedElapsedRealtimeNs: Long? = null,
    val positionSourceElapsedRealtimeNs: Long? = null,
    val gyroscopeBiasBodyRadps: BodyVector = BodyVector(0.0, 0.0, 0.0)
)

enum class NavigationInitializationResultType {
    ACCEPTED,
    INSUFFICIENT_SAMPLES,
    ACCELERATION_NORM,
    ACCELERATION_VARIATION,
    ANGULAR_RATE
}
