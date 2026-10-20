package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * IMU予測へGNSS位置・速度補正を適用するオフライン評価器。
 * 入力: PreparedNavigationInput
 * 出力: 補正前GNSS残差、補正後予測サンプル、各GNSS更新結果
 *
 * この実装は15状態ESKF前の閉ループ確認用である。
 * 姿勢とIMUバイアスは補正せず、位置と速度だけを軸別分散で補正する。
 */
class GnssAidedOfflineNavigationPredictor(
    private val measurementParameters: NavigationMeasurementParameters =
        NavigationMeasurementParameters.DEFAULT,
    private val headingParameters: NavigationHeadingParameters =
        NavigationHeadingParameters.DEFAULT,
    private val gravityParameters: NavigationGravityParameters =
        NavigationGravityParameters.DEFAULT,
    private val outputIntervalNs: Long = DEFAULT_OUTPUT_INTERVAL_NS
) {
    fun run(input: PreparedNavigationInput): GnssAidedNavigationResult {
        require(input.events.isNotEmpty()) { "推定入力イベントがありません。" }
        val initialAttitude = createInitialAttitude(input)
        var state = NavigationState(
            elapsedRealtimeNs = input.initialElapsedRealtimeNs,
            positionEnuM = input.initialPositionEnuM,
            velocityEnuMps = input.initialVelocityEnuMps,
            attitudeBodyToEnu = initialAttitude,
            accelerometerBiasMps2 = ZERO_VECTOR,
            gyroscopeBiasRadps = ZERO_VECTOR,
            accelerationEnuMps2 = ZERO_VECTOR,
            status = NavigationPredictionStatus.WAITING_FOR_IMU
        )
        var variance = NavigationAxisVariance.DEFAULT
        var accelerationBodyMps2: BodyVector? = null
        var angularRateBodyRadps: BodyVector? = null
        var nextOutputNs = input.initialElapsedRealtimeNs
        val updater = NavigationMeasurementUpdater(measurementParameters)
        val headingUpdater = NavigationHeadingUpdater(headingParameters)
        val gravityUpdater = NavigationGravityUpdater(gravityParameters)
        var headingVarianceRad2 = headingParameters.initialHeadingVarianceRad2
        val samples = mutableListOf<NavigationPredictionSample>()
        val residuals = mutableListOf<NavigationGnssResidual>()
        val updates = mutableListOf<NavigationMeasurementUpdateRecord>()
        val gravityUpdates = mutableListOf<NavigationGravityUpdateRecord>()
        var previousGravityUpdateNs: Long? = null
        var consecutivePositionRejections = 0
        var consecutiveVelocityRejections = 0
        var previousGnssUpdate: NavigationGnssUpdate? = null

        input.events.asSequence()
            .filter { it.elapsedRealtimeNs >= input.initialElapsedRealtimeNs }
            .forEach { event ->
                val dtNs = event.elapsedRealtimeNs - state.elapsedRealtimeNs
                if (dtNs > 0L && dtNs <= MAXIMUM_INTEGRATION_INTERVAL_NS) {
                    val dtSec = dtNs / NS_PER_SEC
                    state = predict(
                        state = state,
                        timestampNs = event.elapsedRealtimeNs,
                        dtSec = dtSec,
                        accelerationBodyMps2 = accelerationBodyMps2,
                        angularRateBodyRadps = angularRateBodyRadps
                    )
                    variance = variance.predict(dtSec, measurementParameters)
                    headingVarianceRad2 = headingUpdater.predictVariance(
                        headingVarianceRad2,
                        dtSec
                    )
                } else if (dtNs > MAXIMUM_INTEGRATION_INTERVAL_NS) {
                    state = state.copy(
                        elapsedRealtimeNs = event.elapsedRealtimeNs,
                        status = NavigationPredictionStatus.IMU_GAP
                    )
                }

                when (event) {
                    is PreparedNavigationEvent.Imu -> {
                        event.sample.accelerationBodyMps2?.let { accelerationBodyMps2 = it }
                        event.sample.angularRateBodyRadps?.let { angularRateBodyRadps = it }
                        if (event.sample.accelerationBodyMps2 != null) {
                            val gravityDtSec = previousGravityUpdateNs?.let {
                                (event.elapsedRealtimeNs - it) / NS_PER_SEC
                            } ?: 0.0
                            val gravityResult = gravityUpdater.update(
                                attitudeBodyToEnu = state.attitudeBodyToEnu,
                                accelerationBodyMps2 = accelerationBodyMps2,
                                angularRateBodyRadps = angularRateBodyRadps,
                                dtSec = gravityDtSec
                            )
                            state = state.copy(
                                attitudeBodyToEnu = gravityResult.attitudeBodyToEnu
                            )
                            gravityUpdates += NavigationGravityUpdateRecord(
                                elapsedRealtimeNs = event.elapsedRealtimeNs,
                                available = gravityResult.available,
                                accepted = gravityResult.accepted,
                                accelerationNormMps2 = gravityResult.accelerationNormMps2,
                                angularRateNormRadps = gravityResult.angularRateNormRadps,
                                residualRad = gravityResult.residualRad,
                                correctionRad = gravityResult.correctionRad
                            )
                            previousGravityUpdateNs = event.elapsedRealtimeNs
                        }
                    }
                    is PreparedNavigationEvent.Gnss -> {
                        val residual = createResidual(state, event.update)
                        residuals += residual
                        var result = updater.update(state, variance, event.update)
                        consecutivePositionRejections = if (result.positionAccepted) {
                            0
                        } else {
                            consecutivePositionRejections + 1
                        }
                        consecutiveVelocityRejections = if (
                            event.update.velocityEnuMps == null || result.velocityAccepted
                        ) {
                            0
                        } else {
                            consecutiveVelocityRejections + 1
                        }
                        if (
                            shouldReacquire(
                                consecutivePositionRejections = consecutivePositionRejections,
                                previous = previousGnssUpdate,
                                current = event.update
                            )
                        ) {
                            result = updater.reacquire(state, event.update)
                            consecutivePositionRejections = 0
                            consecutiveVelocityRejections = 0
                        }
                        val headingResult = headingUpdater.update(
                            attitudeBodyToEnu = result.state.attitudeBodyToEnu,
                            headingVarianceRad2 = headingVarianceRad2,
                            measurement = event.update
                        )
                        headingVarianceRad2 = headingResult.headingVarianceRad2
                        state = result.state.copy(
                            attitudeBodyToEnu = headingResult.attitudeBodyToEnu
                        )
                        variance = result.variance
                        updates += NavigationMeasurementUpdateRecord(
                            elapsedRealtimeNs = event.elapsedRealtimeNs,
                            positionAccepted = result.positionAccepted,
                            velocityAccepted = result.velocityAccepted,
                            horizontalPositionInnovationM = result.horizontalPositionInnovationM,
                            velocityInnovationMps = result.velocityInnovationMps,
                            positionGain = result.positionGain,
                            velocityGain = result.velocityGain,
                            horizontalPositionGateM = result.horizontalPositionGateM,
                            verticalPositionGateM = result.verticalPositionGateM,
                            velocityGateMps = result.velocityGateMps,
                            consecutivePositionRejectionCount = consecutivePositionRejections,
                            consecutiveVelocityRejectionCount = consecutiveVelocityRejections,
                            updateMode = result.updateMode,
                            headingAvailable = headingResult.available,
                            headingAccepted = headingResult.accepted,
                            headingInnovationRad = headingResult.innovationRad,
                            headingGateRad = headingResult.gateRad,
                            headingGain = headingResult.gain,
                            headingCorrectionRad = headingResult.correctionRad,
                            headingVarianceRad2 = headingVarianceRad2,
                            rejectionReason = result.rejectionReason
                        )
                        previousGnssUpdate = event.update
                    }
                }
                if (state.elapsedRealtimeNs >= nextOutputNs) {
                    samples += state.toSample()
                    nextOutputNs = state.elapsedRealtimeNs + outputIntervalNs
                }
            }
        return GnssAidedNavigationResult(
            prediction = OfflineNavigationPredictionResult(samples, residuals),
            measurementUpdates = updates,
            gravityUpdates = gravityUpdates
        )
    }

    private fun shouldReacquire(
        consecutivePositionRejections: Int,
        previous: NavigationGnssUpdate?,
        current: NavigationGnssUpdate
    ): Boolean {
        if (consecutivePositionRejections < measurementParameters.reacquisitionRejectionCount) {
            return false
        }
        if (current.positionStdM > measurementParameters.reacquisitionMaximumHorizontalAccuracyM) {
            return false
        }
        if (previous == null) return false
        val intervalNs = current.elapsedRealtimeNs - previous.elapsedRealtimeNs
        if (intervalNs <= 0L) return false
        val intervalSec = intervalNs / NS_PER_SEC
        val east = current.positionEnuM.eastM - previous.positionEnuM.eastM
        val north = current.positionEnuM.northM - previous.positionEnuM.northM
        val displacementM = sqrt(east * east + north * north)
        val referenceSpeedMps = current.velocityEnuMps?.let {
            sqrt(it.eastMps * it.eastMps + it.northMps * it.northMps)
        } ?: previous.velocityEnuMps?.let {
            sqrt(it.eastMps * it.eastMps + it.northMps * it.northMps)
        } ?: 0.0
        val maximumDisplacementM = maxOf(
            measurementParameters.reacquisitionMinimumPositionAllowanceM,
            referenceSpeedMps * intervalSec +
                measurementParameters.reacquisitionPositionAllowanceM
        )
        return displacementM <= maximumDisplacementM
    }

    private fun predict(
        state: NavigationState,
        timestampNs: Long,
        dtSec: Double,
        accelerationBodyMps2: BodyVector?,
        angularRateBodyRadps: BodyVector?
    ): NavigationState {
        val correctedRate = angularRateBodyRadps?.minus(state.gyroscopeBiasRadps) ?: ZERO_VECTOR
        val attitude = (
            state.attitudeBodyToEnu *
                AttitudeQuaternion.fromAngularVelocity(correctedRate, dtSec)
            ).normalized()
        val correctedAcceleration = accelerationBodyMps2?.minus(state.accelerometerBiasMps2)
        val accelerationEnu = correctedAcceleration?.let {
            attitude.rotate(it) - GRAVITY_ENU_MPS2
        } ?: ZERO_VECTOR
        val velocity = EnuVelocity(
            eastMps = state.velocityEnuMps.eastMps + accelerationEnu.x * dtSec,
            northMps = state.velocityEnuMps.northMps + accelerationEnu.y * dtSec,
            upMps = state.velocityEnuMps.upMps + accelerationEnu.z * dtSec
        )
        val position = EnuPosition(
            eastM = state.positionEnuM.eastM +
                0.5 * (state.velocityEnuMps.eastMps + velocity.eastMps) * dtSec,
            northM = state.positionEnuM.northM +
                0.5 * (state.velocityEnuMps.northMps + velocity.northMps) * dtSec,
            upM = state.positionEnuM.upM +
                0.5 * (state.velocityEnuMps.upMps + velocity.upMps) * dtSec
        )
        return state.copy(
            elapsedRealtimeNs = timestampNs,
            positionEnuM = position,
            velocityEnuMps = velocity,
            attitudeBodyToEnu = attitude,
            accelerationEnuMps2 = accelerationEnu,
            status = if (accelerationBodyMps2 == null || angularRateBodyRadps == null) {
                NavigationPredictionStatus.WAITING_FOR_IMU
            } else NavigationPredictionStatus.PREDICTING
        )
    }

    private fun createInitialAttitude(input: PreparedNavigationInput): AttitudeQuaternion {
        val acceleration = input.events.asSequence()
            .takeWhile { it.elapsedRealtimeNs <= input.initialElapsedRealtimeNs }
            .filterIsInstance<PreparedNavigationEvent.Imu>()
            .mapNotNull { it.sample.accelerationBodyMps2 }
            .lastOrNull()
        val horizontalSpeedMps = sqrt(
            input.initialVelocityEnuMps.eastMps * input.initialVelocityEnuMps.eastMps +
                input.initialVelocityEnuMps.northMps * input.initialVelocityEnuMps.northMps
        )
        val yawRad = if (horizontalSpeedMps >= MINIMUM_INITIAL_YAW_SPEED_MPS) {
            atan2(
                input.initialVelocityEnuMps.eastMps,
                input.initialVelocityEnuMps.northMps
            )
        } else 0.0
        if (acceleration == null) {
            return AttitudeQuaternion.fromEulerAngles(0.0, 0.0, yawRad)
        }
        val rollRad = atan2(acceleration.y, acceleration.z)
        val pitchRad = atan2(
            -acceleration.x,
            sqrt(acceleration.y * acceleration.y + acceleration.z * acceleration.z)
        )
        return AttitudeQuaternion.fromEulerAngles(rollRad, pitchRad, yawRad)
    }

    private fun createResidual(
        state: NavigationState,
        measurement: NavigationGnssUpdate
    ): NavigationGnssResidual {
        val eastErrorM = state.positionEnuM.eastM - measurement.positionEnuM.eastM
        val northErrorM = state.positionEnuM.northM - measurement.positionEnuM.northM
        val upErrorM = state.positionEnuM.upM - measurement.positionEnuM.upM
        val velocityErrorMps = measurement.velocityEnuMps?.let {
            val east = state.velocityEnuMps.eastMps - it.eastMps
            val north = state.velocityEnuMps.northMps - it.northMps
            val up = state.velocityEnuMps.upMps - it.upMps
            sqrt(east * east + north * north + up * up)
        }
        return NavigationGnssResidual(
            elapsedRealtimeNs = measurement.elapsedRealtimeNs,
            eastErrorM = eastErrorM,
            northErrorM = northErrorM,
            upErrorM = upErrorM,
            horizontalErrorM = sqrt(eastErrorM * eastErrorM + northErrorM * northErrorM),
            velocityErrorMps = velocityErrorMps
        )
    }

    private fun NavigationState.toSample() = NavigationPredictionSample(
        elapsedRealtimeNs = elapsedRealtimeNs,
        positionEnuM = positionEnuM,
        velocityEnuMps = velocityEnuMps,
        attitudeBodyToEnu = attitudeBodyToEnu,
        accelerationEnuMps2 = accelerationEnuMps2,
        status = status
    )

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val DEFAULT_OUTPUT_INTERVAL_NS = 10_000_000L
        private const val MAXIMUM_INTEGRATION_INTERVAL_NS = 100_000_000L
        private const val MINIMUM_INITIAL_YAW_SPEED_MPS = 2.0
        private val ZERO_VECTOR = BodyVector(0.0, 0.0, 0.0)
        private val GRAVITY_ENU_MPS2 = BodyVector(0.0, 0.0, 9.80665)
    }
}

data class GnssAidedNavigationResult(
    val prediction: OfflineNavigationPredictionResult,
    val measurementUpdates: List<NavigationMeasurementUpdateRecord>,
    val gravityUpdates: List<NavigationGravityUpdateRecord>
)

data class NavigationGravityUpdateRecord(
    val elapsedRealtimeNs: Long,
    val available: Boolean,
    val accepted: Boolean,
    val accelerationNormMps2: Double?,
    val angularRateNormRadps: Double?,
    val residualRad: Double?,
    val correctionRad: Double?
)

data class NavigationMeasurementUpdateRecord(
    val elapsedRealtimeNs: Long,
    val positionAccepted: Boolean,
    val velocityAccepted: Boolean,
    val horizontalPositionInnovationM: Double,
    val velocityInnovationMps: Double?,
    val positionGain: AxisGain?,
    val velocityGain: AxisGain?,
    val horizontalPositionGateM: Double,
    val verticalPositionGateM: Double,
    val velocityGateMps: Double?,
    val consecutivePositionRejectionCount: Int,
    val consecutiveVelocityRejectionCount: Int,
    val updateMode: NavigationPositionUpdateMode,
    val headingAvailable: Boolean,
    val headingAccepted: Boolean,
    val headingInnovationRad: Double?,
    val headingGateRad: Double?,
    val headingGain: Double?,
    val headingCorrectionRad: Double?,
    val headingVarianceRad2: Double,
    val rejectionReason: NavigationMeasurementRejectionReason?
)

private operator fun BodyVector.minus(other: BodyVector) = BodyVector(
    x = x - other.x,
    y = y - other.y,
    z = z - other.z
)
