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
    private val eskfNoiseParameters: NavigationEskfNoiseParameters =
        NavigationEskfNoiseParameters.DEFAULT,
    private val outputIntervalNs: Long = DEFAULT_OUTPUT_INTERVAL_NS,
    private val attitudePredictionSink: NavigationAttitudePredictionSink =
        NavigationAttitudePredictionSink.NONE,
    private val verticalObservations: List<NavigationVerticalObservation> = emptyList(),
    private val verticalUpdateWriter: NavigationVerticalUpdateWriter? = null,
    private val attitudeDiagnosticIntervalNs: Long = DEFAULT_ATTITUDE_DIAGNOSTIC_INTERVAL_NS
) {
    init {
        require(attitudeDiagnosticIntervalNs > 0L) {
            "姿勢診断の出力周期は1 ns以上にしてください。"
        }
    }
    fun run(input: PreparedNavigationInput): GnssAidedNavigationResult {
        require(input.events.isNotEmpty()) { "推定入力イベントがありません。" }
        val initialization = NavigationInitializer().initialize(input)
        var state = NavigationState(
            elapsedRealtimeNs = initialization.endElapsedRealtimeNs,
            positionEnuM = initialization.initialPositionEnuM,
            velocityEnuMps = initialization.initialVelocityEnuMps,
            attitudeBodyToEnu = initialization.attitudeBodyToEnu,
            accelerometerBiasMps2 = ZERO_VECTOR,
            gyroscopeBiasRadps = initialization.initialGyroscopeBiasBodyRadps,
            accelerationEnuMps2 = ZERO_VECTOR,
            status = NavigationPredictionStatus.WAITING_FOR_IMU
        )
        var variance = NavigationAxisVariance.DEFAULT
        var eskfCovariance = NavigationEskfCovariance.initial(eskfNoiseParameters)
        var accelerationBodyMps2: BodyVector? = null
        var angularRateBodyRadps: BodyVector? = null
        var nextOutputNs = initialization.endElapsedRealtimeNs
        val updater = NavigationMeasurementUpdater(measurementParameters)
        val headingUpdater = NavigationHeadingUpdater(headingParameters)
        val gravityUpdater = NavigationGravityUpdater(gravityParameters)
        var headingVarianceRad2 = headingParameters.initialHeadingVarianceRad2
        val samples = mutableListOf<NavigationPredictionSample>()
        val residuals = mutableListOf<NavigationGnssResidual>()
        val updates = mutableListOf<NavigationMeasurementUpdateRecord>()
        val gravityUpdates = mutableListOf<NavigationGravityUpdateRecord>()
        var attitudePredictionCount = 0L
        var nextAttitudeDiagnosticNs = initialization.endElapsedRealtimeNs
        var previousAbnormalReason = ""
        val headingDirections = mutableListOf<NavigationHeadingDirectionRecord>()
        val headingDirectionTracker = NavigationHeadingDirectionTracker()
        val headingTrackingController = NavigationHeadingTrackingController()
        val headingTrackingRecords = mutableListOf<NavigationHeadingTrackingRecord>()
        val accelerometerBiasUpdater = NavigationAccelerometerBiasUpdater()
        val accelerometerBiasUpdates = mutableListOf<NavigationAccelerometerBiasUpdateRecord>()
        val horizontalErrorAnalyzer = NavigationHorizontalErrorAnalyzer(verticalObservations)
        val horizontalErrorRecords = mutableListOf<NavigationHorizontalErrorRecord>()
        val verticalUpdater = NavigationVerticalUpdater()
        var verticalObservationIndex = 0
        var previousGravityUpdateNs: Long? = null
        var consecutivePositionRejections = 0
        var consecutiveVelocityRejections = 0
        var previousGnssUpdate: NavigationGnssUpdate? = null

        input.events.asSequence()
            .filter { it.elapsedRealtimeNs >= initialization.endElapsedRealtimeNs }
            .forEach { event ->
                val dtNs = event.elapsedRealtimeNs - state.elapsedRealtimeNs
                if (dtNs > 0L && dtNs <= MAXIMUM_INTEGRATION_INTERVAL_NS) {
                    val dtSec = dtNs / NS_PER_SEC
                    val correctedAcceleration = accelerationBodyMps2?.minus(state.accelerometerBiasMps2)
                        ?: ZERO_VECTOR
                    val correctedAngularRate = angularRateBodyRadps?.minus(state.gyroscopeBiasRadps)
                        ?: ZERO_VECTOR
                    eskfCovariance = eskfCovariance.predict(
                        dtSec = dtSec,
                        attitudeBodyToEnu = state.attitudeBodyToEnu,
                        accelerationBodyMps2 = correctedAcceleration,
                        angularRateBodyRadps = correctedAngularRate,
                        parameters = eskfNoiseParameters
                    )
                    val attitudeBefore = state.attitudeBodyToEnu
                    val eulerBefore = attitudeBefore.toEulerAngles()
                    state = predict(
                        state = state,
                        timestampNs = event.elapsedRealtimeNs,
                        dtSec = dtSec,
                        accelerationBodyMps2 = accelerationBodyMps2,
                        angularRateBodyRadps = angularRateBodyRadps
                    )
                    val attitudeAfter = state.attitudeBodyToEnu
                    val eulerAfter = attitudeAfter.toEulerAngles()
                    val abnormalReason = predictionAbnormalReason(state)
                    val source = when (event) {
                        is PreparedNavigationEvent.Imu -> "IMU"
                        is PreparedNavigationEvent.Gnss -> "GNSS"
                    }
                    val abnormalChanged = abnormalReason != previousAbnormalReason
                    val shouldWriteAttitudeDiagnostic =
                        event.elapsedRealtimeNs >= nextAttitudeDiagnosticNs ||
                            abnormalChanged || source == "GNSS"
                    if (shouldWriteAttitudeDiagnostic) {
                        attitudePredictionSink.write(
                            NavigationAttitudePredictionRecord(
                                elapsedRealtimeNs = event.elapsedRealtimeNs,
                                source = source,
                                dtSec = dtSec,
                                angularRateForwardRadps = correctedAngularRate.x,
                                angularRateLeftRadps = correctedAngularRate.y,
                                angularRateUpRadps = correctedAngularRate.z,
                                accelerationBodyForwardMps2 = correctedAcceleration.x,
                                accelerationBodyLeftMps2 = correctedAcceleration.y,
                                accelerationBodyUpMps2 = correctedAcceleration.z,
                                accelerationEnuEastMps2 = state.accelerationEnuMps2.x,
                                accelerationEnuNorthMps2 = state.accelerationEnuMps2.y,
                                accelerationEnuUpMps2 = state.accelerationEnuMps2.z,
                                velocityEastMps = state.velocityEnuMps.eastMps,
                                velocityNorthMps = state.velocityEnuMps.northMps,
                                velocityUpMps = state.velocityEnuMps.upMps,
                                rollAfterRad = eulerAfter.rollRad,
                                pitchAfterRad = eulerAfter.pitchRad,
                                yawAfterRad = eulerAfter.yawRad,
                                abnormalReason = abnormalReason,
                                quaternionBefore = attitudeBefore,
                                quaternionAfter = attitudeAfter,
                                rollDeltaRad = wrapHeadingAngle(eulerAfter.rollRad - eulerBefore.rollRad),
                                pitchDeltaRad = wrapHeadingAngle(eulerAfter.pitchRad - eulerBefore.pitchRad),
                                yawDeltaRad = wrapHeadingAngle(eulerAfter.yawRad - eulerBefore.yawRad)
                            )
                        )
                        attitudePredictionCount++
                        if (event.elapsedRealtimeNs >= nextAttitudeDiagnosticNs) {
                            nextAttitudeDiagnosticNs =
                                event.elapsedRealtimeNs + attitudeDiagnosticIntervalNs
                        }
                    }
                    previousAbnormalReason = abnormalReason
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
                            val gravityAttitudeBefore = state.attitudeBodyToEnu
                            val gravityEulerBefore = gravityAttitudeBefore.toEulerAngles()
                            val gravityResult = gravityUpdater.update(
                                attitudeBodyToEnu = gravityAttitudeBefore,
                                accelerationBodyMps2 = accelerationBodyMps2,
                                angularRateBodyRadps = angularRateBodyRadps,
                                dtSec = gravityDtSec
                            )
                            state = state.copy(
                                attitudeBodyToEnu = gravityResult.attitudeBodyToEnu
                            )
                            val gravityEulerAfter = gravityResult.attitudeBodyToEnu.toEulerAngles()
                            gravityUpdates += NavigationGravityUpdateRecord(
                                elapsedRealtimeNs = event.elapsedRealtimeNs,
                                available = gravityResult.available,
                                accepted = gravityResult.accepted,
                                accelerationNormMps2 = gravityResult.accelerationNormMps2,
                                angularRateNormRadps = gravityResult.angularRateNormRadps,
                                residualRad = gravityResult.residualRad,
                                correctionRad = gravityResult.correctionRad,
                                rollBeforeRad = gravityEulerBefore.rollRad,
                                pitchBeforeRad = gravityEulerBefore.pitchRad,
                                yawBeforeRad = gravityEulerBefore.yawRad,
                                rollAfterRad = gravityEulerAfter.rollRad,
                                pitchAfterRad = gravityEulerAfter.pitchRad,
                                yawAfterRad = gravityEulerAfter.yawRad,
                                rollDeltaRad = wrapHeadingAngle(
                                    gravityEulerAfter.rollRad - gravityEulerBefore.rollRad
                                ),
                                pitchDeltaRad = wrapHeadingAngle(
                                    gravityEulerAfter.pitchRad - gravityEulerBefore.pitchRad
                                ),
                                yawDeltaRad = wrapHeadingAngle(
                                    gravityEulerAfter.yawRad - gravityEulerBefore.yawRad
                                )
                            )
                            previousGravityUpdateNs = event.elapsedRealtimeNs
                        }
                    }
                    is PreparedNavigationEvent.Gnss -> {
                        val stateBeforeGnss = state    
                        val residual = createResidual(state, variance, eskfCovariance, event.update)
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
                        eskfCovariance = eskfCovariance.updateGnss(
                            measurement = event.update,
                            positionAccepted = result.positionAccepted,
                            velocityAccepted = result.velocityAccepted
                        )
                        val headingProbe = headingUpdater.update(
                            attitudeBodyToEnu = result.state.attitudeBodyToEnu,
                            headingVarianceRad2 = headingVarianceRad2,
                            measurement = event.update
                        )

                        val eskfHeadingStdBeforeRad = eskfCovariance.standardDeviation(
                            NavigationErrorStateIndex.ATTITUDE + 2
                        )

                        val oppositeHeadingInnovationRad = headingProbe.innovationRad?.let {
                            wrapHeadingAngle(it + kotlin.math.PI)
                        }

                        val oppositeHeadingIsCloser = if (
                            headingProbe.innovationRad != null &&
                            oppositeHeadingInnovationRad != null
                        ) {
                            kotlin.math.abs(oppositeHeadingInnovationRad) <
                                kotlin.math.abs(headingProbe.innovationRad)
                        } else {
                            null
                        }

                        val horizontalSpeedMps = event.update.velocityEnuMps?.let {
                            sqrt(
                                it.eastMps * it.eastMps +
                                    it.northMps * it.northMps
                            )
                        }

                        val yawRateRadps = angularRateBodyRadps?.z

                        val headingDirection = headingDirectionTracker.update(
                            elapsedRealtimeNs = event.elapsedRealtimeNs,
                            speedMps = horizontalSpeedMps,
                            bearingAccuracyRad = event.update.bearingAccuracyRad,
                            speedAccuracyMps = event.update.speedStdMps,
                            yawRateRadps = yawRateRadps,
                            regularInnovationRad = headingProbe.innovationRad,
                            oppositeInnovationRad = oppositeHeadingInnovationRad
                        )

                        headingDirections += headingDirection

                        // 確定した進行方向に合わせてGNSS方位を選択する。
                        val headingMeasurement = if (!headingDirection.headingUpdateAllowed) {
                            event.update.copy(
                                bearingRad = null,
                                bearingAccuracyRad = null
                            )
                        } else when (headingDirection.stateAfter) {
                            NavigationHeadingDirectionState.FORWARD -> event.update
                            NavigationHeadingDirectionState.REVERSE -> event.update.copy(
                                bearingRad = event.update.bearingRad?.let {
                                    wrapHeadingAngle(it + kotlin.math.PI)
                                }
                            )
                            NavigationHeadingDirectionState.UNRESOLVED,
                            NavigationHeadingDirectionState.FORWARD_CANDIDATE,
                            NavigationHeadingDirectionState.REVERSE_CANDIDATE ->
                                event.update.copy(
                                    bearingRad = null,
                                    bearingAccuracyRad = null
                                )
                        }

                        // headingMeasurement生成後に参照する。
                        val eskfHeadingMeasurementStdRad =
                            headingMeasurement.bearingAccuracyRad?.let {
                                maxOf(
                                    it,
                                    headingParameters.minimumBearingAccuracyRad
                                )
                            }

                        val headingResult = headingUpdater.update(
                            attitudeBodyToEnu = result.state.attitudeBodyToEnu,
                            headingVarianceRad2 = headingVarianceRad2,
                            measurement = headingMeasurement
                        )

                        headingVarianceRad2 =
                            headingResult.headingVarianceRad2

                        val eskfHeadingInnovationStdRad =
                            eskfHeadingMeasurementStdRad?.let {
                                sqrt(
                                    eskfHeadingStdBeforeRad * eskfHeadingStdBeforeRad +
                                        it * it
                                )
                            }

                        eskfCovariance = eskfCovariance.updateHeading(
                            measurement = headingMeasurement,
                            headingAccepted = headingResult.accepted,
                            minimumBearingAccuracyRad =
                                headingParameters.minimumBearingAccuracyRad
                        )

                        val eskfHeadingStdAfterRad =
                            eskfCovariance.standardDeviation(
                                NavigationErrorStateIndex.ATTITUDE + 2
                            )
                        state = result.state.copy(
                            attitudeBodyToEnu = headingResult.attitudeBodyToEnu
                        )
                        val trackingResult = headingTrackingController.update(
                            elapsedRealtimeNs = event.elapsedRealtimeNs,
                            attitudeBodyToEnu = state.attitudeBodyToEnu,
                            directionState = headingDirection.stateAfter,
                            bearingRad = event.update.bearingRad,
                            bearingAccuracyRad = event.update.bearingAccuracyRad,
                            speedAccuracyMps = event.update.speedStdMps,
                            speedMps = horizontalSpeedMps,
                            yawRateRadps = yawRateRadps,
                            headingUpdateAccepted = headingResult.accepted,
                            headingUpdateAvailable = headingResult.available,
                            directionConflict = headingDirection.headingUpdateRejectionReason ==
                                "DIRECTION_CONFLICT"
                        )
                        state = state.copy(
                            attitudeBodyToEnu = trackingResult.attitudeBodyToEnu
                        )
                        headingTrackingRecords += trackingResult.record
                        horizontalErrorRecords += horizontalErrorAnalyzer.analyze(
                            stateBeforeGnss = stateBeforeGnss,
                            gnss = event.update,
                            directionState = headingDirection.stateAfter,
                            trackingState = trackingResult.record.stateAfter
                        )
                        val gnssIntervalSec = previousGnssUpdate?.let {
                            (event.update.elapsedRealtimeNs - it.elapsedRealtimeNs) / NS_PER_SEC
                        }

                        val biasResult = accelerometerBiasUpdater.update(
                            elapsedRealtimeNs = event.elapsedRealtimeNs,
                            stateBeforeGnss = stateBeforeGnss.copy(
                                attitudeBodyToEnu = state.attitudeBodyToEnu,
                                accelerometerBiasMps2 = state.accelerometerBiasMps2
                            ),
                            gnssVelocity = event.update.velocityEnuMps,
                            gnssIntervalSec = gnssIntervalSec,
                            gnssSpeedStdMps = event.update.speedStdMps,
                            velocityAccepted = result.velocityAccepted,
                            directionState = headingDirection.stateAfter,
                            headingTrackingState = trackingResult.record.stateAfter,
                            yawRateRadps = yawRateRadps,
                            accelerationBodyMps2 = accelerationBodyMps2
                        )
                        
                        state = state.copy(
                            accelerometerBiasMps2 = biasResult.accelerometerBiasMps2
                        )

                        accelerometerBiasUpdates += biasResult.record
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
                            eskfHeadingStdBeforeRad = eskfHeadingStdBeforeRad,
                            eskfHeadingStdAfterRad = eskfHeadingStdAfterRad,
                            eskfHeadingMeasurementStdRad = eskfHeadingMeasurementStdRad,
                            eskfHeadingInnovationStdRad = eskfHeadingInnovationStdRad,
                            oppositeHeadingInnovationRad = oppositeHeadingInnovationRad,
                            oppositeHeadingIsCloser = oppositeHeadingIsCloser,
                            eskfNormalizedHeadingInnovation = if (
                                headingResult.innovationRad != null &&
                                eskfHeadingInnovationStdRad != null &&
                                eskfHeadingInnovationStdRad > 0.0
                            ) {
                                kotlin.math.abs(headingResult.innovationRad) /  eskfHeadingInnovationStdRad
                            } else null,
                            rejectionReason = result.rejectionReason
                        )
                        previousGnssUpdate = event.update
                    }
                }
                while (
                    verticalObservationIndex < verticalObservations.size &&
                    verticalObservations[verticalObservationIndex].elapsedRealtimeNs <=
                    state.elapsedRealtimeNs
                ) {
                    val observation = verticalObservations[verticalObservationIndex]
                    val stateBefore = state
                    val verticalResult = verticalUpdater.update(state, variance, observation)
                    state = verticalResult.state
                    variance = verticalResult.variance
                    verticalUpdateWriter?.write(observation, stateBefore, verticalResult)
                    verticalObservationIndex++
                }
                if (state.elapsedRealtimeNs >= nextOutputNs) {
                    samples += state.toSample(
                        variance = variance,
                        headingVarianceRad2 = headingVarianceRad2,
                        eskfCovariance = eskfCovariance,
                        angularRateBodyRadps = angularRateBodyRadps?.minus(state.gyroscopeBiasRadps)
                    )
                    nextOutputNs = state.elapsedRealtimeNs + outputIntervalNs
                }
            }
        return GnssAidedNavigationResult(
            initialization = initialization.diagnostics,
            prediction = OfflineNavigationPredictionResult(samples, residuals),
            measurementUpdates = updates,
            gravityUpdates = gravityUpdates,
            attitudePredictionCount = attitudePredictionCount,
            headingDirections = headingDirections,
            headingTrackingRecords = headingTrackingRecords,
            accelerometerBiasUpdates = accelerometerBiasUpdates,
            horizontalErrorRecords = horizontalErrorRecords
        )
    }

    /**
     * 重力除去後加速度と速度から、最初に確認すべき異常を分類する。
     * 入力: IMU予測後の状態
     * 出力: 異常理由。しきい値内は空文字
     */
    private fun predictionAbnormalReason(state: NavigationState): String {
        val acceleration = state.accelerationEnuMps2
        val horizontalAcceleration = sqrt(
            acceleration.x * acceleration.x + acceleration.y * acceleration.y
        )
        val accelerationNorm = sqrt(
            horizontalAcceleration * horizontalAcceleration + acceleration.z * acceleration.z
        )
        val horizontalSpeed = sqrt(
            state.velocityEnuMps.eastMps * state.velocityEnuMps.eastMps +
                state.velocityEnuMps.northMps * state.velocityEnuMps.northMps
        )
        return when {
            !accelerationNorm.isFinite() || !horizontalSpeed.isFinite() ||
                !state.velocityEnuMps.upMps.isFinite() -> "NON_FINITE"
            accelerationNorm >= ABNORMAL_ACCELERATION_NORM_MPS2 -> "ACCELERATION_NORM"
            horizontalAcceleration >= ABNORMAL_HORIZONTAL_ACCELERATION_MPS2 ->
                "HORIZONTAL_ACCELERATION"
            kotlin.math.abs(acceleration.z) >= ABNORMAL_VERTICAL_ACCELERATION_MPS2 ->
                "VERTICAL_ACCELERATION"
            kotlin.math.abs(state.velocityEnuMps.upMps) >= ABNORMAL_VERTICAL_SPEED_MPS ->
                "VERTICAL_SPEED"
            horizontalSpeed >= ABNORMAL_HORIZONTAL_SPEED_MPS -> "HORIZONTAL_SPEED"
            else -> ""
        }
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

    /**
     * GNSS観測直前の推定誤差と不確かさの整合性を記録する。
     * 入力: 推定状態、位置・速度の軸別分散、GNSS観測
     * 出力: GNSS残差、1 sigma標準偏差、正規化水平位置誤差
     */
    private fun createResidual(
        state: NavigationState,
        variance: NavigationAxisVariance,
        eskfCovariance: NavigationEskfCovariance,
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
        val horizontalErrorM = sqrt(eastErrorM * eastErrorM + northErrorM * northErrorM)
        val uncertainty = NavigationPredictionUncertainty.from(
            variance = variance,
            headingVarianceRad2 = 0.0
        )
        return NavigationGnssResidual(
            elapsedRealtimeNs = measurement.elapsedRealtimeNs,
            eastErrorM = eastErrorM,
            northErrorM = northErrorM,
            upErrorM = upErrorM,
            horizontalErrorM = horizontalErrorM,
            velocityErrorMps = velocityErrorMps,
            horizontalPositionStdM = uncertainty.horizontalPositionStdM,
            velocityStdMps = sqrt(
                variance.velocityEastMps2 +
                    variance.velocityNorthMps2 +
                    variance.velocityUpMps2
            ),
            normalizedHorizontalPositionError =
                horizontalErrorM / uncertainty.horizontalPositionStdM,
            eskfHorizontalPositionStdM =
                NavigationEskfUncertainty.from(eskfCovariance).horizontalPositionStdM,
            eskfNormalizedHorizontalPositionError = horizontalErrorM /
                NavigationEskfUncertainty.from(eskfCovariance).horizontalPositionStdM
        )
    }

    /**
     * 現在の推定状態と分散を100 Hz出力用サンプルへ変換する。
     * 入力: 位置・速度の軸別分散[m^2, (m/s)^2]、方位分散[rad^2]
     * 出力: 推定値と1 sigma標準偏差
     */
    private fun NavigationState.toSample(
        variance: NavigationAxisVariance,
        headingVarianceRad2: Double,
        eskfCovariance: NavigationEskfCovariance,
        angularRateBodyRadps: BodyVector?
    ) = NavigationPredictionSample(
        elapsedRealtimeNs = elapsedRealtimeNs,
        positionEnuM = positionEnuM,
        velocityEnuMps = velocityEnuMps,
        attitudeBodyToEnu = attitudeBodyToEnu,
        accelerationEnuMps2 = accelerationEnuMps2,
        angularRateBodyRadps = angularRateBodyRadps,
        status = status,
        uncertainty = NavigationPredictionUncertainty.from(
            variance = variance,
            headingVarianceRad2 = headingVarianceRad2
        ),
        eskfUncertainty = NavigationEskfUncertainty.from(eskfCovariance)
    )

    companion object {
        private const val NS_PER_SEC = 1_000_000_000.0
        private const val DEFAULT_OUTPUT_INTERVAL_NS = 10_000_000L
        private const val DEFAULT_ATTITUDE_DIAGNOSTIC_INTERVAL_NS = 10_000_000L
        private const val MAXIMUM_INTEGRATION_INTERVAL_NS = 100_000_000L
        private const val MINIMUM_INITIAL_YAW_SPEED_MPS = 2.0
        private const val ABNORMAL_ACCELERATION_NORM_MPS2 = 20.0
        private const val ABNORMAL_HORIZONTAL_ACCELERATION_MPS2 = 15.0
        private const val ABNORMAL_VERTICAL_ACCELERATION_MPS2 = 15.0
        private const val ABNORMAL_VERTICAL_SPEED_MPS = 20.0
        private const val ABNORMAL_HORIZONTAL_SPEED_MPS = 80.0
        private val ZERO_VECTOR = BodyVector(0.0, 0.0, 0.0)
        private val GRAVITY_ENU_MPS2 = BodyVector(0.0, 0.0, 9.80665)
    }
}

data class GnssAidedNavigationResult(
    val initialization: NavigationInitializationDiagnostics,
    val prediction: OfflineNavigationPredictionResult,
    val measurementUpdates: List<NavigationMeasurementUpdateRecord>,
    val gravityUpdates: List<NavigationGravityUpdateRecord>,
    val attitudePredictionCount: Long,
    val headingDirections: List<NavigationHeadingDirectionRecord>,
    val headingTrackingRecords: List<NavigationHeadingTrackingRecord>,
    val accelerometerBiasUpdates: List<NavigationAccelerometerBiasUpdateRecord>,
    val horizontalErrorRecords: List<NavigationHorizontalErrorRecord>
)

data class NavigationAttitudePredictionRecord(
    val elapsedRealtimeNs: Long,
    val source: String,
    val dtSec: Double,
    val angularRateForwardRadps: Double,
    val angularRateLeftRadps: Double,
    val angularRateUpRadps: Double,
    val accelerationBodyForwardMps2: Double,
    val accelerationBodyLeftMps2: Double,
    val accelerationBodyUpMps2: Double,
    val accelerationEnuEastMps2: Double,
    val accelerationEnuNorthMps2: Double,
    val accelerationEnuUpMps2: Double,
    val velocityEastMps: Double,
    val velocityNorthMps: Double,
    val velocityUpMps: Double,
    val rollAfterRad: Double,
    val pitchAfterRad: Double,
    val yawAfterRad: Double,
    val abnormalReason: String,
    val quaternionBefore: AttitudeQuaternion,
    val quaternionAfter: AttitudeQuaternion,
    val rollDeltaRad: Double,
    val pitchDeltaRad: Double,
    val yawDeltaRad: Double
)

data class NavigationGravityUpdateRecord(
    val elapsedRealtimeNs: Long,
    val available: Boolean,
    val accepted: Boolean,
    val accelerationNormMps2: Double?,
    val angularRateNormRadps: Double?,
    val residualRad: Double?,
    val correctionRad: Double?,
    val rollBeforeRad: Double,
    val pitchBeforeRad: Double,
    val yawBeforeRad: Double,
    val rollAfterRad: Double,
    val pitchAfterRad: Double,
    val yawAfterRad: Double,
    val rollDeltaRad: Double,
    val pitchDeltaRad: Double,
    val yawDeltaRad: Double
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
    val eskfHeadingStdBeforeRad: Double?,
    val eskfHeadingStdAfterRad: Double?,
    val eskfHeadingMeasurementStdRad: Double?,
    val eskfHeadingInnovationStdRad: Double?,
    val oppositeHeadingInnovationRad: Double?,
    val oppositeHeadingIsCloser: Boolean?,
    val eskfNormalizedHeadingInnovation: Double?,
    val rejectionReason: NavigationMeasurementRejectionReason?
)

/** 角度[rad]を-piからpiへ正規化する。 */
private fun wrapHeadingAngle(angleRad: Double): Double {
    var wrapped = angleRad
    while (wrapped > kotlin.math.PI) wrapped -= 2.0 * kotlin.math.PI
    while (wrapped < -kotlin.math.PI) wrapped += 2.0 * kotlin.math.PI
    return wrapped
}

private operator fun BodyVector.minus(other: BodyVector) = BodyVector(
    x = x - other.x,
    y = y - other.y,
    z = z - other.z
)
