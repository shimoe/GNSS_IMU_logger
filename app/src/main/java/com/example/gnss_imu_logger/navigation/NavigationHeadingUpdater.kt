package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import kotlin.math.abs
import kotlin.math.max

/**
 * GNSS進行方位を使って車体からENUへのヨー角を補正する。
 * 入力: 予測姿勢、ヨー分散[rad^2]、GNSS方位[rad]、方位精度[rad]
 * 出力: ロール・ピッチを維持した補正姿勢、補正後分散、診断値
 *
 * GNSS進行方位は低速時に不安定なため、NavigationInputPreprocessorで
 * bearingUsable=trueとなった観測だけが本処理へ渡される。
 */
class NavigationHeadingUpdater(
    private val parameters: NavigationHeadingParameters = NavigationHeadingParameters.DEFAULT
) {
    fun update(
        attitudeBodyToEnu: AttitudeQuaternion,
        headingVarianceRad2: Double,
        measurement: NavigationGnssUpdate
    ): NavigationHeadingUpdateResult {
        val measuredHeadingRad = measurement.bearingRad
        val accuracyRad = measurement.bearingAccuracyRad
        if (measuredHeadingRad == null || accuracyRad == null || !accuracyRad.isFinite()) {
            return unavailable(attitudeBodyToEnu, headingVarianceRad2)
        }
        val euler = attitudeBodyToEnu.toEulerAngles()
        val innovationRad = wrapAngle(measuredHeadingRad - euler.yawRad)
        val observationVarianceRad2 = max(
            accuracyRad,
            parameters.minimumBearingAccuracyRad
        ).let { it * it }
        val gateRad = max(
            parameters.minimumHeadingGateRad,
            parameters.innovationSigmaGate *
                kotlin.math.sqrt(headingVarianceRad2 + observationVarianceRad2)
        )
        if (abs(innovationRad) > gateRad) {
            return NavigationHeadingUpdateResult(
                attitudeBodyToEnu = attitudeBodyToEnu,
                headingVarianceRad2 = headingVarianceRad2,
                available = true,
                accepted = false,
                innovationRad = innovationRad,
                gateRad = gateRad,
                gain = null,
                correctionRad = null
            )
        }
        val gain = headingVarianceRad2 /
            (headingVarianceRad2 + observationVarianceRad2)
        val correctionRad = (gain * innovationRad).coerceIn(
            -parameters.maximumCorrectionRad,
            parameters.maximumCorrectionRad
        )
        val correctedAttitude = AttitudeQuaternion.fromEulerAngles(
            rollRad = euler.rollRad,
            pitchRad = euler.pitchRad,
            yawRad = wrapAngle(euler.yawRad + correctionRad)
        )
        return NavigationHeadingUpdateResult(
            attitudeBodyToEnu = correctedAttitude,
            headingVarianceRad2 = max(
                parameters.minimumVarianceRad2,
                (1.0 - gain) * headingVarianceRad2
            ),
            available = true,
            accepted = true,
            innovationRad = innovationRad,
            gateRad = gateRad,
            gain = gain,
            correctionRad = correctionRad
        )
    }

    fun predictVariance(
        headingVarianceRad2: Double,
        dtSec: Double
    ): Double = headingVarianceRad2 +
        parameters.headingProcessNoiseRad2PerSec * dtSec

    private fun unavailable(
        attitudeBodyToEnu: AttitudeQuaternion,
        headingVarianceRad2: Double
    ) = NavigationHeadingUpdateResult(
        attitudeBodyToEnu = attitudeBodyToEnu,
        headingVarianceRad2 = headingVarianceRad2,
        available = false,
        accepted = false,
        innovationRad = null,
        gateRad = null,
        gain = null,
        correctionRad = null
    )

    private fun wrapAngle(angleRad: Double): Double {
        var value = angleRad
        while (value > Math.PI) value -= TWO_PI
        while (value < -Math.PI) value += TWO_PI
        return value
    }

    companion object {
        private const val TWO_PI = 2.0 * Math.PI
    }
}

data class NavigationHeadingUpdateResult(
    val attitudeBodyToEnu: AttitudeQuaternion,
    val headingVarianceRad2: Double,
    val available: Boolean,
    val accepted: Boolean,
    val innovationRad: Double?,
    val gateRad: Double?,
    val gain: Double?,
    val correctionRad: Double?
)

data class NavigationHeadingParameters(
    val innovationSigmaGate: Double,
    val minimumHeadingGateRad: Double,
    val minimumBearingAccuracyRad: Double,
    val maximumCorrectionRad: Double,
    val minimumVarianceRad2: Double,
    val initialHeadingVarianceRad2: Double,
    val headingProcessNoiseRad2PerSec: Double
) {
    companion object {
        val DEFAULT = NavigationHeadingParameters(
            innovationSigmaGate = 6.0,
            minimumHeadingGateRad = Math.toRadians(45.0),
            minimumBearingAccuracyRad = Math.toRadians(2.0),
            maximumCorrectionRad = Math.toRadians(10.0),
            minimumVarianceRad2 = Math.toRadians(0.5).let { it * it },
            initialHeadingVarianceRad2 = Math.toRadians(30.0).let { it * it },
            headingProcessNoiseRad2PerSec = Math.toRadians(1.0).let { it * it }
        )
    }
}
