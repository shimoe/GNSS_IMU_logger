package com.example.gnss_imu_logger.navigation

import kotlin.math.abs

/**
 * 15状態ESKFの誤差状態からyawだけを公称状態へ安全に注入する。
 * 入力: GNSS更新後の公称状態、ESKF誤差状態、水平速度[m/s]、yaw角速度[rad/s]
 * 出力: yaw補正後状態、制限結果、診断値
 *
 * Gravityで改善済みのroll・pitchと未検証のIMUバイアスは変更しない。
 */
class NavigationEskfStateInjector(
    private val parameters: NavigationEskfInjectionParameters =
        NavigationEskfInjectionParameters.DEFAULT
) {
    fun injectYaw(
        state: NavigationState,
        errorState: DoubleArray,
        horizontalSpeedMps: Double,
        yawRateRadps: Double?,
        valid: Boolean
    ): NavigationEskfInjectionResult {
        if (!valid || errorState.size != NavigationErrorStateIndex.SIZE) {
            return fallback(state, horizontalSpeedMps, yawRateRadps, "INVALID_ESKF_RESULT")
        }
        val rawYawCorrectionRad = errorState[NavigationErrorStateIndex.ATTITUDE + 2]
        if (!rawYawCorrectionRad.isFinite()) {
            return fallback(state, horizontalSpeedMps, yawRateRadps, "INVALID_YAW_CORRECTION")
        }
        if (horizontalSpeedMps < parameters.minimumYawInjectionSpeedMps) {
            return NavigationEskfInjectionResult(
                state = state,
                rawYawCorrectionRad = rawYawCorrectionRad,
                appliedYawCorrectionRad = 0.0,
                horizontalSpeedMps = horizontalSpeedMps,
                yawRateRadps = yawRateRadps,
                turningUpdate = false,
                yawLimited = false,
                applied = false,
                fallbackReason = "LOW_SPEED"
            )
        }
        val turning = yawRateRadps?.let { abs(it) >= parameters.turningYawRateRadps } == true
        val limit = if (turning) parameters.maximumTurningYawCorrectionRad
        else parameters.maximumNormalYawCorrectionRad
        val appliedCorrection = rawYawCorrectionRad.coerceIn(-limit, limit)
        val euler = state.attitudeBodyToEnu.toEulerAngles()
        val corrected = com.example.gnss_imu_logger.attitude.AttitudeQuaternion.fromEulerAngles(
            rollRad = euler.rollRad,
            pitchRad = euler.pitchRad,
            yawRad = wrapAngle(euler.yawRad + appliedCorrection)
        )
        return NavigationEskfInjectionResult(
            state = state.copy(attitudeBodyToEnu = corrected),
            rawYawCorrectionRad = rawYawCorrectionRad,
            appliedYawCorrectionRad = appliedCorrection,
            horizontalSpeedMps = horizontalSpeedMps,
            yawRateRadps = yawRateRadps,
            turningUpdate = turning,
            yawLimited = abs(appliedCorrection - rawYawCorrectionRad) > 1e-12,
            applied = abs(appliedCorrection) > 1e-12,
            fallbackReason = ""
        )
    }

    private fun fallback(
        state: NavigationState,
        speedMps: Double,
        yawRateRadps: Double?,
        reason: String
    ) = NavigationEskfInjectionResult(
        state = state,
        rawYawCorrectionRad = null,
        appliedYawCorrectionRad = 0.0,
        horizontalSpeedMps = speedMps,
        yawRateRadps = yawRateRadps,
        turningUpdate = false,
        yawLimited = false,
        applied = false,
        fallbackReason = reason
    )

    private fun wrapAngle(value: Double): Double = kotlin.math.atan2(
        kotlin.math.sin(value),
        kotlin.math.cos(value)
    )
}

data class NavigationEskfInjectionParameters(
    val minimumYawInjectionSpeedMps: Double,
    val turningYawRateRadps: Double,
    val maximumNormalYawCorrectionRad: Double,
    val maximumTurningYawCorrectionRad: Double
) {
    companion object {
        val DEFAULT = NavigationEskfInjectionParameters(
            minimumYawInjectionSpeedMps = 3.0,
            turningYawRateRadps = 0.15,
            maximumNormalYawCorrectionRad = Math.toRadians(1.5),
            maximumTurningYawCorrectionRad = Math.toRadians(3.0)
        )
    }
}

data class NavigationEskfInjectionResult(
    val state: NavigationState,
    val rawYawCorrectionRad: Double?,
    val appliedYawCorrectionRad: Double,
    val horizontalSpeedMps: Double,
    val yawRateRadps: Double?,
    val turningUpdate: Boolean,
    val yawLimited: Boolean,
    val applied: Boolean,
    val fallbackReason: String
)
