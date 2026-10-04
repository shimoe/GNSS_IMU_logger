package com.example.gnss_imu_logger.attitude

/**
 * 端末座標のセンサー値を車体座標へ変換する。
 * 車体座標: X前方、Y左方、Z上方
 * 入力: Android端末座標X/Y/Z
 * 出力: 車体座標X/Y/Z
 */
class DeviceMountTransform private constructor(
    private val forwardDevice: BodyVector,
    private val leftDevice: BodyVector,
    private val upDevice: BodyVector
) {
    fun deviceToBody(values: FloatArray): BodyVector = deviceToBody(
        BodyVector(
            values.getOrElse(0) { 0f }.toDouble(),
            values.getOrElse(1) { 0f }.toDouble(),
            values.getOrElse(2) { 0f }.toDouble()
        )
    )

    fun deviceToBody(value: BodyVector): BodyVector = BodyVector(
        x = forwardDevice.dot(value),
        y = leftDevice.dot(value),
        z = upDevice.dot(value)
    )

    companion object {
        fun from(mount: DeviceMount): DeviceMountTransform {
            val forward = mount.forwardDevice
            val up = mount.upDevice
            // 車体Y左方向は、車体上方向と車体前方向の外積で求める。
            val left = up.cross(forward)
            return DeviceMountTransform(forward, left, up)
        }

        val SCREEN_UP_TOP_FORWARD = from(DeviceMount.SCREEN_UP_TOP_FORWARD)
    }
}

private fun BodyVector.dot(other: BodyVector): Double =
    x * other.x + y * other.y + z * other.z

private fun BodyVector.cross(other: BodyVector): BodyVector = BodyVector(
    x = y * other.z - z * other.y,
    y = z * other.x - x * other.z,
    z = x * other.y - y * other.x
)
