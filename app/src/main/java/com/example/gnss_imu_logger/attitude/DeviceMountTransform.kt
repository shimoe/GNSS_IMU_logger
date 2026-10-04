package com.example.gnss_imu_logger.attitude

/**
 * 端末座標のセンサー値を車体座標へ変換する。
 * 車体座標: X前方、Y左方、Z上方
 * 入力: Android端末座標X/Y/Z
 * 出力: 車体座標X/Y/Z
 */
class DeviceMountTransform private constructor(
    private val xx: Double,
    private val xy: Double,
    private val xz: Double,
    private val yx: Double,
    private val yy: Double,
    private val yz: Double,
    private val zx: Double,
    private val zy: Double,
    private val zz: Double
) {
    fun deviceToBody(values: FloatArray): BodyVector = deviceToBody(
        BodyVector(
            x = values.getOrElse(0) { 0f }.toDouble(),
            y = values.getOrElse(1) { 0f }.toDouble(),
            z = values.getOrElse(2) { 0f }.toDouble()
        )
    )

    fun deviceToBody(value: BodyVector): BodyVector = BodyVector(
        x = xx * value.x + xy * value.y + xz * value.z,
        y = yx * value.x + yy * value.y + yz * value.z,
        z = zx * value.x + zy * value.y + zz * value.z
    )

    companion object {
        /** 画面上向き、端末上側が機体前方、端末右側が機体右方。 */
        val SCREEN_UP_TOP_FORWARD = DeviceMountTransform(
            xx = 0.0, xy = 1.0, xz = 0.0,
            yx = -1.0, yy = 0.0, yz = 0.0,
            zx = 0.0, zy = 0.0, zz = 1.0
        )

        /** 画面上向き、端末上側が機体後方。 */
        val SCREEN_UP_TOP_REAR = DeviceMountTransform(
            xx = 0.0, xy = -1.0, xz = 0.0,
            yx = 1.0, yy = 0.0, yz = 0.0,
            zx = 0.0, zy = 0.0, zz = 1.0
        )

        /** 画面上向き、端末右側が機体前方。 */
        val SCREEN_UP_RIGHT_FORWARD = DeviceMountTransform(
            xx = 1.0, xy = 0.0, xz = 0.0,
            yx = 0.0, yy = 1.0, yz = 0.0,
            zx = 0.0, zy = 0.0, zz = 1.0
        )

        /** 画面上向き、端末左側が機体前方。 */
        val SCREEN_UP_LEFT_FORWARD = DeviceMountTransform(
            xx = -1.0, xy = 0.0, xz = 0.0,
            yx = 0.0, yy = -1.0, yz = 0.0,
            zx = 0.0, zy = 0.0, zz = 1.0
        )
    }
}
