package com.example.gnss_imu_logger.attitude

/**
 * 計測開始前に選択する端末取付方向。
 * 入力: 画面法線と端末辺の車体方向
 * 出力: 端末座標から車体座標への直交変換
 */
enum class DeviceMount(
    val displayName: String,
    internal val forwardDevice: BodyVector,
    internal val upDevice: BodyVector
) {
    SCREEN_UP_TOP_FORWARD("画面上・端末上が前", bodyAxis(0, 1, 0), bodyAxis(0, 0, 1)),
    SCREEN_UP_BOTTOM_FORWARD("画面上・端末下が前", bodyAxis(0, -1, 0), bodyAxis(0, 0, 1)),
    SCREEN_UP_RIGHT_FORWARD("画面上・端末右が前", bodyAxis(1, 0, 0), bodyAxis(0, 0, 1)),
    SCREEN_UP_LEFT_FORWARD("画面上・端末左が前", bodyAxis(-1, 0, 0), bodyAxis(0, 0, 1)),
    SCREEN_DOWN_TOP_FORWARD("画面下・端末上が前", bodyAxis(0, 1, 0), bodyAxis(0, 0, -1)),
    SCREEN_DOWN_BOTTOM_FORWARD("画面下・端末下が前", bodyAxis(0, -1, 0), bodyAxis(0, 0, -1)),
    SCREEN_DOWN_RIGHT_FORWARD("画面下・端末右が前", bodyAxis(1, 0, 0), bodyAxis(0, 0, -1)),
    SCREEN_DOWN_LEFT_FORWARD("画面下・端末左が前", bodyAxis(-1, 0, 0), bodyAxis(0, 0, -1)),
    SCREEN_FORWARD_TOP_UP("画面が前・端末上が上", bodyAxis(0, 0, 1), bodyAxis(0, 1, 0)),
    SCREEN_REAR_TOP_UP("画面が後・端末上が上", bodyAxis(0, 0, -1), bodyAxis(0, 1, 0)),
    SCREEN_RIGHT_TOP_UP("画面が右・端末上が上", bodyAxis(1, 0, 0), bodyAxis(0, 1, 0)),
    SCREEN_LEFT_TOP_UP("画面が左・端末上が上", bodyAxis(-1, 0, 0), bodyAxis(0, 1, 0));

    fun transform(): DeviceMountTransform = DeviceMountTransform.from(this)

    companion object {
        val DEFAULT = SCREEN_UP_TOP_FORWARD

        fun fromName(value: String?): DeviceMount =
            entries.firstOrNull { it.name == value } ?: DEFAULT
    }
}

/** enum定数の初期化中でも使用できる端末座標軸を生成する。 */
private fun bodyAxis(x: Int, y: Int, z: Int): BodyVector = BodyVector(
    x = x.toDouble(),
    y = y.toDouble(),
    z = z.toDouble()
)
