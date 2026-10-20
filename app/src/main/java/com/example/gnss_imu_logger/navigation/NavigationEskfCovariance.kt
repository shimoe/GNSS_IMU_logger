package com.example.gnss_imu_logger.navigation

import com.example.gnss_imu_logger.attitude.AttitudeQuaternion
import com.example.gnss_imu_logger.attitude.BodyVector
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

/**
 * 15状態ESKFの誤差状態インデックス。
 * 入力: なし
 * 出力: 15x15共分散行列で使用する開始位置
 */
object NavigationErrorStateIndex {
    const val POSITION = 0
    const val VELOCITY = 3
    const val ATTITUDE = 6
    const val ACCELEROMETER_BIAS = 9
    const val GYROSCOPE_BIAS = 12
    const val SIZE = 15
}

/**
 * 15状態ESKF共分散予測の調整値。
 * 入力: IMU白色雑音とバイアスランダムウォークの標準偏差
 * 出力: 離散プロセス雑音の生成に使用する値
 *
 * 現段階はオフライン比較用であり、GNSS観測更新や公称状態への誤差注入には使用しない。
 */
data class NavigationEskfNoiseParameters(
    val accelerometerNoiseStdMps2: Double,
    val gyroscopeNoiseStdRadps: Double,
    val accelerometerBiasRandomWalkStdMps3: Double,
    val gyroscopeBiasRandomWalkStdRadps2: Double,
    val initialPositionStdM: Double,
    val initialVelocityStdMps: Double,
    val initialAttitudeStdRad: Double,
    val initialAccelerometerBiasStdMps2: Double,
    val initialGyroscopeBiasStdRadps: Double
) {
    companion object {
        val DEFAULT = NavigationEskfNoiseParameters(
            accelerometerNoiseStdMps2 = 0.35,
            gyroscopeNoiseStdRadps = 0.02,
            accelerometerBiasRandomWalkStdMps3 = 0.02,
            gyroscopeBiasRandomWalkStdRadps2 = 0.001,
            initialPositionStdM = 5.0,
            initialVelocityStdMps = 2.0,
            initialAttitudeStdRad = 0.17,
            initialAccelerometerBiasStdMps2 = 0.20,
            initialGyroscopeBiasStdRadps = 0.02
        )
    }
}

/**
 * 15状態ESKFの15x15共分散。
 * 入力: 行優先の225要素
 * 出力: IMU予測後の共分散と診断値
 *
 * 状態順は位置、速度、姿勢誤差、加速度バイアス、ジャイロバイアスである。
 * GNSS観測更新をまだ適用せず、既存の軸別分散と並行して予測だけを評価する。
 */
class NavigationEskfCovariance private constructor(
    private val values: DoubleArray
) {
    init {
        require(values.size == ELEMENT_COUNT) { "15状態共分散は225要素である必要があります。" }
    }

    operator fun get(row: Int, column: Int): Double = values[index(row, column)]

    /**
     * IMU入力から共分散を1周期予測する。
     * 入力: 時間差[s]、車体からENUへの姿勢、補正済み加速度[m/s^2]、補正済み角速度[rad/s]
     * 出力: FPF^T+Qで予測し対称化した15x15共分散
     */
    fun predict(
        dtSec: Double,
        attitudeBodyToEnu: AttitudeQuaternion,
        accelerationBodyMps2: BodyVector,
        angularRateBodyRadps: BodyVector,
        parameters: NavigationEskfNoiseParameters
    ): NavigationEskfCovariance {
        require(dtSec > 0.0 && dtSec.isFinite()) { "共分散予測の時間差が不正です: $dtSec s" }
        val transition = identityMatrix()
        setIdentityCoupling(transition, NavigationErrorStateIndex.POSITION, NavigationErrorStateIndex.VELOCITY, dtSec)

        val rotation = rotationMatrix(attitudeBodyToEnu)
        val accelerationSkew = skew(accelerationBodyMps2)
        val attitudeCoupling = multiply3(rotation, accelerationSkew)
        setBlock(transition, NavigationErrorStateIndex.VELOCITY, NavigationErrorStateIndex.ATTITUDE, attitudeCoupling, -dtSec)
        setBlock(transition, NavigationErrorStateIndex.VELOCITY, NavigationErrorStateIndex.ACCELEROMETER_BIAS, rotation, -dtSec)
        setBlock(transition, NavigationErrorStateIndex.ATTITUDE, NavigationErrorStateIndex.ATTITUDE, skew(angularRateBodyRadps), -dtSec, add = true)
        setIdentityCoupling(transition, NavigationErrorStateIndex.ATTITUDE, NavigationErrorStateIndex.GYROSCOPE_BIAS, -dtSec)

        val predicted = multiply(multiply(transition, values), transpose(transition))
        addProcessNoise(predicted, dtSec, parameters)
        return NavigationEskfCovariance(symmetrize(predicted))
    }


    /**
     * 採用済みGNSS位置・速度観測で共分散のみを更新する。
     * 入力: GNSS観測標準偏差、既存推定器の位置・速度採用結果
     * 出力: スカラーJoseph形式を各観測軸へ順次適用した15x15共分散
     *
     * 公称状態とIMUバイアスは変更しない。既存推定経路を維持したまま、
     * GNSS観測による15状態共分散の収束だけを評価する段階の実装である。
     */
    fun updateGnss(
        measurement: NavigationGnssUpdate,
        positionAccepted: Boolean,
        velocityAccepted: Boolean
    ): NavigationEskfCovariance {
        var updated = values.copyOf()
        if (positionAccepted) {
            val horizontalVariance = measurement.positionStdM * measurement.positionStdM
            val verticalVariance = measurement.verticalStdM * measurement.verticalStdM
            updated = scalarJosephUpdate(updated, NavigationErrorStateIndex.POSITION, horizontalVariance)
            updated = scalarJosephUpdate(updated, NavigationErrorStateIndex.POSITION + 1, horizontalVariance)
            updated = scalarJosephUpdate(updated, NavigationErrorStateIndex.POSITION + 2, verticalVariance)
        }
        if (velocityAccepted && measurement.velocityEnuMps != null) {
            val velocityVariance = measurement.speedStdMps * measurement.speedStdMps
            repeat(3) { axis ->
                updated = scalarJosephUpdate(
                    updated,
                    NavigationErrorStateIndex.VELOCITY + axis,
                    velocityVariance
                )
            }
        }
        return NavigationEskfCovariance(symmetrize(updated))
    }

    fun standardDeviation(stateIndex: Int): Double =
        sqrt(this[stateIndex, stateIndex].coerceAtLeast(0.0))

    fun maximumAsymmetry(): Double {
        var maximum = 0.0
        for (row in 0 until STATE_SIZE) {
            for (column in row + 1 until STATE_SIZE) {
                maximum = max(maximum, abs(this[row, column] - this[column, row]))
            }
        }
        return maximum
    }

    fun minimumDiagonal(): Double =
        (0 until STATE_SIZE).minOf { this[it, it] }

    companion object {
        private const val STATE_SIZE = NavigationErrorStateIndex.SIZE
        private const val ELEMENT_COUNT = STATE_SIZE * STATE_SIZE

        fun initial(parameters: NavigationEskfNoiseParameters): NavigationEskfCovariance {
            val values = DoubleArray(ELEMENT_COUNT)
            setDiagonal(values, NavigationErrorStateIndex.POSITION, parameters.initialPositionStdM)
            setDiagonal(values, NavigationErrorStateIndex.VELOCITY, parameters.initialVelocityStdMps)
            setDiagonal(values, NavigationErrorStateIndex.ATTITUDE, parameters.initialAttitudeStdRad)
            setDiagonal(values, NavigationErrorStateIndex.ACCELEROMETER_BIAS, parameters.initialAccelerometerBiasStdMps2)
            setDiagonal(values, NavigationErrorStateIndex.GYROSCOPE_BIAS, parameters.initialGyroscopeBiasStdRadps)
            return NavigationEskfCovariance(values)
        }

        private fun setDiagonal(values: DoubleArray, start: Int, standardDeviation: Double) {
            val variance = standardDeviation * standardDeviation
            repeat(3) { axis -> values[index(start + axis, start + axis)] = variance }
        }

        private fun addProcessNoise(
            values: DoubleArray,
            dtSec: Double,
            parameters: NavigationEskfNoiseParameters
        ) {
            val accelerationVariance = parameters.accelerometerNoiseStdMps2 * parameters.accelerometerNoiseStdMps2
            val gyroVariance = parameters.gyroscopeNoiseStdRadps * parameters.gyroscopeNoiseStdRadps
            val accelerometerBiasVariance = parameters.accelerometerBiasRandomWalkStdMps3 * parameters.accelerometerBiasRandomWalkStdMps3
            val gyroscopeBiasVariance = parameters.gyroscopeBiasRandomWalkStdRadps2 * parameters.gyroscopeBiasRandomWalkStdRadps2
            val positionNoise = accelerationVariance * dtSec * dtSec * dtSec / 3.0
            val positionVelocityNoise = accelerationVariance * dtSec * dtSec / 2.0
            val velocityNoise = accelerationVariance * dtSec
            repeat(3) { axis ->
                val position = NavigationErrorStateIndex.POSITION + axis
                val velocity = NavigationErrorStateIndex.VELOCITY + axis
                values[index(position, position)] += positionNoise
                values[index(position, velocity)] += positionVelocityNoise
                values[index(velocity, position)] += positionVelocityNoise
                values[index(velocity, velocity)] += velocityNoise
                values[index(NavigationErrorStateIndex.ATTITUDE + axis, NavigationErrorStateIndex.ATTITUDE + axis)] += gyroVariance * dtSec
                values[index(NavigationErrorStateIndex.ACCELEROMETER_BIAS + axis, NavigationErrorStateIndex.ACCELEROMETER_BIAS + axis)] += accelerometerBiasVariance * dtSec
                values[index(NavigationErrorStateIndex.GYROSCOPE_BIAS + axis, NavigationErrorStateIndex.GYROSCOPE_BIAS + axis)] += gyroscopeBiasVariance * dtSec
            }
        }


        /** Hが単一状態を直接観測する場合のJoseph形式共分散更新。 */
        private fun scalarJosephUpdate(
            covariance: DoubleArray,
            observedStateIndex: Int,
            measurementVariance: Double
        ): DoubleArray {
            if (!measurementVariance.isFinite() || measurementVariance <= 0.0) return covariance
            val innovationVariance =
                covariance[index(observedStateIndex, observedStateIndex)] + measurementVariance
            if (!innovationVariance.isFinite() || innovationVariance <= 0.0) return covariance
            val gain = DoubleArray(STATE_SIZE) { row ->
                covariance[index(row, observedStateIndex)] / innovationVariance
            }
            val identityMinusGainH = identityMatrix()
            repeat(STATE_SIZE) { row ->
                identityMinusGainH[index(row, observedStateIndex)] -= gain[row]
            }
            val joseph = multiply(
                multiply(identityMinusGainH, covariance),
                transpose(identityMinusGainH)
            )
            for (row in 0 until STATE_SIZE) for (column in 0 until STATE_SIZE) {
                joseph[index(row, column)] +=
                    gain[row] * measurementVariance * gain[column]
            }
            return joseph
        }

        private fun rotationMatrix(attitude: AttitudeQuaternion): DoubleArray {
            val xAxis = attitude.rotate(BodyVector(1.0, 0.0, 0.0))
            val yAxis = attitude.rotate(BodyVector(0.0, 1.0, 0.0))
            val zAxis = attitude.rotate(BodyVector(0.0, 0.0, 1.0))
            return doubleArrayOf(
                xAxis.x, yAxis.x, zAxis.x,
                xAxis.y, yAxis.y, zAxis.y,
                xAxis.z, yAxis.z, zAxis.z
            )
        }

        private fun skew(vector: BodyVector) = doubleArrayOf(
            0.0, -vector.z, vector.y,
            vector.z, 0.0, -vector.x,
            -vector.y, vector.x, 0.0
        )

        private fun multiply3(left: DoubleArray, right: DoubleArray): DoubleArray {
            val result = DoubleArray(9)
            for (row in 0 until 3) for (column in 0 until 3) for (inner in 0 until 3) {
                result[row * 3 + column] += left[row * 3 + inner] * right[inner * 3 + column]
            }
            return result
        }

        private fun identityMatrix(): DoubleArray = DoubleArray(ELEMENT_COUNT).also { matrix ->
            repeat(STATE_SIZE) { matrix[index(it, it)] = 1.0 }
        }

        private fun setIdentityCoupling(matrix: DoubleArray, rowStart: Int, columnStart: Int, scale: Double) {
            repeat(3) { axis -> matrix[index(rowStart + axis, columnStart + axis)] += scale }
        }

        private fun setBlock(matrix: DoubleArray, rowStart: Int, columnStart: Int, block: DoubleArray, scale: Double, add: Boolean = false) {
            for (row in 0 until 3) for (column in 0 until 3) {
                val matrixIndex = index(rowStart + row, columnStart + column)
                val value = block[row * 3 + column] * scale
                if (add) matrix[matrixIndex] += value else matrix[matrixIndex] = value
            }
        }

        private fun multiply(left: DoubleArray, right: DoubleArray): DoubleArray {
            val result = DoubleArray(ELEMENT_COUNT)
            for (row in 0 until STATE_SIZE) for (inner in 0 until STATE_SIZE) {
                val leftValue = left[index(row, inner)]
                if (leftValue == 0.0) continue
                for (column in 0 until STATE_SIZE) {
                    result[index(row, column)] += leftValue * right[index(inner, column)]
                }
            }
            return result
        }

        private fun transpose(matrix: DoubleArray): DoubleArray = DoubleArray(ELEMENT_COUNT).also { result ->
            for (row in 0 until STATE_SIZE) for (column in 0 until STATE_SIZE) {
                result[index(column, row)] = matrix[index(row, column)]
            }
        }

        private fun symmetrize(matrix: DoubleArray): DoubleArray {
            for (row in 0 until STATE_SIZE) for (column in row + 1 until STATE_SIZE) {
                val average = 0.5 * (matrix[index(row, column)] + matrix[index(column, row)])
                matrix[index(row, column)] = average
                matrix[index(column, row)] = average
            }
            return matrix
        }

        private fun index(row: Int, column: Int): Int = row * STATE_SIZE + column
    }
}
