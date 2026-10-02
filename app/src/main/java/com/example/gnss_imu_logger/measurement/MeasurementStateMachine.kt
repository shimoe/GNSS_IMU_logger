package com.example.gnss_imu_logger.measurement

import com.example.gnss_imu_logger.log.SessionContext
import com.example.gnss_imu_logger.model.EventLevel
import com.example.gnss_imu_logger.model.EventType

/**
 * 計測状態と許可する状態遷移を管理する。
 * 入力: 遷移先状態、遷移理由
 * 出力: 現在状態、events.csvの状態遷移記録
 */
class MeasurementStateMachine(
    private val session: SessionContext,
    initialState: MeasurementState = MeasurementState.IDLE,
    private val onStateChanged: (MeasurementState) -> Unit = {}
) {
    @Volatile
    var state: MeasurementState = initialState
        private set

    @Synchronized
    fun transitionTo(next: MeasurementState, reason: String): Boolean {
        if (state == next) return true
        if (next !in allowedTransitions.getValue(state)) {
            session.event(
                EventLevel.WARNING,
                EventType.STATE_TRANSITION_REJECTED,
                "${state.name}から${next.name}への状態変更を拒否しました: $reason"
            )
            return false
        }
        val previous = state
        state = next
        session.event(
            EventLevel.INFO,
            eventFor(next),
            "${previous.name}から${next.name}へ変更しました: $reason"
        )
        onStateChanged(next)
        return true
    }

    private fun eventFor(state: MeasurementState): EventType = when (state) {
        MeasurementState.INITIALIZING -> EventType.MEASUREMENT_STARTED
        MeasurementState.WAITING_FOR_GNSS -> EventType.GNSS_WAIT_STARTED
        MeasurementState.READY -> EventType.READY_ENTERED
        MeasurementState.DEGRADED -> EventType.GNSS_DEGRADED
        MeasurementState.STOPPING -> EventType.STOP_REQUESTED
        MeasurementState.ERROR -> EventType.MEASUREMENT_ERROR
        MeasurementState.IDLE -> EventType.MEASUREMENT_COMPLETED
    }

    companion object {
        private val allowedTransitions = mapOf(
            MeasurementState.IDLE to setOf(MeasurementState.INITIALIZING),
            MeasurementState.INITIALIZING to setOf(
                MeasurementState.WAITING_FOR_GNSS,
                MeasurementState.STOPPING,
                MeasurementState.ERROR
            ),
            MeasurementState.WAITING_FOR_GNSS to setOf(
                MeasurementState.READY,
                MeasurementState.STOPPING,
                MeasurementState.ERROR
            ),
            MeasurementState.READY to setOf(
                MeasurementState.DEGRADED,
                MeasurementState.STOPPING,
                MeasurementState.ERROR
            ),
            MeasurementState.DEGRADED to setOf(
                MeasurementState.READY,
                MeasurementState.STOPPING,
                MeasurementState.ERROR
            ),
            MeasurementState.STOPPING to setOf(
                MeasurementState.IDLE,
                MeasurementState.ERROR
            ),
            MeasurementState.ERROR to setOf(MeasurementState.STOPPING)
        )
    }
}
