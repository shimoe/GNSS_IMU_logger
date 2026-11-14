package com.example.gnss_imu_logger.ui.playback

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.example.gnss_imu_logger.navigation.GeodeticPosition
import com.example.gnss_imu_logger.navigation.LapCrossingDirection
import com.example.gnss_imu_logger.navigation.LapDetectionConfig

enum class LapLineEditState { IDLE, WAITING_FOR_START_POINT, WAITING_FOR_END_POINT, READY_TO_SAVE }

data class LapLineDraft(
    val start: GeodeticPosition? = null,
    val end: GeodeticPosition? = null,
    val direction: LapCrossingDirection = LapCrossingDirection.LEFT_TO_RIGHT
) {
    fun toConfig(): LapDetectionConfig? {
        val startValue = start ?: return null
        val endValue = end ?: return null
        return LapDetectionConfig(startValue, endValue, direction)
    }
}

@Composable
fun LapLineEditor(
    state: LapLineEditState,
    draft: LapLineDraft,
    saved: LapDetectionConfig?,
    busy: Boolean,
    message: String?,
    onStart: () -> Unit,
    onSwapDirection: () -> Unit,
    onSave: () -> Unit,
    onCancel: () -> Unit,
    onDelete: () -> Unit
) {
    Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
        Text("スタートライン")
        Text(when (state) {
            LapLineEditState.IDLE -> if (saved == null) "未設定" else "設定済み"
            LapLineEditState.WAITING_FOR_START_POINT -> "地図上で始点を選択してください。"
            LapLineEditState.WAITING_FOR_END_POINT -> "地図上で終点を選択してください。"
            LapLineEditState.READY_TO_SAVE -> "ラインと通過方向を確認して保存してください。"
        })
        draft.start?.let { Text("始点: %.7f, %.7f".format(it.latitudeDeg, it.longitudeDeg)) }
        draft.end?.let { Text("終点: %.7f, %.7f".format(it.latitudeDeg, it.longitudeDeg)) }
        Text(if (draft.direction == LapCrossingDirection.LEFT_TO_RIGHT)
            "方向: 始点から終点を見て左側から右側"
        else "方向: 始点から終点を見て右側から左側")
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            if (state == LapLineEditState.IDLE) {
                Button(onClick = onStart, enabled = !busy) { Text(if (saved == null) "ライン設定" else "再設定") }
                if (saved != null) Button(onClick = onDelete, enabled = !busy) { Text("削除") }
            } else {
                Button(onClick = onSwapDirection, enabled = !busy) { Text("方向反転") }
                Button(onClick = onCancel, enabled = !busy) { Text("キャンセル") }
                if (state == LapLineEditState.READY_TO_SAVE) {
                    Button(onClick = onSave, enabled = !busy) { Text("保存して解析") }
                }
            }
        }
        message?.let { Text(it) }
    }
}
