package com.example.gnss_imu_logger.ui

import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import com.example.gnss_imu_logger.measurement.MeasurementSnapshot
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.ui.measurement.MeasurementScreen
import com.example.gnss_imu_logger.ui.playback.PlaybackScreen
import java.io.File

enum class AppScreen {
    MEASUREMENT,
    PLAYBACK
}

@Composable
fun AppContent(
    snapshot: MeasurementSnapshot,
    mode: LogMode,
    selectedScreen: AppScreen,
    documentsDirectory: File,
    onScreenChanged: (AppScreen) -> Unit,
    onModeChanged: (LogMode) -> Unit,
    onStart: () -> Unit,
    onStop: () -> Unit
) {
    Scaffold(
        topBar = {
            TabRow(selectedTabIndex = selectedScreen.ordinal) {
                Tab(
                    selected = selectedScreen == AppScreen.MEASUREMENT,
                    onClick = { onScreenChanged(AppScreen.MEASUREMENT) },
                    text = { Text("計測") }
                )
                Tab(
                    selected = selectedScreen == AppScreen.PLAYBACK,
                    onClick = { onScreenChanged(AppScreen.PLAYBACK) },
                    text = { Text("再生") }
                )
            }
        }
    ) { padding ->
        when (selectedScreen) {
            AppScreen.MEASUREMENT -> MeasurementScreen(
                snapshot = snapshot,
                mode = mode,
                onModeChanged = onModeChanged,
                onStart = onStart,
                onStop = onStop,
                modifier = Modifier.padding(padding)
            )

            AppScreen.PLAYBACK -> PlaybackScreen(
                documentsDirectory = documentsDirectory,
                modifier = Modifier.padding(padding)
            )
        }
    }
}

