package com.example.gnss_imu_logger

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import com.example.gnss_imu_logger.ui.theme.GNSS_IMU_loggerTheme

class MainActivity : ComponentActivity() {
    private var status by mutableStateOf("停止中")

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        if (result[Manifest.permission.ACCESS_FINE_LOCATION] == true) startLogger()
        else status = "正確な位置情報の許可が必要です"
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            GNSS_IMU_loggerTheme {
                Surface(modifier = Modifier.fillMaxSize()) {
                    LoggerScreen(status, ::requestStart, ::stopLogger)
                }
            }
        }
    }

    private fun requestStart() {
        val permissions = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions += Manifest.permission.POST_NOTIFICATIONS
        }
        if (permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }) {
            startLogger()
        } else {
            permissionLauncher.launch(permissions.toTypedArray())
        }
    }

    private fun startLogger() {
        val intent = Intent(this, LoggerService::class.java).setAction(LoggerService.ACTION_START)
        ContextCompat.startForegroundService(this, intent)
        status = "計測中。画面を消しても記録を継続します"
    }

    private fun stopLogger() {
        startService(Intent(this, LoggerService::class.java).setAction(LoggerService.ACTION_STOP))
        status = "停止しました"
    }
}

@Composable
private fun LoggerScreen(status: String, onStart: () -> Unit, onStop: () -> Unit) {
    Column(
        modifier = Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp, Alignment.CenterVertically),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text("SOG05 GNSS・IMU能力診断", style = MaterialTheme.typography.headlineSmall)
        Text(status)
        Button(onClick = onStart) { Text("計測開始") }
        Button(onClick = onStop) { Text("計測停止") }
        Text("保存先: Android/data/com.example.gnss_imu_logger/files/Documents")
    }
}
