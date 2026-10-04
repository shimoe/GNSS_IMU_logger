package com.example.gnss_imu_logger

import android.Manifest
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.os.IBinder
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Surface
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import com.example.gnss_imu_logger.measurement.MeasurementSnapshot
import com.example.gnss_imu_logger.measurement.MeasurementStateListener
import com.example.gnss_imu_logger.model.LogMode
import com.example.gnss_imu_logger.ui.AppContent
import com.example.gnss_imu_logger.ui.AppScreen
import com.example.gnss_imu_logger.ui.theme.GNSS_IMU_loggerTheme
import java.io.File

class MainActivity : ComponentActivity(), MeasurementStateListener {
    private var snapshot by mutableStateOf(MeasurementSnapshot())
    private var mode by mutableStateOf(LogMode.DIAGNOSTIC)
    private var selectedScreen by mutableStateOf(AppScreen.MEASUREMENT)
    private var service: LoggerService? = null
    private var bound = false

    private val connection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, binder: IBinder?) {
            service = (binder as? LoggerService.LocalBinder)?.service()
            service?.addStateListener(this@MainActivity)
            bound = service != null
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            service?.removeStateListener(this@MainActivity)
            service = null
            bound = false
        }
    }

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        if (result[Manifest.permission.ACCESS_FINE_LOCATION] == true) startLogger()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            GNSS_IMU_loggerTheme {
                Surface(modifier = Modifier.fillMaxSize()) {
                    AppContent(
                        snapshot = snapshot,
                        mode = mode,
                        selectedScreen = selectedScreen,
                        documentsDirectory = documentsDirectory(),
                        onScreenChanged = { selectedScreen = it },
                        onModeChanged = { mode = it },
                        onStart = ::requestStart,
                        onStop = ::stopLogger
                    )
                }
            }
        }
    }

    override fun onStart() {
        super.onStart()
        bindService(
            Intent(this, LoggerService::class.java),
            connection,
            Context.BIND_AUTO_CREATE
        )
    }

    override fun onStop() {
        if (bound) {
            service?.removeStateListener(this)
            unbindService(connection)
            bound = false
        }
        super.onStop()
    }

    override fun onSnapshotChanged(snapshot: MeasurementSnapshot) {
        runOnUiThread { this.snapshot = snapshot }
    }

    private fun documentsDirectory(): File =
        File(getExternalFilesDir(null), "Documents").apply { mkdirs() }

    private fun requestStart() {
        val permissions = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions += Manifest.permission.POST_NOTIFICATIONS
        }
        if (permissions.all {
                ContextCompat.checkSelfPermission(this, it) ==
                    PackageManager.PERMISSION_GRANTED
            }
        ) {
            startLogger()
        } else {
            permissionLauncher.launch(permissions.toTypedArray())
        }
    }

    private fun startLogger() {
        ContextCompat.startForegroundService(
            this,
            Intent(this, LoggerService::class.java)
                .setAction(LoggerService.ACTION_START)
                .putExtra(LoggerService.EXTRA_LOG_MODE, mode.name)
        )
    }

    private fun stopLogger() {
        startService(
            Intent(this, LoggerService::class.java)
                .setAction(LoggerService.ACTION_STOP)
        )
    }
}
