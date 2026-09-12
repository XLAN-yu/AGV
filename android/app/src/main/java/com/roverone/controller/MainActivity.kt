package com.roverone.controller

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.provider.Settings
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.SystemBarStyle
import androidx.activity.viewModels
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.getValue
import androidx.lifecycle.compose.collectAsStateWithLifecycle

class MainActivity : ComponentActivity() {
    private val controller: RoverController by viewModels()
    private val bluetoothPermission = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { result ->
        if (result.values.all { it }) controller.connect()
    }

    private fun connectWithPermission() {
        if (controller.state.value.controlLink != ControlLink.BLE || Build.VERSION.SDK_INT < 31 ||
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED
        ) controller.connect()
        else bluetoothPermission.launch(arrayOf(Manifest.permission.BLUETOOTH_CONNECT))
    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge(statusBarStyle = SystemBarStyle.dark(android.graphics.Color.TRANSPARENT),
            navigationBarStyle = SystemBarStyle.dark(android.graphics.Color.TRANSPARENT))
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContent {
            val state by controller.state.collectAsStateWithLifecycle()
            RoverScreen(state, controller::address, controller::bleAddress, controller::controlLink,
                controller::simulation, ::connectWithPermission,
                { controller.disconnect() }, { startActivity(Intent(Settings.ACTION_WIFI_SETTINGS)) },
                { startActivity(Intent(Settings.ACTION_BLUETOOTH_SETTINGS)) },
                controller::arm, { controller.stop() }, controller::estop, controller::clearEstop,
                controller::obstacleGuard, controller::visionFollow, controller::shutdownOrangePi,
                controller::stick, controller::strafe, controller::speed, controller::wheelTest,
                controller::togglePathRecording, controller::pathDirection, controller::replayPath,
                controller::renamePath, controller::deletePath, controller::setPathSegmentDuration,
                controller::addParkingAfter, controller::deletePathSegment)
        }
    }
    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        controller.focusChanged(hasFocus)
    }
    override fun onPause() {
        controller.background()
        super.onPause()
    }
}
