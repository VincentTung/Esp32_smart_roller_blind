package com.vincent.android.rollerblindcontroller.ui

import VTBLECallback
import android.Manifest.permission.ACCESS_FINE_LOCATION
import android.Manifest.permission.BLUETOOTH_CONNECT
import android.Manifest.permission.BLUETOOTH_SCAN
import android.annotation.SuppressLint
import android.app.Activity
import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.os.Bundle
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.TextView
import androidx.lifecycle.lifecycleScope
import com.bumptech.glide.Glide
import com.bumptech.glide.load.resource.gif.GifDrawable
import com.bumptech.glide.request.target.CustomTarget
import com.bumptech.glide.request.transition.Transition
import com.tbruyelle.rxpermissions2.RxPermissions
import com.vincent.android.rollerblindcontroller.R
import com.vincent.android.rollerblindcontroller.logic.RollerBlindController
import com.vincent.library.base.ui.VTBaseActivity
import com.vincent.library.base.util.ToastUtil
import com.vincent.library.base.util.logd
import com.vincent.library.ble.util.VTBluetoothUtil
import io.reactivex.functions.Consumer
import kotlinx.coroutines.launch

/**
 * 智能窗帘控制器主界面
 */
class MainActivity : VTBaseActivity(), VTBLECallback {
    companion object {
        private const val TAG = "MainActivity"
        private const val REQUEST_ENABLE_BLUETOOTH = 1001
    }

    private lateinit var tvDeviceName: TextView
    private lateinit var tvDeviceAddress: TextView
    private lateinit var btnReconnect: LinearLayout
    private lateinit var btnBlindsSet: LinearLayout
    private lateinit var btnBlindsDirection: LinearLayout
    private lateinit var ivGif: ImageView
    
    // 窗帘控制按钮
    private lateinit var ivBlindsUp: ImageView
    private lateinit var ivBlindsDown: ImageView
    private lateinit var ivBlindsLeft: ImageView
    private lateinit var ivBlindsRight: ImageView
    private lateinit var ivBlindsStop: ImageView

    private val rxPermission = RxPermissions(this)
    private lateinit var rollerBlindController: RollerBlindController

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        logd(TAG, "onCreate")
        setContentView(R.layout.activity_main)

        rollerBlindController = RollerBlindController.getInstance()
        initView()
        stopGif()
        checkBluetoothAndPermissions()
    }

    private fun initView() {
        initDeviceInfoViews()
        initBlindsControlViews()
        initStatusViews()
    }

    private fun initDeviceInfoViews() {
        tvDeviceName = findViewById(R.id.tv_device_name)
        tvDeviceAddress = findViewById(R.id.tv_device_address)
        btnReconnect = findViewById(R.id.btn_reconnect)
        btnBlindsSet = findViewById(R.id.btn_curtain_set)
        btnBlindsDirection = findViewById(R.id.btn_curtain_direction)
        ivGif = findViewById(R.id.iv_gif)

        // 设置按钮图标和文字
        setupButton(btnReconnect, R.drawable.link, "重连")
        setupButton(btnBlindsSet, R.drawable.setting, "设置模式")
        setupButton(btnBlindsDirection, R.drawable.refesh_ratate, "切换方向")

        btnReconnect.setOnClickListener { reconnect() }
    }

    private fun setupButton(button: LinearLayout, iconRes: Int, text: String) {
        val icon = button.findViewById<ImageView>(R.id.btn_icon)
        val textView = button.findViewById<TextView>(R.id.btn_text)
        icon.setImageResource(iconRes)
        textView.text = text
    }

    private fun initBlindsControlViews() {
        ivBlindsUp = findViewById(R.id.btn_curtain_up)
        ivBlindsDown = findViewById(R.id.btn_curtain_down)
        ivBlindsLeft = findViewById(R.id.btn_curtain_left)
        ivBlindsRight = findViewById(R.id.btn_curtain_right)
        ivBlindsStop = findViewById(R.id.btn_curtain_stop)
        btnBlindsSet = findViewById(R.id.btn_curtain_set)
        btnBlindsDirection = findViewById(R.id.btn_curtain_direction)

        // 设置按钮点击事件
        ivBlindsUp.setOnClickListener { 
            if (checkConnection()) {
                playGif("up.gif")
                rollerBlindController.curtainUp { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "正在升起窗帘")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                            stopGif()
                        }
                    }
                }
            }
        }

        ivBlindsDown.setOnClickListener { 
            if (checkConnection()) {
                playGif("down.gif")
                rollerBlindController.curtainDown { success ->
                    runOnUiThread {
                        if (success) {
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                            stopGif()
                        }
                    }
                }
            }
        }

        ivBlindsLeft.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainLeft { success ->
                    runOnUiThread {
                        if (success) {
                            stopGif()
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        ivBlindsRight.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainRight { success ->
                    runOnUiThread {
                        if (success) {

                            stopGif()
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        ivBlindsStop.setOnClickListener { 
            if (checkConnection()) {
                stopGif()
                rollerBlindController.curtainStop { success ->
                    runOnUiThread {
                        if (success) {
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnBlindsSet.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainSet { success ->
                    runOnUiThread {
                        if (success) {
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnBlindsDirection.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainDirection { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "方向已切换")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }
    }

    private fun initStatusViews() {

        // 初始化状态显示
        updateDeviceInfo("未连接", "无")

    }

    private fun checkConnection(): Boolean {
        if (!rollerBlindController.isDeviceConnected()) {
            ToastUtil.show(this, R.string.device_not_connected)
            return false
        }
        return true
    }





    private fun checkBluetoothAndPermissions() {
        if (!VTBluetoothUtil.isEnable(this)) {
            turnOnBluetooth()
        } else {
            checkLocationPermission()
        }
    }

    private fun reconnect() {
        if (rollerBlindController.isDeviceConnected()) {
            logd(TAG, "设备已连接，无需重连")
            ToastUtil.show(this, R.string.device_already_connected)
            return
        }

        logd(TAG, "开始重连流程")
        rollerBlindController.reconnect(this)
    }


    private fun playGif(gifFileName: String) {
        Glide.with(this)
            .asGif()
            .load("file:///android_asset/$gifFileName")
            .into(object : CustomTarget<GifDrawable>() {
                override fun onResourceReady(resource: GifDrawable, transition: Transition<in GifDrawable>?) {
                    resource.setLoopCount(1) // 只播放一次
                    ivGif.setImageDrawable(resource)
                    resource.start()
                }
                
                override fun onLoadCleared(placeholder: android.graphics.drawable.Drawable?) {
                    // 清理资源
                }
            })
    }

    private fun stopGif() {
        Glide.with(this).clear(ivGif)
        ivGif.setImageResource(R.drawable.half_close)
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)

        if (requestCode == REQUEST_ENABLE_BLUETOOTH) {
            if (resultCode == Activity.RESULT_OK) {
                checkLocationPermission()
            } else {
                ToastUtil.show(this, R.string.turn_on_failed)
            }
        }
    }

    @SuppressLint("CheckResult")
    private fun checkLocationPermission() {
        if (!rxPermission.isGranted(ACCESS_FINE_LOCATION) || 
            !rxPermission.isGranted(BLUETOOTH_SCAN) || 
            !rxPermission.isGranted(BLUETOOTH_CONNECT)) {
            rxPermission.request(ACCESS_FINE_LOCATION, BLUETOOTH_SCAN, BLUETOOTH_CONNECT)
                .subscribe(Consumer { isGranted ->
                    if (isGranted) {
                        connectToDevice()
                    } else {
                        ToastUtil.show(this, R.string.require_permission_failed)
                    }
                })
        } else {
            connectToDevice()
        }
    }

    private fun connectToDevice() {
        logd(TAG, "开始连接设备")
        startLoading(getString(R.string.connecting_device))
        // 使用RollerBlindController进行连接
        rollerBlindController.connect(this)
    }

    @SuppressLint("MissingPermission")
    private fun turnOnBluetooth() {
        if (!rxPermission.isGranted(BLUETOOTH_CONNECT)) {
            rxPermission.request(BLUETOOTH_CONNECT)
            return
        }
        val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
        startActivityForResult(enableBtIntent, REQUEST_ENABLE_BLUETOOTH)
    }

    private fun updateDeviceInfo(name: String, address: String) {
        tvDeviceName.text = "设备名称: $name"
        tvDeviceAddress.text = "设备地址: $address"
    }

    private fun runOnUiThreadSafely(action: () -> Unit) {
        lifecycleScope.launch {
            if (!isFinishing && !isDestroyed) {
                action()
            }
        }
    }

    // BLE回调方法
    override fun onCheckCharacteristicSuccess() {
        logd(TAG, "BLE特征值检查成功")
    }

    override fun onBrightnessReceived(brightness: Int) {
        // 窗帘控制器不需要亮度功能
    }

    override fun onDisConnected() {
        runOnUiThreadSafely {
            logd(TAG, "设备断开连接")
            stopLoading()
            ToastUtil.show(this, R.string.disconnected)
            updateDeviceInfo("未连接", "无")
        }
    }

    override fun onConnecting() {
        logd(TAG, "正在连接设备")
    }

    override fun onScanFailed() {
        runOnUiThreadSafely {
            logd(TAG, "扫描失败")
            stopLoading()
            ToastUtil.show(this, R.string.find_no_device)
        }
    }

    override fun onConnected(name: String?, address: String?) {
        runOnUiThreadSafely {
            logd(TAG, "设备连接成功")
            ToastUtil.show(this, R.string.connected)
            stopLoading()
            updateDeviceInfo(name ?: "未知设备", address ?: "未知地址")
        }
    }

    override fun writeDataCallback(isSuccess: Boolean) {
        runOnUiThreadSafely {
            if (!isSuccess) {
                ToastUtil.show(this, R.string.write_failed)
            }
        }
    }

    // MTU协商相关回调
    override fun onMtuNegotiationSuccess(mtu: Int) {
        runOnUiThreadSafely {
            logd(TAG, "MTU协商成功: $mtu 字节")
        }
    }

    override fun onMtuNegotiationFailed(requestedMtu: Int, actualMtu: Int) {
        runOnUiThreadSafely {
            logd(TAG, "MTU协商失败: 请求$requestedMtu, 实际$actualMtu")
        }
    }

    // PHY协商相关回调
    override fun onPhyNegotiationSuccess(txPhy: Int, rxPhy: Int) {
        runOnUiThreadSafely {
            logd(TAG, "PHY协商成功: TX=$txPhy, RX=$rxPhy")
        }
    }

    override fun onPhyNegotiationFailed(requestedPhy: Int, actualPhy: Int) {
        runOnUiThreadSafely {
            logd(TAG, "PHY协商失败: 请求$requestedPhy, 实际$actualPhy")
        }
    }

    override fun onPhyReadSuccess(txPhy: Int, rxPhy: Int) {
        runOnUiThreadSafely {
            logd(TAG, "PHY读取成功: TX=$txPhy, RX=$rxPhy")
        }
    }

    override fun onPhyReadFailed() {
        runOnUiThreadSafely {
            logd(TAG, "PHY读取失败")
        }
    }

    override fun onPhyUpdateSuccess(txPhy: Int, rxPhy: Int) {
        runOnUiThreadSafely {
            logd(TAG, "PHY更新成功: TX=$txPhy, RX=$rxPhy")
        }
    }

    override fun onPhyUpdateFailed() {
        runOnUiThreadSafely {
            logd(TAG, "PHY更新失败")
        }
    }

    // 配对相关回调
    override fun onBondingSuccess(name: String?, address: String?) {
        runOnUiThreadSafely {
            logd(TAG, "配对成功: $name ($address)")
            ToastUtil.show(this, "配对成功")
        }
    }

    override fun onDestroy() {
        super.onDestroy()
    }
}