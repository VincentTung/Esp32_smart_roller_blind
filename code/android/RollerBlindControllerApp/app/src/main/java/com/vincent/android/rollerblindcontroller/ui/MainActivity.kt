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
import android.widget.Button
import android.widget.TextView
import androidx.lifecycle.lifecycleScope
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

    // UI组件
    private lateinit var tvDeviceName: TextView
    private lateinit var tvDeviceAddress: TextView
    private lateinit var btnReconnect: Button
    private lateinit var btnPhyInfo: Button
    
    // 窗帘控制按钮
    private lateinit var btnCurtainUp: Button
    private lateinit var btnCurtainDown: Button
    private lateinit var btnCurtainLeft: Button
    private lateinit var btnCurtainRight: Button
    private lateinit var btnCurtainStop: Button
    private lateinit var btnCurtainSet: Button
    private lateinit var btnCurtainDirection: Button
    
    // 状态显示
    private lateinit var tvCurtainStatus: TextView
    private lateinit var tvMotorStatus: TextView
    private lateinit var tvDirectionStatus: TextView

    // 业务逻辑
    private val rxPermission = RxPermissions(this)
    private lateinit var rollerBlindController: RollerBlindController

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        logd(TAG, "onCreate")
        setContentView(R.layout.activity_main)

        // 从Application获取已初始化的控制器实例
        rollerBlindController = RollerBlindController.getInstance()
        logd(TAG, "从Application获取窗帘控制器实例")
        
        initView()
        checkBluetoothAndPermissions()
    }

    private fun initView() {

        initDeviceInfoViews()
        initCurtainControlViews()
        initStatusViews()
    }

    private fun initDeviceInfoViews() {
        tvDeviceName = findViewById(R.id.tv_device_name)
        tvDeviceAddress = findViewById(R.id.tv_device_address)
        btnReconnect = findViewById(R.id.btn_reconnect)
        btnPhyInfo = findViewById(R.id.btn_phy_info)

        btnReconnect.setOnClickListener { reconnect() }
        btnPhyInfo.setOnClickListener { showPhyInfo() }
    }

    private fun initCurtainControlViews() {
        btnCurtainUp = findViewById(R.id.btn_curtain_up)
        btnCurtainDown = findViewById(R.id.btn_curtain_down)
        btnCurtainLeft = findViewById(R.id.btn_curtain_left)
        btnCurtainRight = findViewById(R.id.btn_curtain_right)
        btnCurtainStop = findViewById(R.id.btn_curtain_stop)
        btnCurtainSet = findViewById(R.id.btn_curtain_set)
        btnCurtainDirection = findViewById(R.id.btn_curtain_direction)

        // 设置按钮点击事件
        btnCurtainUp.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainUp { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "正在升起窗帘")
                            updateCurtainStatus("升起中")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainDown.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainDown { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "正在放下窗帘")
                            updateCurtainStatus("放下中")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainLeft.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainLeft { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "微调升起")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainRight.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainRight { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "微调放下")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainStop.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainStop { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "电机已停止")
                            updateMotorStatus("停止")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainSet.setOnClickListener { 
            if (checkConnection()) {
                rollerBlindController.curtainSet { success ->
                    runOnUiThread {
                        if (success) {
                            ToastUtil.show(this, "进入设置模式")
                        } else {
                            ToastUtil.show(this, "命令执行失败")
                        }
                    }
                }
            }
        }

        btnCurtainDirection.setOnClickListener { 
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
        tvCurtainStatus = findViewById(R.id.tv_curtain_status)
        tvMotorStatus = findViewById(R.id.tv_motor_status)
        tvDirectionStatus = findViewById(R.id.tv_direction_status)

        // 初始化状态显示
        updateCurtainStatus("未知")
        updateMotorStatus("停止")
        updateDirectionStatus("正常")
    }

    private fun checkConnection(): Boolean {
        if (!rollerBlindController.isDeviceConnected()) {
            ToastUtil.show(this, R.string.device_not_connected)
            return false
        }
        return true
    }

    private fun updateCurtainStatus(status: String) {
        tvCurtainStatus.text = "窗帘状态: $status"
    }

    private fun updateMotorStatus(status: String) {
        tvMotorStatus.text = "电机状态: $status"
    }

    private fun updateDirectionStatus(status: String) {
        tvDirectionStatus.text = "转动方向: $status"
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

    private fun showPhyInfo() {
        if (!rollerBlindController.isDeviceConnected()) {
            ToastUtil.show(this, "设备未连接")
            return
        }
        
        val mtu = rollerBlindController.getCurrentMtu()
        val txPhy = rollerBlindController.getCurrentTxPhy()
        val rxPhy = rollerBlindController.getCurrentRxPhy()
        val connectionQuality = rollerBlindController.getConnectionQualityInfo()
        val bondingStatus = rollerBlindController.getBondingStatusInfo()
        
        val info = """
            连接信息:
            MTU: $mtu 字节
            TX PHY: ${rollerBlindController.getPhyDescription(txPhy)}
            RX PHY: ${rollerBlindController.getPhyDescription(rxPhy)}
            
            $connectionQuality
            
            $bondingStatus
        """.trimIndent()
        
        ToastUtil.show(this, info)
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