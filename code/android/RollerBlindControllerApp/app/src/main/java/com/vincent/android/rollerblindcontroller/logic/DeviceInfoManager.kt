//package com.vincent.android.rollerblindcontroller.logic
//
//import android.widget.Button
//import android.widget.TextView
//import com.vincent.library.ble.logic.BLEDeviceManager
//import com.vincent.library.base.util.logd
//
///**
// * 设备信息管理器
// * 负责管理设备连接状态、设备信息显示和重连功能
// */
//class DeviceInfoManager(
//    private val ledController: LEDController,
//    private val onReconnectClick: () -> Unit,
//    private val onPhyInfoClick: () -> Unit,
//    private val onBondingClick: () -> Unit
//) {
//    companion object {
//        private const val TAG = "DeviceInfoManager"
//    }
//
//    private var tvDeviceName: TextView? = null
//    private var tvDeviceAddress: TextView? = null
//    private var btnReconnect: Button? = null
//    private var btnPhyInfo: Button? = null
//    private var isConnected = false
//
//    fun initViews(
//        tvDeviceName: TextView,
//        tvDeviceAddress: TextView,
//        btnReconnect: Button,
//        btnPhyInfo: Button
//    ) {
//        this.tvDeviceName = tvDeviceName
//        this.tvDeviceAddress = tvDeviceAddress
//        this.btnReconnect = btnReconnect
//        this.btnPhyInfo = btnPhyInfo
//
//        setupClickListeners()
//        updateDeviceInfo()
//    }
//
//    private fun setupClickListeners() {
//        btnReconnect?.setOnClickListener { onReconnectClick() }
//        btnReconnect?.setOnLongClickListener {
//            onBondingClick()
//            true
//        }
//        btnPhyInfo?.setOnClickListener { onPhyInfoClick() }
//    }
//
//    fun updateDeviceInfo() {
//        if (isConnected) {
//            // 如果已连接，显示当前连接信息（由onConnected方法设置）
//        } else {
//            tvDeviceName?.text = "设备名称："
//            tvDeviceAddress?.text = ""
//        }
//    }
//
//    fun onConnected(name: String?, address: String?) {
//        logd(TAG, "=== 设备连接成功 ===")
//        logd(TAG, "设备名称: ${name ?: "未知"}")
//        logd(TAG, "设备地址: ${address ?: "未知"}")
//
//        isConnected = true
//        name?.let {
//            tvDeviceName?.text = "设备名称: $it"
//        }
//        address?.let {
//            tvDeviceAddress?.text = it
//        }
//        btnReconnect?.isEnabled = false
//    }
//
//    fun onDisconnected() {
//        logd(TAG, "=== 设备断开连接 ===")
//        isConnected = false
//        updateDeviceInfo()
//        btnReconnect?.isEnabled = true
//    }
//
//    fun clearSavedDevice() {
//        logd(TAG, "=== 清除保存的设备信息 ===")
//        BLEDeviceManager.clearSavedDevice()
//        ledController.reinitializeBLE()
//        updateDeviceInfo()
//        logd(TAG, "设备信息清除完成")
//    }
//
//    fun isDeviceConnected(): Boolean = isConnected
//
//    fun getDeviceInfoSummary(): String {
//        val deviceInfo = StringBuilder()
//        deviceInfo.append("设备信息:\n")
//        deviceInfo.append("- 名称: ${tvDeviceName?.text}\n")
//        deviceInfo.append("- 地址: ${tvDeviceAddress?.text}\n")
//        deviceInfo.append("- 连接状态: ${if (isConnected) "已连接" else "未连接"}\n")
//        return deviceInfo.toString()
//    }
//}