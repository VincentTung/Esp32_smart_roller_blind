package com.vincent.android.rollerblindcontroller.logic

import VTBLECallback
import android.content.Context
import com.vincent.android.rollerblindcontroller.utils.DEVICE_NAME
import com.vincent.android.rollerblindcontroller.utils.ROLLER_BLIND_SERVICE_UUID
import com.vincent.android.rollerblindcontroller.utils.ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_UP
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_DOWN
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_LEFT
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_RIGHT
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_STOP
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_SET
import com.vincent.android.rollerblindcontroller.utils.BLE_CMD_DIRECTION
import com.vincent.android.rollerblindcontroller.utils.CONNECTION_CHECK_INTERVAL_MS
import com.vincent.android.rollerblindcontroller.utils.MAX_CONNECTION_RETRY_COUNT
import com.vincent.android.rollerblindcontroller.utils.CONNECTION_RETRY_DELAY_MS
import com.vincent.library.ble.logic.VTBLEController
import com.vincent.library.ble.logic.BLEDeviceManager
import com.vincent.library.base.util.CoroutineUtil
import com.vincent.library.base.util.logd

/**
 * 窗帘控制器 - 负责发送BLE命令控制窗帘
 */
class RollerBlindController private constructor() {
    companion object {
        private const val TAG = "RollerBlindController"
        private const val SCOPE_NAME = "RollerBlindController"
        @Volatile
        private var INSTANCE: RollerBlindController? = null

        fun getInstance(): RollerBlindController {
            return INSTANCE ?: synchronized(this) {
                INSTANCE ?: RollerBlindController().also { INSTANCE = it }
            }
        }
    }
    
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        ERROR
    }

    private var connectionState = ConnectionState.DISCONNECTED
    private lateinit var context: Context
    private lateinit var mBLEController: VTBLEController
    private var isInitialized = false
    private var lastConnectionCheck = 0L
    private var connectionRetryCount = 0
    private val maxRetryCount = MAX_CONNECTION_RETRY_COUNT
    
    /**
     * 初始化BLE连接
     */
    fun initBLE(context: Context) {
        if (!isInitialized) {
            this@RollerBlindController.context = context.applicationContext
            // 初始化DeviceManager
            BLEDeviceManager.init(this@RollerBlindController.context)
            
            // 创建BLE控制器
            initBleController()
            // 如果有保存的设备地址，设置到控制器中用于快速重连
            val savedDeviceAddress = BLEDeviceManager.getLastConnectedDeviceAddress()
            if (savedDeviceAddress != null) {
                mBLEController.setDeviceAddress(savedDeviceAddress)
                logd(TAG, "RollerBlindController initialized with saved device address: $savedDeviceAddress")
            } else {
                logd(TAG, "RollerBlindController initialized without saved device address")
            }
            
            isInitialized = true
        }
    }

    fun getConnectionState(): ConnectionState = connectionState
    
    /**
     * 检查连接状态并尝试重连
     */
    fun checkConnectionAndReconnect() {
        val currentTime = System.currentTimeMillis()
        
        // 每5秒检查一次连接状态
        if (currentTime - lastConnectionCheck < CONNECTION_CHECK_INTERVAL_MS) {
            return
        }
        
        lastConnectionCheck = currentTime
        
        if (connectionState == ConnectionState.CONNECTED && !mBLEController.isConnected()) {
            logd(TAG, "检测到连接断开，尝试重连...")
            connectionState = ConnectionState.DISCONNECTED
            connectionRetryCount++
            
            if (connectionRetryCount <= maxRetryCount) {
                logd(TAG, "尝试重连 ($connectionRetryCount/$maxRetryCount)")
                // 这里可以添加自动重连逻辑
            } else {
                logd(TAG, "重连次数超限，停止自动重连")
                connectionState = ConnectionState.ERROR
            }
        } else if (connectionState == ConnectionState.CONNECTED && mBLEController.isConnected()) {
            // 连接正常，重置重试计数
            connectionRetryCount = 0
        }
    }

    /**
     * 窗帘控制器连接
     */
    fun connect(callback: VTBLECallback) {
        if (!isInitialized) {
            logd(TAG, "RollerBlindController not initialized")
            return
        }
        
        connectionState = ConnectionState.CONNECTING

        mBLEController.connect(object : VTBLECallback by callback {
            override fun onConnected(name: String?, address: String?) {
                connectionState = ConnectionState.CONNECTED
                logd(TAG, "=== 连接成功 ===")
                logd(TAG, "设备名称: ${name ?: "未知"}")
                logd(TAG, "设备地址: ${address ?: "未知"}")
                
                callback.onConnected(name, address)
            }

            override fun onDisConnected() {
                connectionState = ConnectionState.DISCONNECTED
                logd(TAG, "=== 连接断开 ===")
                callback.onDisConnected()
            }

            override fun onScanFailed() {
                connectionState = ConnectionState.ERROR
                logd(TAG, "=== 连接失败 ===")
                logd(TAG, "所有连接方式都已尝试失败")
                callback.onScanFailed()
            }
            
            // MTU协商相关回调
            override fun onMtuNegotiationSuccess(mtu: Int) {
                logd(TAG, "=== RollerBlindController: MTU协商成功 ===")
                logd(TAG, "协商后的MTU大小: $mtu 字节")
                callback.onMtuNegotiationSuccess(mtu)
            }
            
            override fun onMtuNegotiationFailed(requestedMtu: Int, actualMtu: Int) {
                logd(TAG, "=== RollerBlindController: MTU协商失败 ===")
                logd(TAG, "请求的MTU大小: $requestedMtu 字节")
                logd(TAG, "实际使用的MTU大小: $actualMtu 字节")
                callback.onMtuNegotiationFailed(requestedMtu, actualMtu)
            }
            
            // PHY协商相关回调
            override fun onPhyNegotiationSuccess(txPhy: Int, rxPhy: Int) {
                logd(TAG, "=== RollerBlindController: PHY协商成功 ===")
                logd(TAG, "TX PHY: $txPhy, RX PHY: $rxPhy")
                callback.onPhyNegotiationSuccess(txPhy, rxPhy)
            }
            
            override fun onPhyNegotiationFailed(requestedPhy: Int, actualPhy: Int) {
                logd(TAG, "=== RollerBlindController: PHY协商失败 ===")
                logd(TAG, "请求的PHY: $requestedPhy")
                logd(TAG, "实际的PHY: $actualPhy")
                callback.onPhyNegotiationFailed(requestedPhy, actualPhy)
            }
            
            override fun onPhyReadSuccess(txPhy: Int, rxPhy: Int) {
                logd(TAG, "=== RollerBlindController: PHY读取成功 ===")
                logd(TAG, "TX PHY: $txPhy, RX PHY: $rxPhy")
                callback.onPhyReadSuccess(txPhy, rxPhy)
            }
            
            override fun onPhyReadFailed() {
                logd(TAG, "=== RollerBlindController: PHY读取失败 ===")
                callback.onPhyReadFailed()
            }
            
            override fun onPhyUpdateSuccess(txPhy: Int, rxPhy: Int) {
                logd(TAG, "=== RollerBlindController: PHY更新成功 ===")
                logd(TAG, "新的TX PHY: $txPhy, 新的RX PHY: $rxPhy")
                callback.onPhyUpdateSuccess(txPhy, rxPhy)
            }
            
            override fun onPhyUpdateFailed() {
                logd(TAG, "=== RollerBlindController: PHY更新失败 ===")
                callback.onPhyUpdateFailed()
            }
            
            // 配对相关回调
            override fun onBondingSuccess(name: String?, address: String?) {
                logd(TAG, "=== RollerBlindController: 配对成功 ===")
                logd(TAG, "设备名称: ${name ?: "未知"}")
                logd(TAG, "设备地址: ${address ?: "未知"}")
                logd(TAG, "设备配对成功，连接稳定性将得到提升")
                callback.onBondingSuccess(name, address)
            }
        })
    }
    
    /**
     * 重新初始化BLE控制器（当设备地址发生变化时）
     */
    fun reinitializeBLE() {
        logd(TAG, "=== 重新初始化BLE控制器 ===")
        
        // 断开当前连接
        if (connectionState == ConnectionState.CONNECTED) {
            mBLEController.disconnect()
        }

        // 创建BLE控制器
        initBleController()
        // 清除之前保存的设备地址
        mBLEController.clearDeviceAddress()
        
        // 重置连接状态
        connectionState = ConnectionState.DISCONNECTED
        
        logd(TAG, "BLE控制器重新初始化完成")
    }

    private fun initBleController() {
        mBLEController = VTBLEController(
            context, DEVICE_NAME, ROLLER_BLIND_SERVICE_UUID, ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID,
            ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID  // 使用同一个特征值用于控制和通知
        )
        
        logd(TAG, "BLE控制器初始化完成，将自动进行MTU协商和PHY优化")
    }
    
    /**
     * 检查设备是否已连接
     */
    fun isDeviceConnected(): Boolean {
        return connectionState == ConnectionState.CONNECTED && mBLEController.isConnected()
    }
    
    /**
     * 发送BLE命令
     */
    private fun sendCommand(command: Char, callback: ((Boolean) -> Unit)? = null) {
        if (!isDeviceConnected()) {
            logd(TAG, "设备未连接，无法发送命令: $command")
            callback?.invoke(false)
            return
        }
        
        logd(TAG, "发送BLE命令: $command")
        val commandStr = command.toString()
        
        // 使用VTBLEController发送命令
        mBLEController.sendText(
            ROLLER_BLIND_SERVICE_UUID,
            ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID,
            commandStr
        )
        
        // 暂时假设发送成功，实际应该等待回调
        callback?.invoke(true)
    }
    
    /**
     * 升起窗帘
     */
    fun curtainUp(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 升起窗帘")
        sendCommand(BLE_CMD_UP, callback)
    }
    
    /**
     * 放下窗帘
     */
    fun curtainDown(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 放下窗帘")
        sendCommand(BLE_CMD_DOWN, callback)
    }
    
    /**
     * 微调升起
     */
    fun curtainLeft(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 微调升起")
        sendCommand(BLE_CMD_LEFT, callback)
    }
    
    /**
     * 微调放下
     */
    fun curtainRight(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 微调放下")
        sendCommand(BLE_CMD_RIGHT, callback)
    }
    
    /**
     * 停止电机
     */
    fun curtainStop(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 停止电机")
        sendCommand(BLE_CMD_STOP, callback)
    }
    
    /**
     * 设置模式
     */
    fun curtainSet(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 设置模式")
        sendCommand(BLE_CMD_SET, callback)
    }
    
    /**
     * 切换方向
     */
    fun curtainDirection(callback: ((Boolean) -> Unit)? = null) {
        logd(TAG, "执行: 切换方向")
        sendCommand(BLE_CMD_DIRECTION, callback)
    }
    
    /**
     * 断开连接
     */
    fun disconnect() {
        logd(TAG, "断开BLE连接")
        mBLEController.disconnect()
        connectionState = ConnectionState.DISCONNECTED
    }
    
    /**
     * 重连
     */
    fun reconnect(callback: VTBLECallback) {
        logd(TAG, "重新连接")
        disconnect()
        connect(callback)
    }
    
    /**
     * 获取当前MTU大小
     */
    fun getCurrentMtu(): Int {
        return mBLEController.getCurrentMtu()
    }
    
    /**
     * 获取当前TX PHY
     */
    fun getCurrentTxPhy(): Int {
        return mBLEController.getCurrentTxPhy()
    }
    
    /**
     * 获取当前RX PHY
     */
    fun getCurrentRxPhy(): Int {
        return mBLEController.getCurrentRxPhy()
    }
    
    /**
     * 检查PHY协商是否成功
     */
    fun isPhyNegotiationSuccessful(): Boolean {
        return mBLEController.isPhyNegotiationSuccessful()
    }
    
    /**
     * 检查设备是否支持2M PHY
     */
    fun isLe2MPhySupported(): Boolean {
        return mBLEController.isLe2MPhySupported()
    }
    
    /**
     * 检查设备是否支持Coded PHY
     */
    fun isLeCodedPhySupported(): Boolean {
        return mBLEController.isLeCodedPhySupported()
    }
    
    /**
     * 获取设备支持的PHY列表
     */
    fun getSupportedPhys(): List<Int> {
        return mBLEController.getSupportedPhys()
    }
    
    /**
     * 获取PHY描述信息
     */
    fun getPhyDescription(phy: Int): String {
        return mBLEController.getPhyDescription(phy)
    }
    
    /**
     * 获取PHY性能描述
     */
    fun getPhyPerformanceDescription(phy: Int): String {
        return mBLEController.getPhyPerformanceDescription(phy)
    }
    
    /**
     * 获取连接质量信息
     */
    fun getConnectionQualityInfo(): String {
        return mBLEController.getConnectionQualityInfo()
    }
    
    /**
     * 检查当前设备是否已配对
     */
    fun isCurrentDeviceBonded(): Boolean {
        return mBLEController.isCurrentDeviceBonded()
    }
    
    /**
     * 获取配对状态信息
     */
    fun getBondingStatusInfo(): String {
        return mBLEController.getBondingStatusInfo()
    }
    
    /**
     * 获取设备信息摘要
     */
    fun getDeviceInfoSummary(): String {
        return BLEDeviceManager.getDeviceInfoSummary()
    }
    
    /**
     * 手动请求配对（如果需要）
     */
    fun requestBonding() {
        if (mBLEController.isConnected()) {
            val deviceAddress = mBLEController.getDeviceAddress()
            if (deviceAddress.isNotEmpty()) {
                val bluetoothAdapter = com.vincent.library.ble.util.VTBluetoothUtil.getBluetoothAdapter(context)
                val device = bluetoothAdapter?.getRemoteDevice(deviceAddress)
                
                if (device?.bondState != android.bluetooth.BluetoothDevice.BOND_BONDED) {
                    logd(TAG, "=== 手动请求配对 ===")
                    logd(TAG, "设备名称: ${device?.name ?: "未知"}")
                    logd(TAG, "设备地址: $deviceAddress")
                    
                    val bondResult = device?.createBond()
                    logd(TAG, "配对请求结果: $bondResult")
                    
                    if (bondResult == true) {
                        logd(TAG, "配对请求已发送，等待用户确认")
                    } else {
                        logd(TAG, "配对请求失败")
                    }
                } else {
                    logd(TAG, "设备已配对，无需再次配对")
                }
            }
        } else {
            logd(TAG, "设备未连接，无法请求配对")
        }
    }
}