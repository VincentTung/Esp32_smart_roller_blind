package com.vincent.android.rollerblindcontroller.utils

const val LOG_TAG = "RollerBlindController"

/**
 * 打开蓝牙 request code
 */
const val REQUEST_ENABLE_BLUETOOTH = 0x16

/**
 * 蓝牙设备名称 - 窗帘控制器
 */
const val DEVICE_NAME = "SmartRollerBlind"

/**
 * 窗帘控制器服务UUID
 */
const val ROLLER_BLIND_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"

/**
 * 窗帘控制器特征UUID
 */
// 控制特征值 - 用于发送控制命令和接收状态通知
const val ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID = "12345678-1234-1234-1234-123456789def"

/**
 * 窗帘控制命令常量
 */
// 基本控制命令
const val BLE_CMD_UP = 'U'        // 升起窗帘
const val BLE_CMD_DOWN = 'D'      // 放下窗帘
const val BLE_CMD_LEFT = 'L'      // 微调升起
const val BLE_CMD_RIGHT = 'R'     // 微调放下
const val BLE_CMD_STOP = 'S'      // 停止电机
const val BLE_CMD_SET = 'T'       // 设置模式
const val BLE_CMD_DIRECTION = 'G' // 切换方向

/**
 * 连接相关常量
 */
const val CONNECTION_CHECK_INTERVAL_MS = 5000L
const val MAX_CONNECTION_RETRY_COUNT = 3
const val CONNECTION_RETRY_DELAY_MS = 2000L

/**
 * 窗帘状态常量
 */
const val CURTAIN_STATUS_STOPPED = "停止"
const val CURTAIN_STATUS_MOVING_UP = "升起中"
const val CURTAIN_STATUS_MOVING_DOWN = "放下中"
const val CURTAIN_STATUS_SETTING = "设置模式"

/**
 * 电机状态常量
 */
const val MOTOR_STATUS_IDLE = "空闲"
const val MOTOR_STATUS_RUNNING = "运行中"
const val MOTOR_STATUS_ERROR = "错误"

/**
 * 方向状态常量
 */
const val DIRECTION_STATUS_NORMAL = "正常"
const val DIRECTION_STATUS_REVERSED = "反向"