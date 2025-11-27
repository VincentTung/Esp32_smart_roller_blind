#ifndef CONFIG_H
#define CONFIG_H

 // 引脚定义
 // 42步进电机
/* 
 * 连接:
 * ESP32 GPIO 6 -> A4988 STEP
 * ESP32 GPIO 7 -> A4988 DIR
 * ESP32 GPIO 8 -> A4988 ENABLE (可选)
 * ESP32 GPIO 9 -> 红外接收模块 OUT
 * ESP32 3.3V    -> A4988 VDD, 红外接收模块 VCC
 * ESP32 GND     -> A4988 GND, 红外接收模块 GND
 * 12V电源       -> A4988 VMOT
 * 12V电源GND    -> A4988 GND
 *
 */
 #define STEP_PIN 6
 #define DIR_PIN 7
 #define ENABLE_PIN 8
 //红外
 #define IR_RECEIVE_PIN 9
 
//红外地址  （根据实际接收到的地址修改）
#define IR_ADDRESS  0xBF00
// 红外命令定义 (基于实际接收到的命令码)
#define IR_KEY_UP 0x1  // val+
#define IR_KEY_DOWN 0x9 // val-
#define IR_KEY_LEFT 0x4 // <
#define IR_KEY_RIGHT 0x6 // >
#define IR_KEY_SHUTDOWN 0x5  //关闭
#define IR_KEY_SET 0xE //   ST/REPT
#define IR_KEY_DIRECTION 0x1A  // 数字9

//开关窗帘时间默认时间
const int DEFAULT_CURTAIN_TIME = 5000;  // 默认窗帘开关时间 (毫秒)
// 左右按键转动时间设置 
const int SIDE_KEY_TIME = 1000;  // 左右按键转动时间 (毫秒) 上下微调响应参数调这个

const int DEFAULT_IS_NORMAL_DIRECTION = true;

// EEPROM 配置
#define EEPROM_SIZE 512                    // EEPROM 大小
#define EEPROM_CURTAIN_TIME_ADDR 0         // 窗帘时间存储地址
#define EEPROM_DIRECTION_ADDR 4            // 方向设置存储地址
#define EEPROM_DIRECTION_INIT_FLAG_ADDR 5  // 方向设置初始化标志地址

// BLE配置
#define ENABLE_BLE false  // 蓝牙开关：true=开启，false=关闭（节省电量）
#define BLE_DEVICE_NAME "SmartRollerBlind"
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define BLE_CHARACTERISTIC_CONTROL_UUID "12345678-1234-1234-1234-123456789def"

//微步分辨率配置
// MS1/2/3
// 默认步距角电机：200步/圈（1.8°）
// L/L/L: 全步进 (1/1) → 200步/圈
// H/L/L: 1/2 微步 → 400步/圈
// L/H/L: 1/4 微步 → 800步/圈
// H/H/L: 1/8 微步 → 1600步/圈
// H/H/H: 1/16 微步 → 3200步/圈

// 步进电机配置         // 全步进
#define DEFAULT_MOTOR_SPEED 1000   // 默认电机转速（步/秒）
#define MIN_MOTOR_SPEED 200       // 最小电机转速（步/秒）
#define MAX_MOTOR_SPEED 1000      // 最大电机转速（步/秒）

// 电源管理配置
#define ENABLE_DEBUG_OUTPUT true  // 调试输出开关：true=开启，false=关闭（节省电量）
#define IDLE_DELAY_MS 50          // 空闲时延迟（毫秒），降低CPU使用率

// 调试输出宏定义
#if ENABLE_DEBUG_OUTPUT
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif


#endif // 