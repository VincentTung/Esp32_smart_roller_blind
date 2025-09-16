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

#endif // 