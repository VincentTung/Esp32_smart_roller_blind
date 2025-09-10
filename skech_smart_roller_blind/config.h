#ifndef CONFIG_H
#define CONFIG_H

 // 引脚定义
 #define STEP_PIN 16
 #define DIR_PIN 17
 #define ENABLE_PIN 18
 #define IR_RECEIVE_PIN 19
 
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
#endif // 