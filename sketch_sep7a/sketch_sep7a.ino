/*
 * ESP32 + A4988 步进电机 + 红外接收控制程序
 * 适用于42步进电机 - 窗帘控制系统
 * 
 * 连接:
 * ESP32 GPIO 16 -> A4988 STEP
 * ESP32 GPIO 17 -> A4988 DIR
 * ESP32 GPIO 18 -> A4988 ENABLE (可选)
 * ESP32 GPIO 19 -> 红外接收模块 OUT
 * ESP32 3.3V    -> A4988 VDD, 红外接收模块 VCC
 * ESP32 GND     -> A4988 GND, 红外接收模块 GND
 * 12V电源       -> A4988 VMOT
 * 12V电源GND    -> A4988 GND
 * 
 * 红外遥控器按键功能 (基于实际命令码):
 * 上键 (0x1) - 打开窗帘 (顺时针转动指定时间)
 * 下键 (0x9) - 关闭窗帘 (逆时针转动指定时间)
 * 左键 (0x4) - 顺时针转动90度
 * 右键 (0x6) - 逆时针转动90度
 * 设置键 (0xE) - 进入设置模式开始计时关闭 / 停止计时保存时间
 * 0键 (0x0) - 停止电机 / 连续3次清除存储
 */

 #include <IRremote.h>
#include <EEPROM.h>

 // 引脚定义
 #define STEP_PIN 16
 #define DIR_PIN 17
 #define ENABLE_PIN 18
 #define IR_RECEIVE_PIN 19
 
 // 电机参数
 const int STEPS_PER_REVOLUTION = 200;  // 42步进电机全步进
 const int MICROSTEPS = 16;  // 1/16微步进
 const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;  // 3200步/圈
 
// 速度设置
int speed = 300;  // 步/秒 (降低速度以获得最大扭矩，防止窗帘打滑)

// 窗帘控制时间设置
int CURTAIN_TIME = 5000;  // 窗帘开关时间 (毫秒) - 可修改
const int DEFAULT_CURTAIN_TIME = 5000;  // 默认窗帘开关时间 (毫秒)
 
 // 红外命令定义 (基于实际接收到的命令码)
 #define IR_KEY_UP 0x1
 #define IR_KEY_DOWN 0x9
 #define IR_KEY_LEFT 0x4
 #define IR_KEY_RIGHT 0x6
 #define IR_KEY_SHUTDOWN 0x0
 #define IR_KEY_SET 0xE
// 电机状态
bool motorRunning = false;
bool stopRequested = false;  // 停止请求标志

// 设置模式状态
bool setMode = false;           // 是否处于设置模式
bool timingInProgress = false;  // 是否正在计时
unsigned long timingStart = 0;  // 计时开始时间
int shutdownCount = 0;          // 连续关机按键计数
unsigned long lastShutdownTime = 0; // 上次关机按键时间


// 红外接收状态
uint32_t lastIRCommand = 0;
unsigned long lastIRTime = 0;
const unsigned long IR_DEBOUNCE_TIME = 200; // 200ms防抖时间
 
 void setup() {
   Serial.begin(115200);
   Serial.println("ESP32 + A4988 步进电机 + 红外控制");
   
   // 设置引脚
   pinMode(STEP_PIN, OUTPUT);
   pinMode(DIR_PIN, OUTPUT);
   pinMode(ENABLE_PIN, OUTPUT);
   
   // 初始状态
   digitalWrite(STEP_PIN, LOW);
   digitalWrite(DIR_PIN, LOW);
   digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机（待机状态）
   
  // 初始化红外接收 (0038k模块)
  IrReceiver.begin(IR_RECEIVE_PIN, false);
  
  // 读取存储的窗帘时间
  loadCurtainTime();
  
  Serial.println("电机已启用，0038k红外接收模块已初始化");
  Serial.println("等待红外命令...");
  printIRCommands();
   
  // 测试0038k红外接收模块
  Serial.println("正在测试0038k红外接收模块...");
  delay(1000);
  
  // 测试电机
  Serial.println("开始电机测试...");
  // testMotor();
  delay(2000);
 }
 
void loop() {
  // 检查红外信号（设置模式下减少处理频率）
  if (IrReceiver.decode()) {
    uint32_t command = IrReceiver.decodedIRData.command;
    uint32_t address = IrReceiver.decodedIRData.address;
    uint8_t protocol = IrReceiver.decodedIRData.protocol;
    unsigned long currentTime = millis();
    
    // 在设置模式下，只处理关键命令，减少干扰
    if (setMode) {
      // 设置模式下只处理SET键和SHUT_DOWN键
      if ((command == IR_KEY_SET || command == IR_KEY_SHUTDOWN) && 
          command != 0 && command != 0xFFFFFFFF && command != 0x0 &&
          address != 0x0 && protocol != 0) {
        
        // 应用防重复机制（设置模式下使用更长的防抖时间）
        const unsigned long SET_MODE_DEBOUNCE_TIME = 1000; // 设置模式防抖时间：1秒
        if (command != lastIRCommand || (currentTime - lastIRTime) > SET_MODE_DEBOUNCE_TIME) {
          Serial.print("设置模式接收到命令: 0x");
          Serial.println(command, HEX);
          handleIRCommand(command);
          lastIRCommand = command;
          lastIRTime = currentTime;
        } else {
          Serial.println("设置模式：重复命令，已忽略");
        }
      }
      // 忽略其他所有信号，减少干扰
      IrReceiver.resume();
      return;
    }
    
    // 非设置模式下的正常处理
    // 显示接收到的原始数据（用于调试）
    Serial.print("0038k接收 - 协议:");
    Serial.print(protocol);
    Serial.print(" 地址:0x");
    Serial.print(address, HEX);
    Serial.print(" 命令:0x");
    Serial.println(command, HEX);
    
    // 验证信号有效性
    if (command != 0 && command != 0xFFFFFFFF && command != 0x0) {
      // 如果电机正在运行，立即处理停止命令，跳过防重复检查
      if (motorRunning && command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
        Serial.println("电机运行时接收到有效停止命令，立即处理");
        handleIRCommand(command);
        lastIRCommand = command;
        lastIRTime = currentTime;
      }
      // 防重复命令（仅在电机不运行时）
      else if (command != lastIRCommand || (currentTime - lastIRTime) > IR_DEBOUNCE_TIME) {
        handleIRCommand(command);
        lastIRCommand = command;
        lastIRTime = currentTime;
      } else {
        Serial.println("重复命令，已忽略");
      }
    } else {
      Serial.println("无效信号，已忽略");
    }
    
    IrReceiver.resume(); // 准备接收下一个信号
  }
  
  // 检查设置模式下的持续转动
  if (setMode && timingInProgress && motorRunning) {
    // 设置模式下使用与正常使用相同的转速，确保时间设置准确
    unsigned long stepDelay = 1000000 / speed;
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);  // 标准脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 5);
    
    // 添加步数计数，用于调试
    static int totalSteps = 0;
    totalSteps++;
    
    // 每5秒显示一次计时状态
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
      unsigned long elapsed = millis() - timingStart;
      Serial.print("设置模式 - 已计时: ");
      Serial.print(elapsed);
      Serial.print(" 毫秒 (转速: ");
      Serial.print(speed);
      Serial.print(" 步/秒, 总步数: ");
      Serial.print(totalSteps);
      Serial.println(" - 最大扭矩模式)");
      lastStatusTime = millis();
    }
    
    // 检查红外信号，允许SHUT_DOWN按键打断（减少检查频率）
    static int stepCount = 0;
    stepCount++;
    if (stepCount % 500 == 0 && IrReceiver.decode()) {  // 每500步检查一次，减少干扰
      uint32_t command = IrReceiver.decodedIRData.command;
      uint32_t address = IrReceiver.decodedIRData.address;
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      
       // 只响应有效的SHUT_DOWN信号
       if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
         Serial.println("设置模式下接收到停止信号，退出设置模式");
         digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机
         setMode = false;
         timingInProgress = false;
         motorRunning = false;
         stepCount = 0;
         IrReceiver.resume();
         return;
       }
      IrReceiver.resume();
    }
  }
  
  // 只在非设置模式下添加延迟，避免影响电机转动
  if (!setMode || !timingInProgress) {
    delay(50); // 短暂延迟，避免过度占用CPU
  }
}
 
// 顺时针转动指定角度
void rotateClockwise(int degrees) {
  digitalWrite(ENABLE_PIN, LOW);  // 启用电机
  digitalWrite(DIR_PIN, HIGH);  // 设置方向
  
  int steps = (degrees * TOTAL_STEPS) / 360;
  unsigned long stepDelay = 1000000 / speed;  // 微秒
  
  for (int i = 0; i < steps && !stopRequested; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);  // 减少脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 5);  // 调整延迟
    
    // 每50步检查一次红外信号（减少检查频率提高流畅性）
    if (i % 50 == 0 && IrReceiver.decode()) {
      uint32_t command = IrReceiver.decodedIRData.command;
      uint32_t address = IrReceiver.decodedIRData.address;
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      
      // 只有在接收到有效的停止信号时才停止（地址不为0x0，协议不为0）
      if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
        Serial.println("旋转过程中接收到有效停止信号，立即停止");
        stopRequested = true;
        IrReceiver.resume();
        break;
      }
      IrReceiver.resume();
    }
  }
  
  // 重置停止请求标志
  stopRequested = false;
}
 
// 逆时针转动指定角度
void rotateCounterClockwise(int degrees) {
  digitalWrite(ENABLE_PIN, LOW);  // 启用电机
  digitalWrite(DIR_PIN, LOW);  // 设置方向
  
  int steps = (degrees * TOTAL_STEPS) / 360;
  unsigned long stepDelay = 1000000 / speed;  // 微秒
  
  for (int i = 0; i < steps && !stopRequested; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);  // 减少脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 5);  // 调整延迟
    
    // 每50步检查一次红外信号（减少检查频率提高流畅性）
    if (i % 50 == 0 && IrReceiver.decode()) {
      uint32_t command = IrReceiver.decodedIRData.command;
      uint32_t address = IrReceiver.decodedIRData.address;
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      
      // 只有在接收到有效的停止信号时才停止（地址不为0x0，协议不为0）
      if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
        Serial.println("旋转过程中接收到有效停止信号，立即停止");
        stopRequested = true;
        IrReceiver.resume();
        break;
      }
      IrReceiver.resume();
    }
  }
  
  // 重置停止请求标志
  stopRequested = false;
}
 
// 转动指定时长 (毫秒)
void rotateForTime(int duration, bool clockwise) {
  digitalWrite(ENABLE_PIN, LOW);  // 启用电机
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  
  unsigned long startTime = millis();
  unsigned long stepDelay = 1000000 / speed;
  int stepCount = 0;
  
  Serial.print("开始旋转，持续时间: ");
  Serial.print(duration);
  Serial.print(" 毫秒，方向: ");
  Serial.println(clockwise ? "顺时针" : "逆时针");
  Serial.print("步进延迟: ");
  Serial.print(stepDelay);
  Serial.println(" 微秒");
  
  while (millis() - startTime < duration && !stopRequested) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);  // 减少脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 5);  // 调整延迟
    stepCount++;
    
    // 每2000步打印一次进度（减少打印频率）
    if (stepCount % 2000 == 0) {
      unsigned long elapsed = millis() - startTime;
      Serial.print("已执行 ");
      Serial.print(stepCount);
      Serial.print(" 步，用时 ");
      Serial.print(elapsed);
      Serial.println(" 毫秒");
    }
    
    // 每100步检查一次红外信号（减少检查频率提高流畅性）
    if (stepCount % 100 == 0 && IrReceiver.decode()) {
      uint32_t command = IrReceiver.decodedIRData.command;
      uint32_t address = IrReceiver.decodedIRData.address;
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      
      // 只有在接收到有效的停止信号时才停止（地址不为0x0，协议不为0）
      if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
        Serial.println("旋转过程中接收到有效停止信号，立即停止");
        stopRequested = true;
        IrReceiver.resume();
        break;
      }
      IrReceiver.resume();
    }
  }
  
  unsigned long actualDuration = millis() - startTime;
  Serial.print("旋转完成，总步数: ");
  Serial.print(stepCount);
  Serial.print("，实际持续时间: ");
  Serial.print(actualDuration);
  Serial.print(" 毫秒，停止原因: ");
  Serial.println(stopRequested ? "被中断" : "时间到");
}
 
 // 设置速度
 void setSpeed(int newSpeed) {
    speed = constrain(newSpeed, 50, 500);  // 限制速度范围，最大扭矩设置
   Serial.print("速度设置为: ");
   Serial.print(speed);
   Serial.println(" 步/秒");
 }
 
// 打印红外命令说明
void printIRCommands() {
  Serial.println("红外遥控器按键功能:");
  Serial.print("上键 (0x1) - 窗帘卷起");
  Serial.print(CURTAIN_TIME);
  Serial.println("毫秒)");
  Serial.print("下键 (0x9) - 窗帘放下");
  Serial.print(CURTAIN_TIME);
  Serial.println("毫秒)");
  Serial.println("左键 (0x4) - 窗帘卷起转动45度");
  Serial.println("右键 (0x6) - 窗帘放下转动45度");
  Serial.println("设置键 (0xE) - 进入设置模式开始计时关闭 / 停止计时保存时间");
  Serial.println("0键 (0x0) - 停止电机 / 连续3次清除存储");
  Serial.println("-------------------");
}
 
// 处理红外命令
void handleIRCommand(uint32_t command) {
  Serial.print("接收到有效红外命令: 0x");
  Serial.println(command, HEX);
  
  switch (command) {
    case IR_KEY_UP:
      Serial.println("执行: 窗帘卷起");
      rollUpCurtain();
      break;
      
    case IR_KEY_DOWN:
      Serial.println("执行: 窗帘放下");
  
      layDownCurtain();
      break;
      
    case IR_KEY_LEFT:
      Serial.println("执行: 顺时针转动45度（流畅模式）");
      {
        int steps = (45 * TOTAL_STEPS) / 360;
        smoothRotate(steps, false);
      }
      break;
      
    case IR_KEY_RIGHT:
      Serial.println("执行: 逆时针转动45度（流畅模式）");
     
      {
        int steps = (45 * TOTAL_STEPS) / 360;
        smoothRotate(steps, true);
      }
      break;
      
    case IR_KEY_SET:
      handleSetKey();
      break;
      
    case IR_KEY_SHUTDOWN:
      handleShutdownKey();
      break;
      
    default:
      Serial.print("未知命令: 0x");
      Serial.print(command, HEX);
      Serial.println(" - 请使用正确的按键或检查遥控器");
      break;
  }
}
 
// 停止电机
void stopMotor() {
  digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机
  motorRunning = false;
  Serial.println("电机已停止");
  delay(100);
  // 注意：不重新启用电机，保持待机状态
}

// 打开窗帘
void rollUpCurtain() {
  Serial.println("=== 开始卷起窗帘 ===");
  Serial.print("当前速度设置: ");
  Serial.print(speed);
  Serial.println(" 步/秒");
  
  motorRunning = true;
  stopRequested = false;  // 重置停止标志
  
  Serial.print("调用 rotateForTime(");
  Serial.print(CURTAIN_TIME);
  Serial.println(", true)");
  // 顺时针转动指定时间，打开窗帘
  rotateForTime(CURTAIN_TIME, false);
  
  Serial.println("rotateForTime 函数返回");
  motorRunning = false;
  
  Serial.print("stopRequested 状态: ");
  Serial.println(stopRequested ? "true" : "false");
  
  if (!stopRequested) {
    Serial.println("窗帘已卷起");
  } else {
    Serial.println("窗帘卷起被中断");
  }
  
  // 重置停止请求标志
  stopRequested = false;
  Serial.println("=== 卷起窗帘完成 ===");
}

// 放下窗帘
void layDownCurtain() {
  Serial.println("正在放下窗帘...");
  motorRunning = true;
  stopRequested = false;  // 重置停止标志
  
  // 逆时针转动指定时间，关闭窗帘
  rotateForTime(CURTAIN_TIME, true);
  
  motorRunning = false;
  if (!stopRequested) {
    Serial.println("窗帘已放下");
  } else {
    Serial.println("窗帘放下被中断");
  }
  
  // 重置停止请求标志
  stopRequested = false;
  Serial.println("=== 放下窗帘完成 ===");
}
 

// 处理设置键
void handleSetKey() {
  if (!setMode) {
    // 进入设置模式并开始计时关闭
    Serial.println("=== 进入设置模式 ===");
    Serial.print("当前转速: ");
    Serial.print(speed);
    Serial.println(" 步/秒 (最大扭矩设置，与正常使用转速一致)");
    Serial.println("开始计时关闭窗帘，请观察窗帘完全关闭后按设置键停止");
    
    setMode = true;
    timingInProgress = true;
    timingStart = millis();
    
     // 开始关闭窗帘
     digitalWrite(ENABLE_PIN, LOW);  // 启用电机
     digitalWrite(DIR_PIN, HIGH);  // 顺时针
     motorRunning = true;
    
    Serial.println("设置模式：电机已启动，开始持续转动");
    Serial.print("方向: 顺时针 (DIR_PIN=HIGH), 转速: ");
    Serial.print(speed);
    Serial.println(" 步/秒");
    
  } else {
    // 在设置模式中，停止计时
    Serial.println("第二次按下SET键，停止计时并保存时间");
    unsigned long duration = millis() - timingStart;
    Serial.print("计时结束，窗帘关闭时间: ");
    Serial.print(duration);
    Serial.println(" 毫秒");
    
     // 停止电机
     digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机
     motorRunning = false;
     timingInProgress = false;
    
    // 保存时间
    CURTAIN_TIME = duration;
    saveCurtainTime();
    
    // 退出设置模式
    setMode = false;
    Serial.println("=== 退出设置模式 ===");
    Serial.print("新的窗帘时间已保存: ");
    Serial.print(CURTAIN_TIME);
    Serial.println(" 毫秒");
  }
}

// 处理关机键
void handleShutdownKey() {
  unsigned long currentTime = millis();
  
   // 检查是否在设置模式
   if (setMode) {
     Serial.println("设置模式下按关机键，退出设置模式");
     digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机
     setMode = false;
     timingInProgress = false;
     motorRunning = false;
     return;
   }
  
  // 检查连续按键
  if (currentTime - lastShutdownTime < 2000) { // 2秒内
    shutdownCount++;
  } else {
    shutdownCount = 1; // 重新开始计数
  }
  
  lastShutdownTime = currentTime;
  
  if (shutdownCount >= 3) {
    // 连续3次，清除存储
    Serial.println("连续3次关机键，清除存储的时间设置");
    clearCurtainTime();
    shutdownCount = 0;
  } else {
    // 正常停止电机
    Serial.println("执行: 停止电机");
    stopRequested = true;
    stopMotor();
  }
}

// 保存窗帘时间到EEPROM
void saveCurtainTime() {
  EEPROM.begin(512);
  EEPROM.put(0, CURTAIN_TIME);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("窗帘时间已保存到EEPROM");
}

// 从EEPROM读取窗帘时间
void loadCurtainTime() {
  EEPROM.begin(512);
  int savedTime;
  EEPROM.get(0, savedTime);
  EEPROM.end();
  
  if (savedTime > 0 && savedTime < 60000) { // 合理范围检查
    CURTAIN_TIME = savedTime;
    Serial.print("从EEPROM读取窗帘时间: ");
    Serial.print(CURTAIN_TIME);
    Serial.println(" 毫秒");
  } else {
    CURTAIN_TIME = DEFAULT_CURTAIN_TIME;
    Serial.print("使用默认窗帘时间: ");
    Serial.print(CURTAIN_TIME);
    Serial.println(" 毫秒");
  }
}

// 清除存储的窗帘时间
void clearCurtainTime() {
  EEPROM.begin(512);
  EEPROM.put(0, 0); // 写入0表示无效
  EEPROM.commit();
  EEPROM.end();
  
  CURTAIN_TIME = DEFAULT_CURTAIN_TIME;
  Serial.print("已清除存储，恢复默认时间: ");
  Serial.print(CURTAIN_TIME);
  Serial.println(" 毫秒");
}

// 流畅转动函数（带加速减速）
void smoothRotate(int steps, bool clockwise) {
  digitalWrite(ENABLE_PIN, LOW);  // 启用电机
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  
  const int maxSpeed = 400;   // 最大速度 (步/秒) - 降低以获得更大扭矩
  const int minSpeed = 100;   // 最小速度 (步/秒) - 降低以获得更大扭矩
  const int accelSteps = steps / 4;  // 加速步数
  const int decelSteps = steps / 4;  // 减速步数
  
  for (int i = 0; i < steps && !stopRequested; i++) {
    // 计算当前速度
    int currentSpeed;
    if (i < accelSteps) {
      // 加速阶段
      currentSpeed = minSpeed + (maxSpeed - minSpeed) * i / accelSteps;
    } else if (i >= steps - decelSteps) {
      // 减速阶段
      currentSpeed = minSpeed + (maxSpeed - minSpeed) * (steps - i) / decelSteps;
    } else {
      // 匀速阶段
      currentSpeed = maxSpeed;
    }
    
    unsigned long stepDelay = 1000000 / currentSpeed;
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3);  // 更短的脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 3);
    
    // 每100步检查一次红外信号
    if (i % 100 == 0 && IrReceiver.decode()) {
      uint32_t command = IrReceiver.decodedIRData.command;
      uint32_t address = IrReceiver.decodedIRData.address;
      uint8_t protocol = IrReceiver.decodedIRData.protocol;
      
      if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
        Serial.println("流畅旋转过程中接收到有效停止信号，立即停止");
        stopRequested = true;
        IrReceiver.resume();
        break;
      }
      IrReceiver.resume();
    }
  }
  
  stopRequested = false;
}

// 简单电机测试函数
void testMotor() {
  Serial.println("=== 电机测试开始 ===");
  Serial.println("测试顺时针转动1秒...");
  
  digitalWrite(DIR_PIN, HIGH);  // 顺时针
  unsigned long startTime = millis();
  int stepCount = 0;
  unsigned long stepDelay = 1000000 / speed;
  
  while (millis() - startTime < 1000) {  // 1秒
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);  // 减少脉冲宽度
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 5);  // 调整延迟
    stepCount++;
  }
  
  Serial.print("1秒内执行了 ");
  Serial.print(stepCount);
  Serial.println(" 步");
  Serial.println("=== 电机测试完成 ===");
}

// 调试红外接收（可选使用）
void debugIRReceiver() {
  if (IrReceiver.decode()) {
    Serial.print("原始数据 - 协议: ");
    Serial.print(IrReceiver.decodedIRData.protocol);
    Serial.print(", 地址: 0x");
    Serial.print(IrReceiver.decodedIRData.address, HEX);
    Serial.print(", 命令: 0x");
    Serial.print(IrReceiver.decodedIRData.command, HEX);
    Serial.print(", 重复: ");
    Serial.println(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT ? "是" : "否");
    IrReceiver.resume();
  }
}