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
 * 上键 (0x1) - 升起窗帘 (默认:逆时针转动指定时间)
 * 下键 (0x9) - 放下窗帘 (默认:顺时针时针转动指定时间)
 * 左键 (0x4) - 默认:升起窗帘(微调)
 * 右键 (0x6) - 默认:放下转动(微调)
 * 设置键 (0xE) - 进入设置模式开始计时关闭 / 停止计时保存时间
 * 关闭键 (0x5) - 停止电机 / 连续3次清除存储
 *  0键  (0x1A) - 切换电机转动方向
 */
#include "config.h"
#include <IRremote.h>
#include <EEPROM.h>


 // 电机参数
 const int STEPS_PER_REVOLUTION = 200;  // 42步进电机全步进
 const int MICROSTEPS = 16;  // 1/16微步进
 const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;  // 3200步/圈
 
// 速度设置
int speed = 600;  // 步/秒 (流畅转动)

// 窗帘控制时间设置
int CURTAIN_TIME = DEFAULT_CURTAIN_TIME;  // 窗帘开关时间 (毫秒) - 可修改


//方向设置（可动态切换）
bool isNormalDirection = true;  // 默认方向

// 电机状态
bool motorRunning = false;
bool stopRequested = false;  // 停止请求标志

// 窗帘状态标志位
bool isFullyRolledUp = false;    // 是否完全卷起
bool isFullyRolledDown = false;  // 是否完全放下

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
  
  // 读取存储的方向设置
  loadDirectionSetting();
  
  // 初始化窗帘状态标志位
  isFullyRolledUp = false;
  isFullyRolledDown = false;
  
  Serial.println("电机已启用，0038k红外接收模块已初始化");
  Serial.println("等待红外命令...");
  printIRCommands();
   

 }
 
void loop() {
  // 检查红外信号（设置模式下减少处理频率）
  if (IrReceiver.decode()) {

    uint32_t address = IrReceiver.decodedIRData.address;
    uint32_t command = IrReceiver.decodedIRData.command;
    uint8_t protocol = IrReceiver.decodedIRData.protocol;
    // 显示接收到的原始数据（用于调试）
    Serial.print("0038k接收 - 协议:");
    Serial.print(protocol);
    Serial.print(" 地址:0x");
    Serial.print(address, HEX);
    Serial.print(" 命令:0x");
    Serial.println(command, HEX);
    if(address != IR_ADDRESS){
       IrReceiver.resume();
       return;
    }

    unsigned long currentTime = millis();
    
    // 在设置模式下，只处理关键命令，减少干扰
    if (setMode) {
      // 设置模式下只处理SET键和SHUT_DOWN键
      if ((command == IR_KEY_SET || command == IR_KEY_SHUTDOWN) && 
          command != 0xFFFFFFFF &&
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
    // 验证信号有效性
    if (command != 0xFFFFFFFF) {
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
    // 确保电机已启用
    digitalWrite(ENABLE_PIN, LOW);  // 启用电机
    
    // 设置模式下使用与正常使用相同的转速，确保时间设置准确
    unsigned long stepDelay = 1000000 / speed;
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1);  // 极短脉冲宽度，流畅
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 1);
    
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
      Serial.println(" - 流畅转动模式)");
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
  
  // 安全措施：如果不在设置模式且电机应该停止，确保电机被禁用
  if (!setMode && !motorRunning) {
    digitalWrite(ENABLE_PIN, HIGH);  // 确保电机被禁用
  }
}
 

// 转动指定时长 (毫秒)
void rotateForTime(int duration, bool clockwise) {
  Serial.print("rotateForTime函数接收参数: duration=");
  Serial.print(duration);
  Serial.print(", clockwise=");
  Serial.println(clockwise ? "true" : "false");
  
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
    delayMicroseconds(1);  // 极短脉冲宽度，流畅
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay - 1);  // 调整延迟
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
  
  // 确保电机被禁用
  digitalWrite(ENABLE_PIN, HIGH);
}

// 设置速度
  void setSpeed(int newSpeed) {
     speed = constrain(newSpeed, 200, 1000);  // 限制速度范围，流畅转动设置
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
  Serial.println("左键 (0x4) -默认:逆时针转动0.8秒");
  Serial.println("右键 (0x6) - 默认:顺时针转动0.8秒");
  Serial.println("设置键 (0xE) - 进入设置模式开始计时关闭 / 停止计时保存时间");
  Serial.println("0键 (0x0) - 停止电机 / 连续3次清除存储");
  Serial.println("方向键 (0x1A) - 切换电机转动方向");
  Serial.println("-------------------");
}
 
// 处理红外命令
void handleIRCommand(uint32_t command) {
  Serial.print("接收到有效红外命令: 0x");
  Serial.println(command, HEX);
  
  switch (command) {
    case IR_KEY_UP:
      if (isFullyRolledUp) {
        Serial.println("窗帘已经完全卷起，停止动作");
        break;
      }
      Serial.println("执行: 窗帘卷起");
      Serial.print("isNormalDirection = ");
      Serial.println(isNormalDirection ? "true" : "false");
      isFullyRolledDown = false;
      isFullyRolledUp = false;
      rollUpCurtain();
      break;
      
    case IR_KEY_DOWN:
      if (isFullyRolledDown) {
        Serial.println("窗帘已经完全放下，无响应");
        break;
      }
      isFullyRolledDown = false;
      isFullyRolledUp = false;
      Serial.println("执行: 窗帘放下");
      Serial.print("isNormalDirection = ");
      Serial.println(isNormalDirection ? "true" : "false");
      layDownCurtain();
      break;
      
    case IR_KEY_LEFT: {
      Serial.println("执行: 微调升起（丝滑模式）");
      Serial.print("isNormalDirection = ");
      Serial.print(isNormalDirection ? "true" : "false");
      // 微调时清零状态标志位
      isFullyRolledUp = false;
      isFullyRolledDown = false;
      // 微调升起逻辑：
      // 当isNormalDirection=true时，微调升起=逆时针=false
      // 当isNormalDirection=false时，微调升起=顺时针=true
      Serial.print("微调升起进入if判断前 isNormalDirection = ");
      Serial.println(isNormalDirection ? "true" : "false");
      
      bool leftDirection;
      if (isNormalDirection == true) {
        Serial.println("微调升起进入 if (isNormalDirection == true) 分支");
        leftDirection = true;  // 微调升起=逆时针
        Serial.println("微调升起设置 leftDirection = true");
      } else {
        Serial.println("微调升起进入 else 分支");
        leftDirection = false;   // 微调升起=顺时针
        Serial.println("微调升起设置 leftDirection = false");
      }
      Serial.print("微调升起方向参数: 手动计算 = ");
      Serial.println(leftDirection ? "true" : "false");
      rotateForTime(SIDE_KEY_TIME, leftDirection);
      break;
    }
      
    case IR_KEY_RIGHT: {
      Serial.println("执行: 微调放下（丝滑模式）");
      Serial.print("isNormalDirection = ");
      Serial.print(isNormalDirection ? "true" : "false");
      // 微调时清零状态标志位
      isFullyRolledUp = false;
      isFullyRolledDown = false;
      // 微调放下逻辑：
      Serial.print("微调放下进入if判断前 isNormalDirection = ");
      Serial.println(isNormalDirection ? "true" : "false");
      
      bool rightDirection;
      if (isNormalDirection == true) {
        Serial.println("微调放下进入 if (isNormalDirection == true) 分支");
        rightDirection = false;   // 微调放下=顺时针
        Serial.println("微调放下设置 rightDirection = false");
      } else {
        Serial.println("微调放下进入 else 分支");
        rightDirection = true; 
         // 微调放下=逆时针
        Serial.println("微调放下设置 rightDirection = true");
      }
      Serial.print("微调放下方向参数: 手动计算 = ");
      Serial.println(rightDirection ? "true" : "false");
      rotateForTime(SIDE_KEY_TIME, rightDirection);
      break;
    }
      
    case IR_KEY_SET:
      handleSetKey();
      break;
      
    case IR_KEY_SHUTDOWN:
      handleShutdownKey();
      break;
      
    case IR_KEY_DIRECTION:
      handleDirectionKey();
      break;
      
    default:
      Serial.print("未知命令: 0x");
      Serial.print(command, HEX);
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
  Serial.print(", ");
  Serial.print(isNormalDirection ? "true" : "false");
  Serial.println(")");
  Serial.print("isNormalDirection = ");
  Serial.print(isNormalDirection ? "true" : "false");
  Serial.print(", 传递给rotateForTime的参数 = ");
  Serial.println(isNormalDirection ? "true" : "false");
  // 卷起窗帘：使用isNormalDirection的值
  bool directionParam;
  if (isNormalDirection == true) {
    directionParam = true;
  } else {
    directionParam = false;
  }
  Serial.print("计算方向参数: 手动计算 = ");
  Serial.println(directionParam ? "true" : "false");
  Serial.print("isNormalDirection当前值: ");
  Serial.println(isNormalDirection ? "true" : "false");
  rotateForTime(CURTAIN_TIME, directionParam);
  
  Serial.println("rotateForTime 函数返回");
  motorRunning = false;
  
  Serial.print("stopRequested 状态: ");
  Serial.println(stopRequested ? "true" : "false");
  
  if (!stopRequested) {
    Serial.println("窗帘已卷起");
    isFullyRolledUp = true;    // 设置完全卷起状态
    isFullyRolledDown = false; // 清除完全放下状态
  } else {
    Serial.println("窗帘卷起被中断");
  }
  
  // 重置停止请求标志
  stopRequested = false;
  Serial.println("=== 卷起窗帘完成 ===");
}

// 放下窗帘
void layDownCurtain() {

  motorRunning = true;
  stopRequested = false;  // 重置停止标志
  bool directionParam;
  if (isNormalDirection == true) {
    directionParam = false;  
  } else {
    directionParam = true;
  }
  
  Serial.print("计算方向参数: 手动计算 = ");
  Serial.println(directionParam ? "true" : "false");
  Serial.print("isNormalDirection当前值: ");
  Serial.println(isNormalDirection ? "true" : "false");
  rotateForTime(CURTAIN_TIME, directionParam);
  
  motorRunning = false;
  if (!stopRequested) {
    Serial.println("窗帘已放下");
    isFullyRolledDown = true;  // 设置完全放下状态
    isFullyRolledUp = false;   // 清除完全卷起状态
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
    Serial.println(" 步/秒 (流畅转动设置，与正常使用转速一致)");
    Serial.println("开始计时关闭窗帘，观察窗帘完全关闭后按设置键停止");
    
    setMode = true;
    timingInProgress = true;
    timingStart = millis();
    
     // 开始关闭窗帘（放下方向）
     digitalWrite(ENABLE_PIN, LOW);  // 启用电机
     Serial.print("设置模式进入if判断前 isNormalDirection = ");
     Serial.println(isNormalDirection ? "true" : "false");
     
     bool setDirection;
     if (isNormalDirection == true) {
       Serial.println("设置模式进入 if (isNormalDirection == true) 分支");
       setDirection = false;  // 放下方向=逆时针=LOW
       Serial.println("设置模式设置 setDirection = false");
     } else {
       Serial.println("设置模式进入 else 分支");
       setDirection = true;   // 放下方向=顺时针=HIGH
       Serial.println("设置模式设置 setDirection = true");
     }
     Serial.print("设置模式方向参数: 手动计算 = ");
     Serial.println(setDirection ? "true" : "false");
     digitalWrite(DIR_PIN, setDirection ? HIGH : LOW);  // 放下方向
     motorRunning = true;
    
    Serial.println("设置模式：电机已启动，开始持续转动");
    Serial.print("方向: 放下方向, 转速: ");
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
     Serial.println("窗帘完全放下..");
    
    // 保存时间
    CURTAIN_TIME = duration;
    saveCurtainTime();
    isFullyRolledDown = true;  // 设置完全放下状态
    isFullyRolledUp = false;  
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

// 处理方向切换键
void handleDirectionKey() {
  // 切换方向
  Serial.print("切换前 isNormalDirection = ");
  Serial.println(isNormalDirection ? "true" : "false");
  
  if (isNormalDirection == true) {
    isNormalDirection = false;
    Serial.println("设置 isNormalDirection = false");
  } else {
    isNormalDirection = true;
    Serial.println("设置 isNormalDirection = true");
  }
  
  Serial.print("切换后 isNormalDirection = ");
  Serial.println(isNormalDirection ? "true" : "false");
  
  // 保存方向设置
  saveDirectionSetting();
  
  Serial.print("方向已切换为: ");
  Serial.println(isNormalDirection ? "RIGHT模式" : "LEFT模式");
  Serial.println("新的方向设置已保存到EEPROM");
}

// 保存方向设置到EEPROM
void saveDirectionSetting() {
  EEPROM.begin(512);
  EEPROM.put(4, isNormalDirection);  // 地址4存储方向设置
  EEPROM.commit();
  EEPROM.end();
  Serial.println("方向设置已保存到EEPROM");
}

// 从EEPROM读取方向设置
void loadDirectionSetting() {
  EEPROM.begin(512);
  bool savedDirection;
  EEPROM.get(4, savedDirection);
  EEPROM.end();
  
  // 检查读取的值是否有效（bool类型，任何值都有效）
  isNormalDirection = savedDirection;
  
  Serial.print("从EEPROM读取方向设置: ");
  Serial.println(isNormalDirection ? "RIGHT模式" : "LEFT模式");
}

