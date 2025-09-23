/*
 * ESP32 + A4988 步进电机 + 红外接收控制程序
 * 适用于42步进电机 - 窗帘控制系统
 * 
 * 红外遥控器按键功能 (基于实际命令码):
 * 上键 (0x1) - 升起窗帘 (默认:逆时针转动指定时间)
 * 下键 (0x9) - 放下窗帘 (默认:顺时针时针转动指定时间)
 * 左键 (0x4) - 默认:升起窗帘(微调)
 * 右键 (0x6) - 默认:放下转动(微调)
 * 设置键 (0xE) - 进入设置模式开始计时关闭 / 停止计时保存时间
 * 中间键 (0x5) - 停止电机 / 连续3次清除存储
 *  9键  (0x1A) - 切换电机转动方向
 */
#include "config.h"
#include <IRremote.h>
#include <EEPROM.h>
#include "BLEHandler.h"


 // 电机参数
 const int STEPS_PER_REVOLUTION = 200;  // 42步进电机全步进
 const int MICROSTEPS = 16;  // 1/16微步进
 const int TOTAL_STEPS = STEPS_PER_REVOLUTION * MICROSTEPS;  // 3200步/圈
 
// 速度设置
int speed = 600;  // 步/秒 (流畅转动)

// 窗帘控制时间设置
int currentCurtainTime = DEFAULT_CURTAIN_TIME;  // 窗帘开关时间 (毫秒) - 可修改

//方向设置（可动态切换）
bool isNormalDirection = true;  // 默认方向

// 电机状态
bool motorRunning = false;
volatile bool stopRequested = false;  // 停止请求标志

// 窗帘状态标志位
bool isFullyRolledUp = false;    // 是否完全升起
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

// BLE处理器
BLEHandler* bleHandler = nullptr;

// BLE命令队列（用于非阻塞处理）
volatile bool bleStopRequested = false;
volatile bool bleUpRequested = false;
volatile bool bleDownRequested = false;
volatile bool bleLeftRequested = false;
volatile bool bleRightRequested = false;
volatile bool bleSetRequested = false;
volatile bool bleDirectionRequested = false;
 
 // 辅助函数：重置窗帘状态
void resetCurtainState() {
  isFullyRolledUp = false;
  isFullyRolledDown = false;
  stopRequested = false;
}

// 辅助函数：获取方向参数
bool getDirection(bool isUp) {
  if (isUp) {
    return isNormalDirection; // 升起方向
  } else {
    return !isNormalDirection; // 放下方向
  }
}

// 辅助函数：验证红外命令有效性
bool isValidIRCommand(uint32_t command, uint8_t protocol) {
  return command != 0xFFFFFFFF && protocol != 0;
}

// 辅助函数：判断是否应该处理命令（防抖机制）
bool shouldProcessCommand(uint32_t command, unsigned long currentTime, unsigned long debounceTime) {
  return command != lastIRCommand || (currentTime - lastIRTime) > debounceTime;
}

// 辅助函数：更新红外状态
void updateIRState(uint32_t command, unsigned long currentTime) {
  lastIRCommand = command;
  lastIRTime = currentTime;
}

// 设置模式旋转处理函数
void handleSetModeRotation() {
  // 确保电机已启用
  digitalWrite(ENABLE_PIN, LOW);
  
  // 使用与rotateForTime完全相同的步进控制逻辑
  static unsigned long stepDelay = 1000000 / speed;
  static unsigned long lastStepTime = 0;
  static bool stepState = false;
  static int totalSteps = 0;
  static unsigned long lastSetModeTime = 0;
  static unsigned long startTime = 0;
  static bool directionSet = false;
  
  // 每次进入设置模式时重新初始化所有变量
  if (timingStart != lastSetModeTime) {
    stepState = false;
    lastStepTime = 0;
    totalSteps = 0;
    lastSetModeTime = timingStart;
    startTime = millis();
    directionSet = false;
  }
  
  // 设置方向（只设置一次）
  if (!directionSet) {
    digitalWrite(DIR_PIN, getDirection(false) ? HIGH : LOW); // 放下方向
    directionSet = true;
  }
  
  // 检查停止标志
  if (bleStopRequested || stopRequested) {
    exitSetMode();
    return;
  }
  
  // 步进控制逻辑
  if (millis() - lastStepTime >= stepDelay / 1000) {
    if (!stepState) {
      digitalWrite(STEP_PIN, HIGH);
      stepState = true;
    } else {
      digitalWrite(STEP_PIN, LOW);
      stepState = false;
      lastStepTime = millis();
      totalSteps++;
    }
  }
  
  delay(1); // 让出CPU时间
  
  // 每1秒显示一次计时状态
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 1000) {
    printSetModeStatus(millis() - startTime, totalSteps, stepDelay);
    lastStatusTime = millis();
  }
}

// 退出设置模式
void exitSetMode() {
  unsigned long duration = millis() - timingStart;
  Serial.print("设置模式结束，窗帘关闭时间: ");
  Serial.print(duration);
  Serial.println(" 毫秒");
  
  // 停止电机
  digitalWrite(ENABLE_PIN, HIGH);
  motorRunning = false;
  timingInProgress = false;
  setMode = false;
  
  Serial.print("exitSetMode() 设置 setMode = ");
  Serial.println(setMode ? "true" : "false");
  
  // 保存时间
  currentCurtainTime = duration;
  saveCurtainTime();
  isFullyRolledDown = true;
  isFullyRolledUp = false;
  
  // 清除停止标志
  stopRequested = false;
  bleStopRequested = false;
  
  Serial.println("=== 退出设置模式 ===");
}

// 打印设置模式状态
void printSetModeStatus(unsigned long elapsed, int totalSteps, unsigned long stepDelay) {
  Serial.print("设置模式 - 已计时: ");
  Serial.print(elapsed);
  Serial.print(" 毫秒 (转速: ");
  Serial.print(speed);
  Serial.print(" 步/秒, 总步数: ");
  Serial.print(totalSteps);
  Serial.print(", 步进延迟: ");
  Serial.print(stepDelay);
  Serial.println(" 微秒)");
}

void setup() {
   Serial.begin(9600);
   Serial.println("ESP32 + A4988 步进电机 + 红外控制");
   
   // 设置引脚
   pinMode(STEP_PIN, OUTPUT);
   pinMode(DIR_PIN, OUTPUT);
   pinMode(ENABLE_PIN, OUTPUT);
   
   // 初始状态
   digitalWrite(STEP_PIN, LOW);
   digitalWrite(DIR_PIN, LOW);
   digitalWrite(ENABLE_PIN, HIGH);  // 禁用电机（待机状态）
  // 读取存储的窗帘时间
  loadCurtainTime();
  
  // 读取存储的方向设置
  loadDirectionSetting();
  // 初始化红外接收 (0038k模块)
  IrReceiver.begin(IR_RECEIVE_PIN, false);
  
  // 初始化BLE
  bleHandler = new BLEHandler();
  bleHandler->init();
  bleHandler->setCommandCallbacks(
    handleBLEUpCommand,      // 升起窗帘
    handleBLEDownCommand,    // 放下窗帘
    handleBLELeftCommand,    // 微调升起
    handleBLERightCommand,   // 微调放下
    handleBLEStopCommand,    // 停止电机
    handleBLESetCommand,     // 设置模式
    handleBLEDirectionCommand // 切换方向
  );
  bleHandler->startAdvertising();
  
  // 初始化窗帘状态标志位
  isFullyRolledUp = false;
  isFullyRolledDown = false;
  
  Serial.println("电机已启用，0038k红外接收模块已初始化");
  Serial.println("BLE已初始化并开始广播");
  Serial.println("等待红外命令或BLE命令...");
  printIRCommands();
   

 }
 
void loop() {
  // 每5秒打印一次BLE状态（用于调试）
  static unsigned long lastBLEStatusTime = 0;
  if (millis() - lastBLEStatusTime > 5000) {
    Serial.println(bleHandler != nullptr ? "BLE处理器状态正常" : "BLE处理器为空！");
    lastBLEStatusTime = millis();
  }
  
  // 处理BLE命令队列（优先级最高）
  if (bleStopRequested) {
    bleStopRequested = false;
    stopRequested = true;
    stopMotor();
  }
  
  if (bleUpRequested) {
    bleUpRequested = false;
    if (!isFullyRolledUp) {
      resetCurtainState();
      rollUpCurtain();
    }
  }
  
  if (bleDownRequested) {
    bleDownRequested = false;
    if (!isFullyRolledDown) {
      resetCurtainState();
      layDownCurtain();
    }
  }
  
  if (bleLeftRequested) {
    bleLeftRequested = false;
    resetCurtainState();
    rotateForTime(SIDE_KEY_TIME, getDirection(true)); // 微调升起
  }
  
  if (bleRightRequested) {
    bleRightRequested = false;
    resetCurtainState();
    rotateForTime(SIDE_KEY_TIME, getDirection(false)); // 微调放下
  }
  
  if (bleSetRequested) {
    bleSetRequested = false;
    handleSetKey();
  }
  
  if (bleDirectionRequested) {
    bleDirectionRequested = false;
    handleDirectionKey();
  }
  
  // 检查红外信号
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
    
    // 验证地址
    if (address != IR_ADDRESS) {
      Serial.println("address not equals,return");
      IrReceiver.resume();
      return;
    }

    unsigned long currentTime = millis();
    
    // 在设置模式下，只处理关键命令
    if (setMode) {
      if (isValidIRCommand(command, protocol)) {
        if (shouldProcessCommand(command, currentTime, 200)) { // 设置模式防抖时间：200ms
          Serial.print("设置模式接收到命令: 0x");
          Serial.println(command, HEX);
          handleIRCommand(command);
          updateIRState(command, currentTime);
        } else {
          Serial.println("设置模式：重复命令，已忽略");
        }
      }
      IrReceiver.resume();
      return;
    }
    
    // 非设置模式下的正常处理
    if (isValidIRCommand(command, protocol)) {
      // 电机运行时立即处理停止命令，否则使用防抖机制
      if (motorRunning && command == IR_KEY_SHUTDOWN) {
        Serial.println("电机运行时接收到有效停止命令，立即处理");
        handleIRCommand(command);
        updateIRState(command, currentTime);
      } else if (shouldProcessCommand(command, currentTime, IR_DEBOUNCE_TIME)) {
        handleIRCommand(command);
        updateIRState(command, currentTime);
      } else {
        Serial.println("重复命令，已忽略");
      }
    } else {
      Serial.println("无效信号，已忽略");
    }
    
    IrReceiver.resume();
  }
  
  // 检查设置模式下的持续转动
  if (setMode && timingInProgress && motorRunning) {
    handleSetModeRotation();
  }
  
  // 调试输出：每10秒显示一次状态
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 10000) {
    Serial.print("调试信息 - setMode: ");
    Serial.print(setMode ? "true" : "false");
    Serial.print(", timingInProgress: ");
    Serial.print(timingInProgress ? "true" : "false");
    Serial.print(", motorRunning: ");
    Serial.println(motorRunning ? "true" : "false");
    lastDebugTime = millis();
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

// 转动指定时长 (毫秒) - 非阻塞版本
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
  unsigned long lastStepTime = 0;
  
  Serial.print("开始旋转，持续时间: ");
  Serial.print(duration);
  Serial.print(" 毫秒，方向: ");
  Serial.println(clockwise ? "顺时针" : "逆时针");
  Serial.print("步进延迟: ");
  Serial.print(stepDelay);
  Serial.println(" 微秒");
  
  // 完全非阻塞方式：使用状态机而不是while循环
  bool stepState = false;  // 移除static，每次调用函数时重新初始化
  while (millis() - startTime < duration) {
    // 每次循环都检查停止标志（提高响应速度）
    if (bleStopRequested) {
      Serial.println("旋转过程中接收到BLE停止命令，立即停止");
      bleStopRequested = false;  // 清除标志
      break;
    }
    
    if (stopRequested) {
      Serial.println("旋转过程中接收到停止命令，立即停止");
      break;
    }
    
    // 检查是否到了执行下一步的时间
    if (millis() - lastStepTime >= stepDelay / 1000) {  // 转换为毫秒
      // 执行步进脉冲（非阻塞）
      if (!stepState) {
        digitalWrite(STEP_PIN, HIGH);
        stepState = true;
      } else {
        digitalWrite(STEP_PIN, LOW);
        stepState = false;
        lastStepTime = millis();
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
        
        // 每10步检查一次红外信号（减少检查频率提高流畅性）
        if (stepCount % 10 == 0) {
          // 检查红外信号
          if (IrReceiver.decode()) {
            uint32_t command = IrReceiver.decodedIRData.command;
            uint32_t address = IrReceiver.decodedIRData.address;
            uint8_t protocol = IrReceiver.decodedIRData.protocol;
            
            // 只有在接收到有效的停止信号时才停止（地址不为0x0，协议不为0）
            if (command == IR_KEY_SHUTDOWN && address != 0x0 && protocol != 0) {
              Serial.println("旋转过程中接收到有效红外停止信号，立即停止");
              stopRequested = true;
              IrReceiver.resume();
              break;
            }
            IrReceiver.resume();
          }
        }
      }
    }
    
    // 让出CPU时间给其他任务（包括BLE回调）
    delay(1);  // 1毫秒延迟，让BLE回调有机会执行
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
  Serial.print("上键 (0x1) - 窗帘升起");
  Serial.print(currentCurtainTime);
  Serial.println("毫秒)");
  Serial.print("下键 (0x9) - 窗帘放下");
  Serial.print(currentCurtainTime);
  Serial.println("毫秒)");
  Serial.println("左键 (0x4) - 微调升起");
  Serial.println("右键 (0x6) - 微调放下");
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
        Serial.println("窗帘已经完全升起，停止动作");
        break;
      }
      Serial.println("执行: 窗帘升起");
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
      resetCurtainState();
      rotateForTime(SIDE_KEY_TIME, getDirection(true)); // 微调升起
      break;
    }
      
    case IR_KEY_RIGHT: {
      Serial.println("执行: 微调放下（丝滑模式）");
      resetCurtainState();
      rotateForTime(SIDE_KEY_TIME, getDirection(false)); // 微调放下
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
  
  // 如果在设置模式下停止，需要退出设置模式
  if (setMode) {
    exitSetMode();
  }
  
  Serial.println("电机已停止");
  delay(100);
  // 注意：不重新启用电机，保持待机状态
}

// 打开窗帘
void rollUpCurtain() {
  Serial.println("=== 开始升起窗帘 ===");
  Serial.print("当前速度设置: ");
  Serial.print(speed);
  Serial.println(" 步/秒");
  
  motorRunning = true;
  stopRequested = false;  // 重置停止标志
  
  rotateForTime(currentCurtainTime, getDirection(true));
  
  motorRunning = false;
  if (!stopRequested) {
    Serial.println("窗帘已升起");
    isFullyRolledUp = true;
    isFullyRolledDown = false;
  } else {
    Serial.println("窗帘升起被中断");
  }
  
  stopRequested = false;
  Serial.println("=== 升起窗帘完成 ===");
}

// 放下窗帘
void layDownCurtain() {
  motorRunning = true;
  stopRequested = false;
  rotateForTime(currentCurtainTime, getDirection(false));
  
  motorRunning = false;
  if (!stopRequested) {
    Serial.println("窗帘已放下");
    isFullyRolledDown = true;
    isFullyRolledUp = false;
  } else {
    Serial.println("窗帘放下被中断");
  }
  
  stopRequested = false;
  Serial.println("=== 放下窗帘完成 ===");
}
 

// 处理设置键
void handleSetKey() {
  Serial.print("handleSetKey() 被调用，当前 setMode = ");
  Serial.println(setMode ? "true" : "false");
  
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
    
    // 清除停止标志
    stopRequested = false;
    bleStopRequested = false;
    
     // 开始关闭窗帘（放下方向）
     digitalWrite(ENABLE_PIN, LOW);  // 启用电机
     digitalWrite(DIR_PIN, getDirection(false) ? HIGH : LOW);  // 放下方向
     motorRunning = true;
    
    Serial.println("设置模式：电机已启动，开始持续转动");
    Serial.print("方向: 放下方向, 转速: ");
    Serial.print(speed);
    Serial.println(" 步/秒");
    
  } else {
    // 在设置模式中，设置停止标志（使用和BLE停止相同的方式）
    Serial.println("第二次按下SET键，设置停止标志");
    stopRequested = true;
    bleStopRequested = true;  // 同时设置BLE停止标志，确保rotateForTime能检测到
  }
}

// 处理关机键
void handleShutdownKey() {
  unsigned long currentTime = millis();
  
  // 检查是否在设置模式
  if (setMode) {
    Serial.println("设置模式下按关机键，退出设置模式");
    exitSetMode();
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
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_CURTAIN_TIME_ADDR, currentCurtainTime);
  EEPROM.commit();
  EEPROM.end();
  Serial.println("窗帘时间已保存到EEPROM");
}

// 从EEPROM读取窗帘时间
void loadCurtainTime() {
  EEPROM.begin(EEPROM_SIZE);
  int savedTime;
  EEPROM.get(EEPROM_CURTAIN_TIME_ADDR, savedTime);
  EEPROM.end();
  
  if (savedTime > 0 && savedTime < 60000) { // 合理范围检查
    currentCurtainTime = savedTime;
    Serial.print("从EEPROM读取窗帘时间: ");
    Serial.print(currentCurtainTime);
    Serial.println(" 毫秒");
  } else {
    currentCurtainTime = DEFAULT_CURTAIN_TIME;
    Serial.print("使用默认窗帘时间: ");
    Serial.print(currentCurtainTime);
    Serial.println(" 毫秒");
  }
}

// 清除存储的窗帘时间
void clearCurtainTime() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_CURTAIN_TIME_ADDR, 0); // 写入0表示无效
  EEPROM.commit();
  EEPROM.end();
  
  currentCurtainTime = DEFAULT_CURTAIN_TIME;
  Serial.print("已清除存储，恢复默认时间: ");
  Serial.print(currentCurtainTime);
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
  isFullyRolledUp = false;    
  isFullyRolledDown = false;
  Serial.print("方向已切换为: ");
  Serial.println(isNormalDirection ? "RIGHT模式" : "LEFT模式");
  Serial.println("新的方向设置已保存到EEPROM");
}

// 保存方向设置到EEPROM
void saveDirectionSetting() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(EEPROM_DIRECTION_ADDR, isNormalDirection);  // 存储方向设置
  EEPROM.commit();
  EEPROM.end();
  Serial.println("方向设置已保存到EEPROM");
}

// 从EEPROM读取方向设置
void loadDirectionSetting() {
  EEPROM.begin(EEPROM_SIZE);
  bool savedDirection;
  EEPROM.get(EEPROM_DIRECTION_ADDR, savedDirection);
  EEPROM.end();
  
  // 检查是否是第一次运行（通过检查是否有有效的方向设置标志）
  // 使用地址5作为方向设置是否已初始化的标志
  EEPROM.begin(EEPROM_SIZE);
  bool directionInitialized;
  EEPROM.get(EEPROM_DIRECTION_INIT_FLAG_ADDR, directionInitialized);
  EEPROM.end();
  
  if (!directionInitialized) {
    // 第一次运行，使用默认方向设置并保存
    isNormalDirection = DEFAULT_IS_NORMAL_DIRECTION;  // 使用配置的默认方向
    saveDirectionSetting();
    // 标记方向设置已初始化
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(EEPROM_DIRECTION_INIT_FLAG_ADDR, true);  // 标记已初始化
    EEPROM.commit();
    EEPROM.end();
    Serial.println("首次运行，使用默认方向设置并保存");
  } else {
    // 已初始化过，使用保存的设置
    isNormalDirection = savedDirection;
  }
  
  Serial.print("从EEPROM读取方向设置: ");
  Serial.println(isNormalDirection ? "RIGHT模式" : "LEFT模式");
}

// ============================================================================
// BLE命令处理函数
// ============================================================================

void handleBLEUpCommand() {
  Serial.println("=== BLE命令: 升起窗帘 ===");
  bleUpRequested = true;
}

void handleBLEDownCommand() {
  Serial.println("=== BLE命令: 放下窗帘 ===");
  bleDownRequested = true;
}

void handleBLELeftCommand() {
  Serial.println("=== BLE命令: 微调升起 ===");
  bleLeftRequested = true;
}

void handleBLERightCommand() {
  Serial.println("=== BLE命令: 微调放下 ===");
  bleRightRequested = true;
}

void handleBLEStopCommand() {
  Serial.println("=== BLE命令: 停止电机 ===");
  bleStopRequested = true;
}

void handleBLESetCommand() {
  Serial.println("=== BLE命令: 设置模式 ===");
  bleSetRequested = true;
}

void handleBLEDirectionCommand() {
  Serial.println("=== BLE命令: 切换方向 ===");
  bleDirectionRequested = true;
}

