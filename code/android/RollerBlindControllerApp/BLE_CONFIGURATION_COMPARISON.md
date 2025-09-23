# ESP32与Android端BLE配置对比

## 统一后的配置

### ✅ 已统一的配置

| 配置项 | ESP32端 | Android端 | 状态 |
|--------|---------|-----------|------|
| **设备名称** | `"SmartRollerBlind"` | `"SmartRollerBlind"` | ✅ 统一 |
| **服务UUID** | `"12345678-1234-1234-1234-123456789abc"` | `"12345678-1234-1234-1234-123456789abc"` | ✅ 统一 |
| **控制特征UUID** | `"12345678-1234-1234-1234-123456789def"` | `"12345678-1234-1234-1234-123456789def"` | ✅ 统一 |
| **控制命令** | U, D, L, R, S, T, G | U, D, L, R, S, T, G | ✅ 统一 |

### 📋 命令对应关系

| 命令字符 | 功能 | ESP32端处理 | Android端发送 |
|----------|------|-------------|---------------|
| `U` | 升起窗帘 | `onUpCommand()` | `curtainUp()` |
| `D` | 放下窗帘 | `onDownCommand()` | `curtainDown()` |
| `L` | 微调升起 | `onLeftCommand()` | `curtainLeft()` |
| `R` | 微调放下 | `onRightCommand()` | `curtainRight()` |
| `S` | 停止电机 | `onStopCommand()` | `curtainStop()` |
| `T` | 设置模式 | `onSetCommand()` | `curtainSet()` |
| `G` | 切换方向 | `onDirectionCommand()` | `curtainDirection()` |

### 🔧 ESP32端BLE特征值配置

```cpp
// BLE服务UUID
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"

// BLE特征值UUID
#define BLE_CHARACTERISTIC_CONTROL_UUID "12345678-1234-1234-1234-123456789def"

// BLE设备名称
#define BLE_DEVICE_NAME "SmartRollerBlind"
```

### 📱 Android端BLE配置

```kotlin
// 蓝牙设备名称
const val DEVICE_NAME = "SmartRollerBlind"

// 窗帘控制器服务UUID
const val ROLLER_BLIND_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"

// 控制特征值 - 用于发送控制命令和接收状态通知
const val ROLLER_BLIND_CHARACTERISTIC_CONTROL_UUID = "12345678-1234-1234-1234-123456789def"
```

### 🚀 连接流程

1. **ESP32端启动**：
   - 初始化BLE设备
   - 创建服务和特征值
   - 开始广播

2. **Android端连接**：
   - 扫描设备（按服务UUID或设备名称）
   - 连接到ESP32设备
   - 发现服务和特征值
   - 建立通信

3. **命令发送**：
   - Android端通过控制特征值发送命令字符
   - ESP32端接收命令并执行相应动作

### ⚠️ 注意事项

1. **特征值属性**：ESP32端的控制特征值同时支持READ、WRITE和NOTIFY属性
2. **命令格式**：Android端发送单个字符命令，ESP32端接收字符串并取第一个字符
3. **连接管理**：ESP32端在断开连接后会自动重新开始广播
4. **错误处理**：ESP32端会打印未知命令，Android端有连接状态检查

### 🔍 调试信息

**ESP32端日志**：
```
BLE接收到命令: U
BLE命令: 升起窗帘
```

**Android端日志**：
```
发送BLE命令: U
执行: 升起窗帘
```

现在ESP32端和Android端的BLE配置已经完全统一，可以正常进行通信了！