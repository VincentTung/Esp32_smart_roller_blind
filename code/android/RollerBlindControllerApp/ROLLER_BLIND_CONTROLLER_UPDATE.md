# 窗帘控制器蓝牙连接更新

## 概述
已成功将RollerBlindController更新为使用与LED控制器相同的VTBLEController进行蓝牙连接，实现了稳定的BLE连接管理。

## 主要更改

### 1. 创建了新的常量定义文件
- **文件**: `RollerBlindConstants.kt`
- **内容**: 定义了窗帘控制器专用的BLE服务UUID、特征UUID、命令常量等
- **设备名称**: "SmartRollerBlind"
- **服务UUID**: "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

### 2. 重构了RollerBlindController类
- **从object改为class**: 支持单例模式，便于管理状态
- **添加了完整的BLE连接管理**: 包括初始化、连接、断开、重连等功能
- **集成了VTBLEController**: 使用与LED控制器相同的底层BLE实现
- **添加了连接状态管理**: 包括DISCONNECTED、CONNECTING、CONNECTED、ERROR状态
- **实现了所有VTBLECallback回调**: 支持MTU协商、PHY优化、配对管理等

### 3. 在Application中初始化控制器
- **更新了RollerBlindApp**: 在onCreate()中自动初始化RollerBlindController
- **创建了ControllerManager**: 提供统一的控制器实例获取方法
- **避免了重复初始化**: 确保整个应用生命周期中只有一个控制器实例

### 4. 更新了MainActivity
- **使用ControllerManager**: 从Application获取已初始化的控制器实例
- **集成了完整的连接流程**: 包括连接、重连等
- **添加了连接信息显示**: 显示MTU、PHY、连接质量等信息
- **保持了原有的UI交互**: 所有按钮功能保持不变

### 5. 新增功能

#### 连接管理
- `initBLE(context)`: 初始化BLE连接
- `connect(callback)`: 连接设备
- `disconnect()`: 断开连接
- `reconnect(callback)`: 重新连接
- `isDeviceConnected()`: 检查连接状态

#### 连接优化
- **MTU协商**: 自动协商最佳MTU大小
- **PHY优化**: 支持1M、2M、Coded PHY
- **配对管理**: 自动处理设备配对
- **连接监控**: 定期检查连接状态并自动重连

#### 信息获取
- `getCurrentMtu()`: 获取当前MTU大小
- `getCurrentTxPhy()` / `getCurrentRxPhy()`: 获取PHY信息
- `getConnectionQualityInfo()`: 获取连接质量信息
- `getBondingStatusInfo()`: 获取配对状态信息

## 使用方法

### Application中初始化（推荐）
```kotlin
// 在RollerBlindApp.onCreate()中自动初始化
// 无需手动调用initBLE()

// 在其他组件中获取控制器实例
val controller = ControllerManager.getRollerBlindController(context)

// 连接设备
controller.connect(callback)

// 控制窗帘
controller.curtainUp { success ->
    // 处理结果
}
```

### 传统方式（不推荐）
```kotlin
// 获取控制器实例
val controller = RollerBlindController.getInstance()

// 初始化BLE
controller.initBLE(context)

// 连接设备
controller.connect(callback)
```

### 连接回调
```kotlin
override fun onConnected(name: String?, address: String?) {
    // 连接成功
}

override fun onDisConnected() {
    // 连接断开
}

override fun onMtuNegotiationSuccess(mtu: Int) {
    // MTU协商成功
}
```

## 技术特性

### 1. Application级别初始化
- **全局单例**: 确保整个应用只有一个控制器实例
- **自动初始化**: 应用启动时自动完成BLE初始化
- **避免重复**: 防止多次初始化导致的资源浪费
- **统一管理**: 通过ControllerManager统一获取实例

### 2. 稳定的连接管理
- 自动重连机制
- 连接状态监控
- 异常恢复处理

### 3. 优化的传输性能
- 动态MTU协商
- PHY参数优化
- 连接参数调优

### 4. 完整的错误处理
- 连接超时处理
- 扫描失败重试
- 配对状态管理

### 5. 设备管理
- 设备地址保存
- 快速重连支持
- 配对状态持久化

## 与LED控制器的兼容性

- 使用相同的VTBLEController底层实现
- 支持相同的连接优化特性
- 保持相同的API设计模式
- 共享相同的BLE配置和常量

## 注意事项

1. **权限要求**: 需要BLUETOOTH、BLUETOOTH_SCAN、BLUETOOTH_CONNECT权限
2. **Application初始化**: 控制器在Application中自动初始化，无需手动调用initBLE()
3. **线程安全**: 所有回调都在主线程执行
4. **资源管理**: 在Activity销毁时应该调用disconnect()
5. **实例获取**: 推荐使用ControllerManager.getRollerBlindController(context)获取实例

## 文件结构

```
RollerBlindControllerApp/
├── app/src/main/java/com/vincent/android/rollerblindcontroller/
│   ├── app/
│   │   └── RollerBlindApp.kt                 # Application类（已更新）
│   ├── logic/
│   │   ├── RollerBlindController.kt          # 主控制器类
│   │   └── RollerBlindControllerUsage.kt     # 使用示例
│   ├── utils/
│   │   ├── RollerBlindConstants.kt           # 常量定义
│   │   └── ControllerManager.kt              # 控制器管理器
│   └── ui/
│       └── MainActivity.kt                   # 主界面（已更新）
└── ROLLER_BLIND_CONTROLLER_UPDATE.md         # 本文档
```

## 总结

通过这次更新，RollerBlindController现在具备了与LED控制器相同的稳定蓝牙连接能力，包括：

- ✅ 稳定的BLE连接管理
- ✅ 自动重连机制
- ✅ MTU和PHY优化
- ✅ 设备配对管理
- ✅ 连接状态监控
- ✅ 完整的错误处理
- ✅ 与现有UI的完美集成
- ✅ Application级别初始化
- ✅ 全局单例管理
- ✅ 统一的实例获取方式

窗帘控制器现在可以像LED控制器一样稳定地工作，并且通过Application级别的初始化，确保了更好的资源管理和更简洁的使用方式，为用户提供可靠的智能窗帘控制体验。