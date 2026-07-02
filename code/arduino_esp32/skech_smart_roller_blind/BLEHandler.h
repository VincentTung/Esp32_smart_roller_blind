#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include "config.h"

#if ENABLE_BLE
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE服务UUID
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"

// BLE特征值UUID
#define BLE_CHARACTERISTIC_CONTROL_UUID "12345678-1234-1234-1234-123456789def"

// BLE设备名称
#define BLE_DEVICE_NAME "SmartRollerBlind"

// BLE命令定义
#define BLE_CMD_UP 'U'        // 升起窗帘
#define BLE_CMD_DOWN 'D'      // 放下窗帘
#define BLE_CMD_LEFT 'L'      // 微调升起
#define BLE_CMD_RIGHT 'R'     // 微调放下
#define BLE_CMD_STOP 'S'      // 停止电机
#define BLE_CMD_SET 'T'       // 设置模式
#define BLE_CMD_DIRECTION 'G' // 切换方向

class BLEHandler;

// BLE控制特征值回调
class ControlCharacteristicCallbacks : public BLECharacteristicCallbacks {
private:
    BLEHandler* bleHandler;
    
public:
    ControlCharacteristicCallbacks(BLEHandler* handler);
    void onWrite(BLECharacteristic *pCharacteristic);
    
private:
    void handleBLECommand(const String& value);
};

// BLE服务器回调
class MyBLEServerCallbacks : public BLEServerCallbacks {
public:
    void onConnect(BLEServer* pServer);
    void onDisconnect(BLEServer* pServer);
};

// BLE处理类
class BLEHandler {
private:
    BLEServer* pServer;
    BLEService* pService;
    BLECharacteristic* pControlCharacteristic;
    ControlCharacteristicCallbacks* controlCallbacks;
    
    // 回调函数指针
    void (*onUpCommand)();
    void (*onDownCommand)();
    void (*onLeftCommand)();
    void (*onRightCommand)();
    void (*onStopCommand)();
    void (*onSetCommand)();
    void (*onDirectionCommand)();
    

    friend class ControlCharacteristicCallbacks;
    
public:
    static BLEHandler* instance;
    
    BLEHandler();
    void init();
    void startAdvertising();
    void stopAdvertising();
    void disconnectBLE();
    
    // 设置回调函数
    void setCommandCallbacks(
        void (*upFunc)(),
        void (*downFunc)(),
        void (*leftFunc)(),
        void (*rightFunc)(),
        void (*stopFunc)(),
        void (*setFunc)(),
        void (*directionFunc)()
    );
    
private:
    void createCharacteristics();
    void setupCallbacks();
};

#endif // ENABLE_BLE

#endif // BLE_HANDLER_H