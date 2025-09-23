#include "BLEHandler.h"
#include <Arduino.h>

// BLEHandler静态实例指针初始化
BLEHandler* BLEHandler::instance = nullptr;

// ControlCharacteristicCallbacks 实现
ControlCharacteristicCallbacks::ControlCharacteristicCallbacks(BLEHandler* handler) {
    bleHandler = handler;
}

void ControlCharacteristicCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
    Serial.println("=== BLE onWrite 回调被调用 ===");
    std::string value = pCharacteristic->getValue();
    
    Serial.print("BLE接收到命令: ");
    Serial.println(value.c_str());
    Serial.print("命令长度: ");
    Serial.println(value.length());
    
    if (value.length() > 0) {
        Serial.println("开始处理BLE命令");
        handleBLECommand(value);
        Serial.println("BLE命令处理完成");
    } else {
        Serial.println("命令长度为0，忽略");
    }
}

void ControlCharacteristicCallbacks::handleBLECommand(std::string value) {
    if (bleHandler == nullptr) {
        Serial.println("BLE处理器为空");
        return;
    }
    
    char command = value[0];
    
    switch (command) {
        case BLE_CMD_UP:
            Serial.println("BLE命令: 升起窗帘");
            if (bleHandler->onUpCommand) {
                bleHandler->onUpCommand();
            }
            break;
            
        case BLE_CMD_DOWN:
            Serial.println("BLE命令: 放下窗帘");
            if (bleHandler->onDownCommand) {
                bleHandler->onDownCommand();
            }
            break;
            
        case BLE_CMD_LEFT:
            Serial.println("BLE命令: 微调升起");
            if (bleHandler->onLeftCommand) {
                bleHandler->onLeftCommand();
            }
            break;
            
        case BLE_CMD_RIGHT:
            Serial.println("BLE命令: 微调放下");
            if (bleHandler->onRightCommand) {
                bleHandler->onRightCommand();
            }
            break;
            
        case BLE_CMD_STOP:
            Serial.println("BLE命令: 停止电机");
            if (bleHandler->onStopCommand) {
                bleHandler->onStopCommand();
            }
            break;
            
        case BLE_CMD_SET:
            Serial.println("BLE命令: 设置模式");
            if (bleHandler->onSetCommand) {
                bleHandler->onSetCommand();
            }
            break;
            
        case BLE_CMD_DIRECTION:
            Serial.println("BLE命令: 切换方向");
            if (bleHandler->onDirectionCommand) {
                bleHandler->onDirectionCommand();
            }
            break;
            
        default:
            Serial.print("未知BLE命令: ");
            Serial.println(command);
            break;
    }
}

// MyBLEServerCallbacks 实现
void MyBLEServerCallbacks::onConnect(BLEServer *pServer) {
    Serial.println("BLE设备已连接");
}

void MyBLEServerCallbacks::onDisconnect(BLEServer *pServer) {
    Serial.println("BLE设备已断开连接");
    pServer->getAdvertising()->start();
}

// BLEHandler 实现
BLEHandler::BLEHandler() {
    instance = this;
    pServer = nullptr;
    pService = nullptr;
    pControlCharacteristic = nullptr;
    controlCallbacks = nullptr;
    
    // 初始化回调函数指针
    onUpCommand = nullptr;
    onDownCommand = nullptr;
    onLeftCommand = nullptr;
    onRightCommand = nullptr;
    onStopCommand = nullptr;
    onSetCommand = nullptr;
    onDirectionCommand = nullptr;
}

void BLEHandler::init() {
    Serial.println("初始化BLE");
    
    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyBLEServerCallbacks());
    
    pService = pServer->createService(BLE_SERVICE_UUID);
    
    createCharacteristics();
    setupCallbacks();
    
    pService->start();
    
    Serial.println("BLE初始化完成");
}

void BLEHandler::createCharacteristics() {
    Serial.println("创建BLE特征值");
    
    // 控制特征值
    pControlCharacteristic = pService->createCharacteristic(
        BLE_CHARACTERISTIC_CONTROL_UUID,
        BLECharacteristic::PROPERTY_READ | 
        BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    Serial.println("BLE特征值创建完成");
}

void BLEHandler::setupCallbacks() {
    controlCallbacks = new ControlCharacteristicCallbacks(this);
    pControlCharacteristic->setCallbacks(controlCallbacks);
}

void BLEHandler::startAdvertising() {
    Serial.println("开始BLE广播");
    pServer->getAdvertising()->start();
}

void BLEHandler::stopAdvertising() {
    Serial.println("停止BLE广播");
    pServer->getAdvertising()->stop();
}

void BLEHandler::disconnectBLE() {
    Serial.println("断开BLE连接");
    if (pServer != nullptr) {
        pServer->getAdvertising()->stop();
        
        // 断开所有连接的客户端
        int connectedCount = pServer->getConnectedCount();
        for (int i = 0; i < connectedCount; i++) {
            uint16_t connId = pServer->getConnId();
            if (connId != 0) {
                pServer->disconnect(connId);
            }
        }
        
        delay(1000);
    }
}

void BLEHandler::setCommandCallbacks(
    void (*upFunc)(),
    void (*downFunc)(),
    void (*leftFunc)(),
    void (*rightFunc)(),
    void (*stopFunc)(),
    void (*setFunc)(),
    void (*directionFunc)()
) {
    onUpCommand = upFunc;
    onDownCommand = downFunc;
    onLeftCommand = leftFunc;
    onRightCommand = rightFunc;
    onStopCommand = stopFunc;
    onSetCommand = setFunc;
    onDirectionCommand = directionFunc;
    
    Serial.println("BLE命令回调函数已设置");
}