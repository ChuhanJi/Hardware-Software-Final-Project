/*
 * 自适应环境光填充系统
 * 传感器设备 - XIAO ESP32-C3 搭配 BH1750
 * 平台：PlatformIO / Arduino 框架
 */

#include <Arduino.h>
#include <Wire.h>
#include <BH1750.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_sleep.h>

// 引脚定义 - 注意：GPIO8 可能有特殊限制，建议使用 GPIO6
#define I2C_SDA 8  // 当前使用 GPIO8，但建议改为 6 以获得更好稳定性
#define I2C_SCL 9  // XIAO ESP32-C3 SCL 引脚
#define LED_PIN 2  // 状态 LED（XIAO 没有板载 LED，预留外部连接）

// BLE 服务和特征 UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// 传感器配置
#define LUX_THRESHOLD_DARK 50.0    // 黑暗阈值（勒克斯）
#define LUX_THRESHOLD_BRIGHT 300.0 // 明亮阈值（勒克斯）
#define SAMPLE_INTERVAL_MS 2000    // 读取间隔（毫秒）
#define MOVING_AVG_WINDOW 5        // 移动平均窗口大小

// 深度睡眠配置
#define SLEEP_ENABLED false        // 启用/禁用读取间隔间的深度睡眠
#define SLEEP_DURATION_SECONDS 5   // 启用时的睡眠持续时间

// BH1750 传感器实例
BH1750 lightMeter;

// BLE 变量
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// 传感器数据处理
float luxValues[MOVING_AVG_WINDOW] = {0};
int luxIndex = 0;
float filteredLux = 0;
float brightnessPercentage = 0;

// 定时
unsigned long lastReadingTime = 0;

// BLE 服务器回调函数
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("设备已连接");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("设备已断开连接");
  }
};

/**
 * 扫描 I2C 总线上的设备
 */
void scanI2C() {
  Serial.println("扫描 I2C 设备...");
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("发现设备地址：0x%02X", address);
      if (address == 0x23) Serial.println(" (BH1750 默认地址)");
      else if (address == 0x5C) Serial.println(" (BH1750 备用地址)");
      else Serial.println();
      nDevices++;
      
      // 如果是 BH1750，测试读取
      if (address == 0x23 || address == 0x5C) {
        Wire.beginTransmission(address);
        Wire.write(0x10); // 连续高分辨率模式命令
        Wire.endTransmission();
        delay(180); // 等待测量
        
        Wire.requestFrom(address, 2);
        if (Wire.available() == 2) {
          uint16_t value = Wire.read() << 8 | Wire.read();
          float lux = value / 1.2;
          Serial.printf("  测试读取：%.2f lux\n", lux);
        }
      }
    }
  }
  
  if (nDevices == 0) {
    Serial.println("未发现任何 I2C 设备");
    Serial.println("请检查：");
    Serial.println("1. 传感器电源连接（VCC 和 GND）");
    Serial.println("2. I2C 引脚连接（SDA 和 SCL）");
    Serial.println("3. 是否需要上拉电阻（建议 4.7kΩ）");
  } else {
    Serial.printf("扫描完成，发现 %d 个设备\n", nDevices);
  }
}

/**
 * 初始化 BH1750 传感器，包含错误处理
 */
bool initBH1750() {
  // 如果使用 GPIO8，尝试配置上拉电阻和低速模式
  if (I2C_SDA == 8) {
    Serial.println("警告：使用 GPIO8 作为 I2C SDA 可能不稳定");
    Serial.println("建议改用 GPIO6 以获得更好稳定性");
    
    // 启用内部上拉电阻
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    
    // 使用更慢的 I2C 速度（10kHz）提高稳定性
    Wire.begin(I2C_SDA, I2C_SCL, 10000);
    Wire.setClock(10000);
  } else {
    // 正常初始化
    Wire.begin(I2C_SDA, I2C_SCL);
  }
  
  // 先扫描 I2C 总线
  scanI2C();
  
  // 尝试初始化 BH1750
  for (int retry = 0; retry < 5; retry++) {
    Serial.printf("尝试初始化 BH1750（尝试 %d/5）...\n", retry + 1);
    
    // 尝试两个可能的地址
    bool success = false;
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire)) {
      success = true;
      Serial.println("使用地址 0x23 初始化成功");
    } else if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire)) {
      success = true;
      Serial.println("使用地址 0x5C 初始化成功");
    }
    
    if (success) {
      Serial.println("BH1750 初始化成功");
      return true;
    }
    
    Serial.printf("BH1750 初始化尝试 %d 失败\n", retry + 1);
    delay(500); // 增加延迟
  }
  
  Serial.println("尝试 5 次后 BH1750 初始化失败");
  return false;
}

/**
 * 初始化 BLE 服务器和特征
 */
void initBLE() {
  BLEDevice::init("环境光传感器");
  
  // 创建 BLE 服务器
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // 创建 BLE 服务
  BLEService* pService = pServer->createService(SERVICE_UUID);
  
  // 创建 BLE 特征
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // 添加 BLE 描述符以支持通知
  pCharacteristic->addDescriptor(new BLE2902());
  
  // 启动服务
  pService->start();
  
  // 开始广播
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // 有助于解决 iPhone 连接问题
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE 初始化完成，等待连接...");
}

/**
 * 对传感器读数应用移动平均滤波
 */
float movingAverageFilter(float newValue) {
  luxValues[luxIndex] = newValue;
  luxIndex = (luxIndex + 1) % MOVING_AVG_WINDOW;
  
  float sum = 0;
  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    sum += luxValues[i];
  }
  
  return sum / MOVING_AVG_WINDOW;
}

/**
 * 应用中值滤波以去除尖峰
 */
float medianFilter(float newValue, float* history, int size) {
  // 移位历史记录
  for (int i = size - 1; i > 0; i--) {
    history[i] = history[i - 1];
  }
  history[0] = newValue;
  
  // 复制用于排序
  float temp[size];
  memcpy(temp, history, size * sizeof(float));
  
  // 简单的冒泡排序（size 较小）
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (temp[j] > temp[j + 1]) {
        float t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }
  
  return temp[size / 2];
}

/**
 * 将勒克斯值转换为亮度百分比
 * 包含滞回比较以防止振荡
 */
float luxToPercentage(float lux) {
  static float lastPercentage = 0;
  static const float hysteresis = 5.0; // 5% 滞回
  
  float rawPercentage;
  
  if (lux <= LUX_THRESHOLD_DARK) {
    rawPercentage = 0;
  } else if (lux >= LUX_THRESHOLD_BRIGHT) {
    rawPercentage = 100;
  } else {
    // 阈值之间的线性映射
    rawPercentage = (lux - LUX_THRESHOLD_DARK) * 100.0 / 
                    (LUX_THRESHOLD_BRIGHT - LUX_THRESHOLD_DARK);
  }
  
  // 应用滞回以防止快速波动
  if (abs(rawPercentage - lastPercentage) > hysteresis) {
    lastPercentage = rawPercentage;
  }
  
  return lastPercentage;
}

/**
 * 读取并处理光传感器数据
 */
float readLightSensor() {
  if (!lightMeter.measurementReady()) {
    Serial.println("传感器数据未就绪");
    return filteredLux; // 返回最后一个有效值
  }
  
  float lux = lightMeter.readLightLevel();
  
  // 验证读数
  if (lux < 0 || lux > 100000) { // 无效范围
    Serial.println("无效的传感器读数");
    return filteredLux;
  }
  
  Serial.printf("原始勒克斯值：%.2f\n", lux);
  
  // 首先应用中值滤波（去除尖峰）
  static float history[5] = {0};
  float median = medianFilter(lux, history, 5);
  
  // 然后应用移动平均
  filteredLux = movingAverageFilter(median);
  
  // 转换为百分比
  brightnessPercentage = luxToPercentage(filteredLux);
  
  Serial.printf("滤波后勒克斯值：%.2f，百分比：%.1f%%\n", 
                filteredLux, brightnessPercentage);
  
  return filteredLux;
}

/**
 * 通过 BLE 发送数据到显示设备
 */
void sendDataOverBLE() {
  if (deviceConnected) {
    // 创建数据包
    char dataPacket[50];
    snprintf(dataPacket, sizeof(dataPacket), "LUX:%.2f,PERC:%.1f", 
             filteredLux, brightnessPercentage);
    
    // 设置值并通知
    pCharacteristic->setValue(dataPacket);
    pCharacteristic->notify();
    
    Serial.println("数据已发送：" + String(dataPacket));
    
    // 如果连接了外部 LED，可以闪烁
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  } else {
    Serial.println("无 BLE 连接，数据未发送");
  }
}

/**
 * 进入深度睡眠模式以节省功耗
 */
void enterDeepSleep() {
  if (!SLEEP_ENABLED) return;
  
  Serial.println("进入深度睡眠 " + String(SLEEP_DURATION_SECONDS) + " 秒");
  Serial.flush();
  
  // 配置唤醒源
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);
  
  // 进入深度睡眠
  esp_deep_sleep_start();
}

void setup() {
  // 初始化串口
  Serial.begin(115200);
  delay(2000); // 增加延迟确保 USB 串口就绪
  
  Serial.println("\n=========================================");
  Serial.println("=== 自适应环境光系统 - 传感器设备 ===");
  Serial.println("=========================================");
  
  // 打印编译信息
  Serial.printf("编译时间：%s %s\n", __DATE__, __TIME__);
  Serial.printf("I2C 引脚配置：SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);
  
  if (I2C_SDA == 8) {
    Serial.println("注意：当前使用 GPIO8 作为 SDA");
    Serial.println("如果通信不稳定，建议改用 GPIO6");
  }
  
  // 初始化 LED（虽然 XIAO 没有板载 LED，但保留代码以便外接）
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // 初始化 I2C 和 BH1750
  Serial.println("\n--- 初始化传感器 ---");
  if (!initBH1750()) {
    Serial.println("\n严重错误：BH1750 初始化失败！");
    Serial.println("请检查硬件连接：");
    Serial.println("1. 传感器 VCC → 3.3V");
    Serial.println("2. 传感器 GND → GND");
    Serial.println("3. 传感器 SDA → GPIO" + String(I2C_SDA));
    Serial.println("4. 传感器 SCL → GPIO" + String(I2C_SCL));
    Serial.println("5. 是否需要上拉电阻（建议 4.7kΩ）");
    
    // 在无限循环中持续输出调试信息
    int counter = 0;
    while (1) {
      counter++;
      Serial.printf("\n[%d] 等待硬件重置或修正连接...\n", counter);
      
      // 每10次重新扫描一次 I2C
      if (counter % 10 == 0) {
        Serial.println("\n重新扫描 I2C 总线：");
        Wire.begin(I2C_SDA, I2C_SCL, 10000);
        scanI2C();
      }
      
      // 快速闪烁 LED（如果有外部连接）
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      
      delay(2000);
    }
  }
  
  // 初始化 BLE
  Serial.println("\n--- 初始化 BLE ---");
  initBLE();
  
  // 初始传感器读数
  Serial.println("\n--- 首次读取传感器 ---");
  readLightSensor();
  lastReadingTime = millis();
  
  Serial.println("\n=========================================");
  Serial.println("传感器设备就绪！");
  Serial.println("等待 BLE 连接...");
  Serial.println("=========================================\n");
}

void loop() {
  unsigned long currentTime = millis();
  
  // 处理 BLE 断开连接
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("设备断开连接，重新开始广播...");
    delay(500); // 给蓝牙协议栈时间准备就绪
    pServer->startAdvertising();
    Serial.println("广播已重启");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("BLE 设备已连接！");
    oldDeviceConnected = deviceConnected;
  }
  
  // 按指定间隔读取传感器
  if (currentTime - lastReadingTime >= SAMPLE_INTERVAL_MS) {
    Serial.println("\n--- 执行周期读取 ---");
    readLightSensor();
    sendDataOverBLE();
    lastReadingTime = currentTime;
    
    // 可选：在两次读取之间进入深度睡眠
    if (SLEEP_ENABLED) {
      enterDeepSleep();
      // 设备将唤醒并从 setup() 重新开始
    }
  }
  
  // 短暂延迟以防止看门狗问题
  delay(10);
}