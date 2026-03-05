/*
 * 自适应环境光填充系统
 * 显示设备 - 接收端程序（带步进电机指针和WS2812补光）
 * 平台：PlatformIO / Arduino 框架
 * 功能：通过 BLE 接收传感器数据，通过步进电机指针显示亮度百分比
 *       按键控制 WS2812 补光模式，自动调节亮度维持目标光照
 * 步进电机：28BYJ-48 (使用 ULN2003 驱动板)
 * 引脚：D0-D3 对应 IN1-IN4
 * WS2812：D9
 * 按键：D8 (软件上拉，按下为低电平，使用中断)
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>
#include <Adafruit_NeoPixel.h>

// 步进电机引脚定义（使用 D0-D3）
#define MOTOR_IN1 D0  // GPIO0
#define MOTOR_IN2 D1  // GPIO1
#define MOTOR_IN3 D2  // GPIO2
#define MOTOR_IN4 D3  // GPIO3

// WS2812 配置
#define WS2812_PIN D9     // WS2812 数据引脚
#define NUM_PIXELS 1      // WS2812 灯珠数量

// 按键配置
#define BUTTON_PIN D8     // 按键引脚（软件上拉，按下为低电平）

// 28BYJ-48 步进电机参数
#define STEPS_PER_REV 4096  // 减速比 1/64，步进电机每转 4096 步
#define MAX_ANGLE 180        // 指针最大转动角度（度）
#define MIN_ANGLE 0          // 指针最小角度
#define ANGLE_OFFSET 0       // 初始位置偏移（0度，指向0%位置）

// 补光模式配置
#define TARGET_LUX_DEFAULT 200     // 默认目标光照值 (lux)
#define LUX_TOLERANCE 20           // 允许误差范围 (±20 lux)
#define LUX_MIN_SAFE 10            // 最小安全目标值
#define LUX_MAX_SAFE 1000          // 最大安全目标值
#define LUX_MIN_DETECT 1           // 最小可检测光照
#define LUX_MAX_DETECT 40000       // 最大可检测光照

// PID 控制参数
#define PID_KP 0.8                  // 比例系数 - 响应速度
#define PID_KI 0.05                 // 积分系数 - 消除静差
#define PID_KD 0.1                  // 微分系数 - 抑制震荡

// WS2812 亮度范围
#define BRIGHTNESS_MIN 0            // 最小亮度 (0-255)
#define BRIGHTNESS_MAX 255          // 最大亮度
#define BRIGHTNESS_DEFAULT 100      // 默认初始亮度

// 电机控制相关变量
int motorPins[4] = {MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4};
int currentStep = 0;         // 当前步进位置
int targetStep = 0;          // 目标步进位置
int motorSpeed = 800;        // 电机速度（微秒延迟，值越小越快）
bool motorRunning = false;

// WS2812 对象
Adafruit_NeoPixel pixels(NUM_PIXELS, WS2812_PIN, NEO_GRB + NEO_KHZ800);

// 补光控制变量
bool lightModeEnabled = false;      // 补光模式开关
float targetLux = TARGET_LUX_DEFAULT;  // 目标光照值
int ws2812Brightness = 0;           // WS2812 当前亮度 (0-255)

// PID 控制变量
float pidIntegral = 0;              // 积分项
float lastError = 0;                // 上次误差
unsigned long lastControlTime = 0;  // 上次控制时间
float currentLux = 0;                // 当前光照值

// 按键相关变量
volatile bool buttonPressed = false;  // 中断标志
unsigned long lastButtonTime = 0;
const unsigned long buttonDebounceDelay = 200; // 200ms消抖

// 步进序列
const int stepSequence[8][4] = {
  {1, 0, 0, 0},  // Step 1: A
  {1, 1, 0, 0},  // Step 2: AB
  {0, 1, 0, 0},  // Step 3: B
  {0, 1, 1, 0},  // Step 4: BC
  {0, 0, 1, 0},  // Step 5: C
  {0, 0, 1, 1},  // Step 6: CD
  {0, 0, 0, 1},  // Step 7: D
  {1, 0, 0, 1}   // Step 8: DA
};

// 配置参数
#define SCAN_TIME 5           // 每次扫描持续时间（秒）
#define LED_PIN D4            // 状态指示灯引脚

// 传感器设备的 BLE 名称和服务 UUID
#define SENSOR_BLE_NAME "环境光传感器"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE 客户端相关变量
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;
bool deviceFound = false;
bool connected = false;

// 数据解析结果
float lastPercentage = -1;  // 上次显示的百分比
unsigned long lastDataTime = 0;
int dataCount = 0;

// 连接状态
bool isScanning = false;
unsigned long lastScanTime = 0;

// 串口命令处理
bool serialModeEnabled = false;     // 串口控制模式
float serialTargetLux = TARGET_LUX_DEFAULT;

/**
 * 按键中断服务函数
 */
void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

/**
 * 初始化 WS2812
 */
void initWS2812() {
  pixels.begin();
  pixels.setBrightness(255); // WS2812 亮度由我们自己控制
  pixels.show();
  Serial.println("WS2812 初始化完成");
}

/**
 * 设置 WS2812 亮度和颜色
 * @param brightness 亮度 0-255
 * @param color 颜色（默认暖白光）
 */
void setWS2812(int brightness, uint32_t color = pixels.Color(255, 200, 150)) {
  if (brightness <= BRIGHTNESS_MIN) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    ws2812Brightness = 0;
  } else {
    // 限制亮度范围
    if (brightness > BRIGHTNESS_MAX) brightness = BRIGHTNESS_MAX;
    
    // 根据亮度调整颜色强度
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    
    // 按比例调整
    r = r * brightness / BRIGHTNESS_MAX;
    g = g * brightness / BRIGHTNESS_MAX;
    b = b * brightness / BRIGHTNESS_MAX;
    
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    ws2812Brightness = brightness;
  }
  pixels.show();
  
  Serial.printf("WS2812 亮度设置为: %d\n", ws2812Brightness);
}

/**
 * PID 控制算法计算所需亮度
 * @return 建议的亮度调整值 (-255 到 255)
 */
int pidControl() {
  if (!lightModeEnabled) return 0;
  
  float error = targetLux - currentLux;
  
  // 如果误差在允许范围内，不调整
  if (abs(error) <= LUX_TOLERANCE) {
    return 0;
  }
  
  unsigned long now = millis();
  float dt = (now - lastControlTime) / 1000.0; // 转换为秒
  if (dt < 0.1) dt = 0.1; // 最小时间间隔
  
  // 比例项
  float pTerm = PID_KP * error;
  
  // 积分项（带限幅）
  pidIntegral += error * dt;
  if (pidIntegral > 100) pidIntegral = 100;
  if (pidIntegral < -100) pidIntegral = -100;
  float iTerm = PID_KI * pidIntegral;
  
  // 微分项
  float dTerm = PID_KD * (error - lastError) / dt;
  
  // 计算输出
  float output = pTerm + iTerm + dTerm;
  
  // 限幅输出
  if (output > 255) output = 255;
  if (output < -255) output = -255;
  
  lastError = error;
  lastControlTime = now;
  
  Serial.printf("PID控制: 当前=%.1f, 目标=%.1f, 误差=%.1f, 输出=%.1f\n", 
                currentLux, targetLux, error, output);
  
  return (int)output;
}

/**
 * 更新补光亮度
 */
void updateLightControl() {
  if (!lightModeEnabled) {
    // 如果补光模式关闭，关闭WS2812
    if (ws2812Brightness > 0) {
      setWS2812(0);
    }
    return;
  }
  
  // 计算所需亮度调整
  int adjustment = pidControl();
  
  if (adjustment != 0) {
    int newBrightness = ws2812Brightness + adjustment;
    
    // 限制亮度范围
    if (newBrightness < BRIGHTNESS_MIN) newBrightness = BRIGHTNESS_MIN;
    if (newBrightness > BRIGHTNESS_MAX) newBrightness = BRIGHTNESS_MAX;
    
    // 应用新的亮度
    setWS2812(newBrightness);
  }
}

/**
 * 切换补光模式
 */
void toggleLightMode() {
  lightModeEnabled = !lightModeEnabled;
  
  if (lightModeEnabled) {
    Serial.println("\n=== 补光模式已开启 ===");
    Serial.printf("目标光照值: %.1f lux\n", targetLux);
    Serial.printf("当前光照值: %.1f lux\n", currentLux);
    
    // 重置PID参数
    pidIntegral = 0;
    lastError = 0;
    lastControlTime = millis();
    
    // 根据当前光照设置初始亮度
    if (currentLux < targetLux) {
      // 需要补光，设置中等亮度
      setWS2812(BRIGHTNESS_DEFAULT);
    } else {
      // 当前光照已达标，关闭补光
      setWS2812(0);
    }
  } else {
    Serial.println("\n=== 补光模式已关闭 ===");
    setWS2812(0);
  }
}

/**
 * 设置目标光照值
 */
void setTargetLux(float lux) {
  if (lux < LUX_MIN_SAFE) lux = LUX_MIN_SAFE;
  if (lux > LUX_MAX_SAFE) lux = LUX_MAX_SAFE;
  
  targetLux = lux;
  Serial.printf("目标光照值已设置为: %.1f lux\n", targetLux);
  
  // 重置PID参数
  if (lightModeEnabled) {
    pidIntegral = 0;
    lastError = 0;
    lastControlTime = millis();
  }
}

/**
 * 初始化按键（使用中断）
 */
void initButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  Serial.println("按键中断初始化完成");
}

/**
 * 处理按键事件
 */
void handleButtonPress() {
  if (buttonPressed) {
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
    
    unsigned long currentTime = millis();
    if (currentTime - lastButtonTime > buttonDebounceDelay) {
      lastButtonTime = currentTime;
      
      if (digitalRead(BUTTON_PIN) == LOW) {
        // 切换补光模式
        toggleLightMode();
        
        // LED反馈
        digitalWrite(LED_PIN, HIGH);
        delay(50);
        digitalWrite(LED_PIN, LOW);
      }
    }
    
    buttonPressed = false;
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  }
}

/**
 * 步进电机控制函数
 */
void stepMotor(int direction) {
  currentStep += direction;
  if (currentStep < 0) currentStep += 8;
  if (currentStep >= 8) currentStep -= 8;
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], stepSequence[currentStep][i]);
  }
  delayMicroseconds(motorSpeed);
}

int percentageToSteps(float percentage) {
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  return (STEPS_PER_REV * MAX_ANGLE / 360) * percentage / 100;
}

void motorHome() {
  Serial.println("电机归零中...");
  targetStep = 0;
  
  static int currentStepPos = 0;
  int stepsToMove = targetStep - currentStepPos;
  
  if (stepsToMove != 0) {
    int direction = (stepsToMove > 0) ? 1 : -1;
    int steps = abs(stepsToMove);
    
    for (int i = 0; i < steps; i++) {
      stepMotor(direction);
      delay(1);
    }
    currentStepPos = targetStep;
  }
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }
  Serial.println("电机归零完成");
}

void moveToPercentage(float percentage) {
  targetStep = percentageToSteps(percentage);
  float targetAngle = MAX_ANGLE * percentage / 100;
  
  Serial.printf("移动指针: %.1f%% -> %.1f度, 步数: %d\n", 
                percentage, targetAngle, targetStep);
  
  static int currentStepPos = 0;
  int stepsToMove = targetStep - currentStepPos;
  
  if (stepsToMove != 0) {
    motorRunning = true;
    int direction = (stepsToMove > 0) ? 1 : -1;
    int steps = abs(stepsToMove);
    
    for (int i = 0; i < steps; i++) {
      stepMotor(direction);
      delay(1);
    }
    
    currentStepPos = targetStep;
    motorRunning = false;
    
    for (int i = 0; i < 4; i++) {
      digitalWrite(motorPins[i], LOW);
    }
  }
}

void initMotor() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  motorHome();
  delay(1000);
  Serial.println("电机初始化完成");
}

/**
 * 设备发现回调
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("发现设备: %s [%s]\n", 
                  advertisedDevice.getName().c_str(),
                  advertisedDevice.getAddress().toString().c_str());
    
    if (advertisedDevice.getName() == SENSOR_BLE_NAME) {
      Serial.println("\n>>> 找到目标传感器设备！ <<<");
      advertisedDevice.getScan()->stop();
      targetDevice = new BLEAdvertisedDevice(advertisedDevice);
      deviceFound = true;
    }
  }
};

/**
 * 解析传感器数据
 * 格式: "LUX:123.45,PERC:67.8"
 */
bool parseSensorData(const std::string& data) {
  char buffer[100];
  strcpy(buffer, data.c_str());
  
  float lux = 0, perc = 0;
  
  char* luxPtr = strstr(buffer, "LUX:");
  if (luxPtr != nullptr) {
    luxPtr += 4;
    char* percPtr = strstr(luxPtr, ",PERC:");
    if (percPtr != nullptr) {
      *percPtr = '\0';
      lux = atof(luxPtr);
      
      percPtr += 6;
      perc = atof(percPtr);
      
      currentLux = lux;
      lastDataTime = millis();
      dataCount++;
      
      // 移动电机指针
      if (abs(perc - lastPercentage) > 2.0) {
        moveToPercentage(perc);
        lastPercentage = perc;
      }
      
      // 如果补光模式开启，更新补光控制
      if (lightModeEnabled) {
        updateLightControl();
      }
      
      return true;
    }
  }
  return false;
}

/**
 * 连接到传感器
 */
bool connectToSensor() {
  if (targetDevice == nullptr) return false;
  
  Serial.print("正在连接至 ");
  Serial.println(targetDevice->getAddress().toString().c_str());
  
  pClient = BLEDevice::createClient();
  if (!pClient->connect(targetDevice)) {
    Serial.println("连接失败！");
    return false;
  }
  Serial.println("连接成功");
  
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("获取服务失败！");
    pClient->disconnect();
    return false;
  }
  
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("获取特征失败！");
    pClient->disconnect();
    return false;
  }
  
  // 启用通知
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify([](BLERemoteCharacteristic* pChar, 
                                                 uint8_t* pData, size_t length, 
                                                 bool isNotify) {
      std::string value = std::string((char*)pData, length);
      parseSensorData(value);
    });
    Serial.println("启用通知成功");
  }
  
  connected = true;
  
  // 读取初始值
  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    parseSensorData(value);
  }
  
  return true;
}

void disconnectFromSensor() {
  if (pClient != nullptr) {
    if (pClient->isConnected()) {
      pClient->disconnect();
    }
    delete pClient;
    pClient = nullptr;
  }
  connected = false;
  pRemoteCharacteristic = nullptr;
  Serial.println("已断开连接");
}

void startScanning() {
  if (isScanning) return;
  
  Serial.println("\n=== 开始扫描环境光传感器 ===");
  
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  deviceFound = false;
  isScanning = true;
  pBLEScan->start(SCAN_TIME, false);
  isScanning = false;
  
  if (deviceFound && targetDevice != nullptr) {
    Serial.println("设备找到，尝试连接...");
    if (connectToSensor()) {
      Serial.println("连接成功！\n");
    } else {
      Serial.println("连接失败，5秒后重新扫描");
      delay(5000);
    }
  } else {
    Serial.println("未找到目标设备，5秒后重新扫描");
    delay(5000);
  }
}

/**
 * 打印状态信息
 */
void printStatus() {
  Serial.println("\n=== 系统状态 ===");
  Serial.printf("连接状态: %s\n", connected ? "已连接" : "未连接");
  Serial.printf("补光模式: %s\n", lightModeEnabled ? "开启" : "关闭");
  Serial.printf("当前光照: %.1f lux\n", currentLux);
  Serial.printf("目标光照: %.1f lux\n", targetLux);
  Serial.printf("WS2812亮度: %d\n", ws2812Brightness);
  Serial.printf("接收数据次数: %d\n", dataCount);
  if (dataCount > 0) {
    Serial.printf("最后接收: %lu ms前\n", millis() - lastDataTime);
  }
  Serial.println("================\n");
}

/**
 * 处理串口命令
 */
void handleSerialCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "ON") {
      if (!lightModeEnabled) toggleLightMode();
    }
    else if (cmd == "OFF") {
      if (lightModeEnabled) toggleLightMode();
    }
    else if (cmd.startsWith("SET ")) {
      float lux = cmd.substring(4).toFloat();
      setTargetLux(lux);
    }
    else if (cmd == "STATUS") {
      printStatus();
    }
    else if (cmd == "HELP") {
      Serial.println("\n=== 命令帮助 ===");
      Serial.println("ON      - 开启补光模式");
      Serial.println("OFF     - 关闭补光模式");
      Serial.println("SET xxx - 设置目标光照值 (如 SET 300)");
      Serial.println("STATUS  - 显示当前状态");
      Serial.println("HELP    - 显示此帮助");
      Serial.println("================\n");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n==============================================");
  Serial.println("=== 自适应环境光系统 - 显示设备 ===");
  Serial.println("=== 支持补光自动调节 ===");
  Serial.println("==============================================");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  initButton();
  initWS2812();
  initMotor();
  
  BLEDevice::init("环境光显示器");
  Serial.println("BLE 初始化完成\n");
  
  Serial.println("按键控制: 按下切换补光模式");
  Serial.println("串口控制: 输入 HELP 查看命令\n");
  
  lastScanTime = millis();
  startScanning();
}

void loop() {
  handleButtonPress();
  handleSerialCommand();
  
  if (connected && pClient != nullptr) {
    if (!pClient->isConnected()) {
      Serial.println("\n*** 连接断开 ***");
      disconnectFromSensor();
      lastScanTime = millis();
    } else {
      // LED闪烁指示活动
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 1000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
      }
    }
  }
  
  if (!connected && (millis() - lastScanTime > 5000)) {
    if (!isScanning) {
      lastScanTime = millis();
      startScanning();
    }
  }
  
  // 检查是否长时间没有收到数据
  if (connected && dataCount > 0 && (millis() - lastDataTime > 10000)) {
    Serial.println("\n*** 警告：超过10秒未收到数据 ***");
    printStatus();
  }
  
  delay(10);
}