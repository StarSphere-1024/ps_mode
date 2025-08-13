#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <FastLED.h>
#include <PS2X_lib.h>
#include <SoftwareSerial.h>
#include "LIS3DHTR.h"
#include "DHT.h"
#include "Ultrasonic.h"
#include <WonderK210.h>

#define USER_BUTTON_A_PIN 21
#define RGB_PIN 33
#define I2C_SDA 39
#define I2C_SCL 40

#define K210_RX_PIN 37
#define K210_TX_PIN 36
#define ASR_RX_PIN 35
#define ASR_TX_PIN 34

#define GROVE6_PIN_A 1
#define GROVE6_PIN_B 2
#define GROVE3_PIN_A 3
#define GROVE3_PIN_B 4
#define GROVE5_PIN_A 5
#define GROVE5_PIN_B 6
#define GROVE2_PIN_A 7
#define GROVE2_PIN_B 8
#define GROVE4_PIN_A 26
#define GROVE4_PIN_B 38

#define LIGHT_PIN GROVE2_PIN_A
#define DHT_PIN GROVE4_PIN_A
#define ULTRASONIC_PIN GROVE5_PIN_A

#define PS2_CMD_PIN 9
#define PS2_DATA_PIN 10
#define PS2_CLK_PIN 41
#define PS2_CS_PIN 42

#define LF_MOTOR_FWD_PWM 11
#define LF_MOTOR_REV_PWM 12
#define RF_MOTOR_FWD_PWM 14
#define RF_MOTOR_REV_PWM 13
#define LR_MOTOR_FWD_PWM 15
#define LR_MOTOR_REV_PWM 16
#define RR_MOTOR_FWD_PWM 18
#define RR_MOTOR_REV_PWM 17

#define NUM_RGB_LEDS 9
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define INITIAL_RGB_BRIGHTNESS 100

#define SERVO1_PIN 48
#define SERVO2_PIN 47
#define SERVO_TIMER_FREQUENCY 1000000
#define SERVO_TIMER_TICK_US 50
#define SERVO_PWM_PERIOD_MS 20
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO1_DEFAULT_ANGLE 180
#define SERVO2_DEFAULT_ANGLE 90

#define TASK_LINE_FOLLOWING_PRIO 3
#define TASK_SENSOR_COLLECTION_PRIO 2
#define TASK_SENSOR_COLLECTION_PRIO 2
#define TASK_DATA_REPORTING_PRIO 1
#define TASK_CONTROL_PRIO 3
#define TASK_MOTOR_PRIO 4
#define TASK_UI_PRIO 2

#define TASK_LINE_FOLLOWING_STACK_SIZE 4096
#define TASK_SENSOR_COLLECTION_STACK_SIZE 4096
#define TASK_DATA_REPORTING_STACK_SIZE 4096
#define TASK_CONTROL_STACK_SIZE 8192
#define TASK_MOTOR_STACK_SIZE 4096
#define TASK_UI_STACK_SIZE 4096

#define MOTOR_CMD_QUEUE_LEN 5
#define SENSOR_DATA_QUEUE_LEN 1

#define EVENT_RGB_TOGGLE_ON_OFF (1 << 0)
#define EVENT_RGB_BRIGHTNESS_CYCLE (1 << 1)

#define LIS3DHTR_I2C_ADDRESS 0x19

#define WIFI_AP_SSID "SmartCar_AP"
#define WIFI_AP_PASSWORD "12345678"

#define BLE_SERVER_NAME "SmartCar_BLE"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/**
 * @brief 电机控制指令结构体
 */
typedef struct
{
  int16_t lf_speed;  ///< 左前轮速度
  int16_t rf_speed;  ///< 右前轮速度
  int16_t lr_speed;  ///< 左后轮速度
  int16_t rr_speed;  ///< 右后轮速度
} MotorCommand;

/**
 * @brief 舵机控制指令结构体
 */
typedef struct
{
  uint8_t servo1_angle; ///< 舵机1角度
  uint8_t servo2_angle; ///< 舵机2角度
} ServoCommand;

/**
 * @brief 传感器数据包结构体
 */
typedef struct
{
  float temperature;              ///< 温度
  float humidity;                 ///< 湿度
  float acc_x;                    ///< 加速度X
  float acc_y;                    ///< 加速度Y
  float acc_z;                    ///< 加速度Z
  bool accelerometer_available;   ///< 加速度计是否可用
  long ultrasonic_dist;           ///< 超声波距离
  int light_level;                ///< 光照强度
  Find_Box_st face_result;        ///< 人脸识别结果
  bool face_detected;             ///< 是否检测到人脸
} SensorDataPacket;

QueueHandle_t g_motor_cmd_queue;
QueueHandle_t g_servo_cmd_queue;
QueueHandle_t g_sensor_data_queue;
EventGroupHandle_t g_ui_event_group;

bool g_is_accelerometer_available = false;
bool g_is_ps2_connected = false;
BLECharacteristic *pCharacteristic;
bool g_device_connected = false;

PS2X ps2x;
LIS3DHTR<TwoWire> LIS;
DHT dht(DHT_PIN, DHT11);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
WonderK210 *wk;
CRGB g_leds[NUM_RGB_LEDS];

hw_timer_t *g_servo_timer = NULL;
portMUX_TYPE g_servo_timer_mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t g_servo1_pulse_ticks;
volatile uint16_t g_servo2_pulse_ticks;
#define SERVO_PWM_PERIOD_TICKS (SERVO_PWM_PERIOD_MS * 1000 / SERVO_TIMER_TICK_US)

/**
 * @brief 舵机定时器中断服务函数
 */
void ARDUINO_ISR_ATTR on_servo_timer() {
  portENTER_CRITICAL_ISR(&g_servo_timer_mux);
  static uint16_t counter = 0;
  if (counter == 0) {
    digitalWrite(SERVO1_PIN, HIGH);
    digitalWrite(SERVO2_PIN, HIGH);
  }
  if (counter >= g_servo1_pulse_ticks) {
    digitalWrite(SERVO1_PIN, LOW);
  }
  if (counter >= g_servo2_pulse_ticks) {
    digitalWrite(SERVO2_PIN, LOW);
  }
  counter++;
  if (counter >= SERVO_PWM_PERIOD_TICKS) {
    counter = 0;
  }
  portEXIT_CRITICAL_ISR(&g_servo_timer_mux);
}

/**
 * @brief 更新指定舵机的PWM脉宽
 * @param servo_num 舵机编号（1或2）
 * @param angle 舵机角度（0-180）
 */
void hal_servo_update_pulse(uint8_t servo_num, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  uint16_t pulse_ticks = pulse_us / SERVO_TIMER_TICK_US;

  portENTER_CRITICAL(&g_servo_timer_mux);
  if (servo_num == 1)
    g_servo1_pulse_ticks = pulse_ticks;
  else if (servo_num == 2)
    g_servo2_pulse_ticks = pulse_ticks;
  portEXIT_CRITICAL(&g_servo_timer_mux);
}

/**
 * @brief 初始化舵机PWM相关硬件
 */
void hal_servo_init() {
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  g_servo_timer = timerBegin(SERVO_TIMER_FREQUENCY);
  timerAttachInterrupt(g_servo_timer, &on_servo_timer);
  timerAlarm(g_servo_timer, SERVO_TIMER_TICK_US, true, 0);
  hal_servo_update_pulse(1, SERVO1_DEFAULT_ANGLE);
  hal_servo_update_pulse(2, SERVO2_DEFAULT_ANGLE);
  Serial.println("HAL: Servos initialized and reset to 90 degrees.");
}

/**
 * @brief 设置电机速度
 * @param fwd_pin 前进PWM引脚
 * @param rev_pin 后退PWM引脚
 * @param speed 速度（-255~255）
 */
void hal_motor_set_speed(uint8_t fwd_pin, uint8_t rev_pin, int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(fwd_pin, speed);
    analogWrite(rev_pin, 0);
  } else if (speed < 0) {
    analogWrite(fwd_pin, 0);
    analogWrite(rev_pin, -speed);
  } else {
    analogWrite(fwd_pin, 0);
    analogWrite(rev_pin, 0);
  }
}

/**
 * @brief 执行电机控制指令
 * @param cmd 电机指令
 */
void hal_motor_write_command(const MotorCommand cmd) {
  hal_motor_set_speed(LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, -cmd.lf_speed);
  hal_motor_set_speed(RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM, -cmd.rf_speed);
  hal_motor_set_speed(LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, -cmd.lr_speed);
  hal_motor_set_speed(RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM, -cmd.rr_speed);
}

/**
 * @brief 初始化所有电机相关硬件
 */
void hal_motor_init() {
  uint8_t motorPins[] = {
    LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM,
    LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM
  };
  for (uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    analogWriteFrequency(pin, 5000);
  }
  MotorCommand stop_cmd = { 0, 0, 0, 0 };
  hal_motor_write_command(stop_cmd);
  Serial.println("HAL: Motors initialized.");
}

/**
 * @brief 初始化RGB灯带
 */
void hal_rgb_init() {
  FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(g_leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(INITIAL_RGB_BRIGHTNESS);
  fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
  FastLED.show();
  Serial.println("HAL: RGB LEDs initialized.");
}

/**
 * @brief 检查I2C设备是否存在
 * @param address I2C地址
 * @return true 存在, false 不存在
 */
bool hal_i2c_check_device(byte address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

/**
 * @brief 初始化所有传感器
 */
void hal_sensors_init() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (hal_i2c_check_device(LIS3DHTR_I2C_ADDRESS)) {
    g_is_accelerometer_available = true;
    LIS.begin(Wire, LIS3DHTR_I2C_ADDRESS);
    LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    Serial.println("HAL: LIS3DHTR (Accelerometer) found and initialized.");
  } else {
    g_is_accelerometer_available = false;
    Serial.println("HAL WARNING: LIS3DHTR (Accelerometer) not found. Skipping initialization.");
  }
  dht.begin();
  pinMode(LIGHT_PIN, INPUT);
  Serial1.begin(115200, SERIAL_8N1, K210_RX_PIN, K210_TX_PIN);
  wk = new WonderK210(&Serial1);
  Serial.println("HAL: All sensors initialized.");
}

/**
 * @brief 初始化ASR语音串口
 */
void hal_comms_init() {
  Serial2.begin(115200, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);
  Serial.println("HAL: ASR serial initialized.");
}

/**
 * @brief 初始化PS2手柄
 */
void hal_ps2_init() {
  int error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_CS_PIN, PS2_DATA_PIN, true, true);

  if (error == 0) {
    g_is_ps2_connected = true;
    Serial.println("HAL: PS2 Controller configured successfully.");
  } else {
    g_is_ps2_connected = false;
    Serial.printf("HAL WARNING: PS2 Controller not found or failed to configure (error code: %d)\n", error);
  }
}

/**
 * @brief 初始化用户按键
 */
void hal_button_init() {
  pinMode(USER_BUTTON_A_PIN, INPUT_PULLUP);
  Serial.println("HAL: User button initialized.");
}

/**
 * @brief BLE服务器回调类
 */
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    g_device_connected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer *pServer) {
    g_device_connected = false;
    Serial.println("BLE Client Disconnected");
    pServer->getAdvertising()->start();
    Serial.println("Restarting BLE advertising...");
  }
};

/**
 * @brief 初始化WiFi AP模式
 */
void init_wifi_ap() {
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)) {
    Serial.print("WiFi AP Started. SSID: ");
    Serial.println(WIFI_AP_SSID);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Error: Failed to start WiFi AP.");
  }
}

/**
 * @brief 初始化BLE服务器
 */
void init_ble_server() {
  BLEDevice::init(BLE_SERVER_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setValue("Hello from SmartCar!");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE Server started and advertising.");
}

/**
 * @brief 电机与舵机控制任务
 * @param pvParameters 未使用
 */
void vTaskMotorControl(void *pvParameters) {
  MotorCommand cmd;
  ServoCommand servo_cmd;

  Serial.println("TASK: Motor & Servo Control task started.");

  for (;;) {
    if (xQueueReceive(g_motor_cmd_queue, &cmd, 0) == pdPASS) {
      hal_motor_write_command(cmd);
    }
    if (xQueueReceive(g_servo_cmd_queue, &servo_cmd, 0) == pdPASS) {
      hal_servo_update_pulse(1, servo_cmd.servo1_angle);
      hal_servo_update_pulse(2, servo_cmd.servo2_angle);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief PS2手柄控制任务
 * @param pvParameters 未使用
 */
void vTaskControl(void *pvParameters) {
  MotorCommand motor_cmd = { 0, 0, 0, 0 };
  ServoCommand servo_cmd = { SERVO1_DEFAULT_ANGLE, SERVO2_DEFAULT_ANGLE };
  const MotorCommand motor_cmd_stop = { 0, 0, 0, 0 };
  const ServoCommand servo_cmd_center = { SERVO1_DEFAULT_ANGLE, SERVO2_DEFAULT_ANGLE };

  Serial.println("TASK: PS2 Control task started.");

  for (;;) {
    if (g_is_ps2_connected) {
      if (ps2x.read_gamepad(false, false)) {
          int ly = ps2x.Analog(PSS_LY);
          int lx = ps2x.Analog(PSS_LX);
          int ry = ps2x.Analog(PSS_RY);
          int rx = ps2x.Analog(PSS_RX);

          int move_speed = 0;
          int strafe_speed = 0;
          int rotate_speed = 0;
          const int dpad_speed = 180;

          if (abs(ly - 128) > 15)
            move_speed = map(ly, 0, 255, 255, -255);
          if (abs(lx - 128) > 15)
            strafe_speed = map(lx, 0, 255, -255, 255);

          if (move_speed == 0 && strafe_speed == 0) {
            if (ps2x.Button(PSB_PAD_UP)) {
              move_speed = dpad_speed;
            } else if (ps2x.Button(PSB_PAD_DOWN)) {
              move_speed = -dpad_speed;
            }

            if (ps2x.Button(PSB_PAD_LEFT)) {
              strafe_speed = -dpad_speed;
            } else if (ps2x.Button(PSB_PAD_RIGHT)) {
              strafe_speed = dpad_speed;
            }
          }

          if (ps2x.Button(PSB_L1))
          rotate_speed = -200;
        if (ps2x.Button(PSB_R1))
            rotate_speed = 200;

          if (rotate_speed != 0) {
            motor_cmd.lf_speed = rotate_speed;
            motor_cmd.rf_speed = -rotate_speed;
            motor_cmd.lr_speed = rotate_speed;
            motor_cmd.rr_speed = -rotate_speed;
          } else {
            motor_cmd.lf_speed = move_speed + strafe_speed;
            motor_cmd.rf_speed = move_speed - strafe_speed;
            motor_cmd.lr_speed = move_speed - strafe_speed;
            motor_cmd.rr_speed = move_speed + strafe_speed;
          }

          xQueueSend(g_motor_cmd_queue, &motor_cmd, 0);

          if (abs(ry - 128) > 15 || abs(rx - 128) > 15) {
            servo_cmd.servo1_angle = map(ry, 0, 255, 180, 100);
            servo_cmd.servo2_angle = map(rx, 0, 255, 0, 180);
            xQueueSend(g_servo_cmd_queue, &servo_cmd, 0);
          }

        vTaskDelay(pdMS_TO_TICKS(50));
      } else {
        Serial.println("ERROR: PS2 Controller disconnected during runtime!");
        g_is_ps2_connected = false;
        xQueueSend(g_motor_cmd_queue, &motor_cmd_stop, 0);
        xQueueSend(g_servo_cmd_queue, &servo_cmd_center, 0);
      }
    } else {
      Serial.println("PS2 Control Task: Controller not connected. Attempting to reconnect...");
      if (ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_CS_PIN, PS2_DATA_PIN, true, true) == 0) {
        g_is_ps2_connected = true;
        Serial.println("PS2 Controller reconnected successfully!");
      }
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

/**
 * @brief 传感器数据采集任务
 * @param pvParameters 未使用
 */
void vTaskSensorCollection(void *pvParameters) {
  SensorDataPacket data_packet;
  Find_Box_st face_result_buffer;

  Serial.println("TASK: Sensor Collection task started.");

  for (;;) {
    float temp_hum_val[2] = { 0 };
    data_packet.humidity = dht.readHumidity();
    data_packet.temperature = dht.readTemperature();

    data_packet.accelerometer_available = g_is_accelerometer_available;
    if (g_is_accelerometer_available) {
      data_packet.acc_x = LIS.getAccelerationX();
      data_packet.acc_y = LIS.getAccelerationY();
      data_packet.acc_z = LIS.getAccelerationZ();
    } else {
      data_packet.acc_x = 0.0f;
      data_packet.acc_y = 0.0f;
      data_packet.acc_z = 0.0f;
    }

    data_packet.ultrasonic_dist = ultrasonic.MeasureInCentimeters();
    data_packet.light_level = analogRead(LIGHT_PIN);

    wk->update_data();
    if (wk->recive_box(&face_result_buffer, K210_FIND_FACE_YOLO)) {
      data_packet.face_result = face_result_buffer;
      data_packet.face_detected = true;
    } else {
      data_packet.face_detected = false;
    }

    xQueueOverwrite(g_sensor_data_queue, &data_packet);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * @brief 数据上报任务
 * @param pvParameters 未使用
 */
void vTaskDataReporting(void *pvParameters) {
  SensorDataPacket received_packet;

  Serial.println("TASK: Data Reporting task started.");

  for (;;) {
    if (xQueueReceive(g_sensor_data_queue, &received_packet, portMAX_DELAY) == pdPASS) {

      Serial.printf("--- Sensor Data @ %lu ms ---\n", millis());
      Serial.printf("Temp: %.1f C, Humidity: %.1f %%\n", received_packet.temperature, received_packet.humidity);
      if (received_packet.accelerometer_available) {
        Serial.printf("Accel (x,y,z): %.2f, %.2f, %.2f\n", received_packet.acc_x, received_packet.acc_y, received_packet.acc_z);
      } else {
        Serial.println("Accel: Not Available");
      }
      Serial.printf("Ultrasonic: %ld cm\n", received_packet.ultrasonic_dist);
      Serial.printf("Light Level: %d\n", received_packet.light_level);
      if (received_packet.face_detected) {
        Serial.printf("Face Detected at (x,y,w,h): %d, %d, %d, %d\n",
                      received_packet.face_result.x, received_packet.face_result.y,
                      received_packet.face_result.w, received_packet.face_result.h);
      } else {
        Serial.println("Face: Not Detected");
      }
      Serial.printf("-------------------------------\n\n");
    }
    }
}

/**
 * @brief 用户交互任务（RGB、按键、语音）
 * @param pvParameters 未使用
 */
void vTaskUI(void *pvParameters) {
  Serial.println("TASK: UI (RGB, Button, Voice) task started.");

  uint8_t hue = 0;
  bool rgb_on = true;
  uint8_t brightness_levels[] = { 10, 50, 100, 150, 200 };
  uint8_t current_brightness_index = 2;
  FastLED.setBrightness(brightness_levels[current_brightness_index]);

  for (;;) {
    if (digitalRead(USER_BUTTON_A_PIN) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(20));
      if (digitalRead(USER_BUTTON_A_PIN) == LOW) {
        xEventGroupSetBits(g_ui_event_group, EVENT_RGB_BRIGHTNESS_CYCLE);
        while (digitalRead(USER_BUTTON_A_PIN) == LOW)
          vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    if (Serial2.available() > 0) {
      String command = Serial2.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("RGB:ON")) {
        if (!rgb_on)
          xEventGroupSetBits(g_ui_event_group, EVENT_RGB_TOGGLE_ON_OFF);
      } else if (command.equalsIgnoreCase("RGB:OFF")) {
        if (rgb_on)
          xEventGroupSetBits(g_ui_event_group, EVENT_RGB_TOGGLE_ON_OFF);
      }
    }

    EventBits_t bits = xEventGroupWaitBits(g_ui_event_group,
                                           EVENT_RGB_TOGGLE_ON_OFF | EVENT_RGB_BRIGHTNESS_CYCLE,
                                           pdTRUE,
                                           pdFALSE,
                                           0);

    if (bits & EVENT_RGB_TOGGLE_ON_OFF) {
      rgb_on = !rgb_on;
      Serial.printf("UI Event: RGB Toggled to %s\n", rgb_on ? "ON" : "OFF");
      if (!rgb_on) {
        fill_solid(g_leds, NUM_RGB_LEDS, CRGB::Black);
        FastLED.show();
      }
    }

    if (bits & EVENT_RGB_BRIGHTNESS_CYCLE) {
      current_brightness_index = (current_brightness_index + 1) % (sizeof(brightness_levels) / sizeof(uint8_t));
      uint8_t new_brightness = brightness_levels[current_brightness_index];
      FastLED.setBrightness(new_brightness);
      Serial.printf("UI Event: Brightness cycled to %d\n", new_brightness);
    }

    if (rgb_on) {
          fill_rainbow(g_leds, NUM_RGB_LEDS, hue++, 7);
      FastLED.show();
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/**
 * @brief 主程序初始化
 */
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("\n\n===== FreeRTOS Smart Car Initializing =====");

  init_wifi_ap();
  init_ble_server();

  hal_motor_init();
  hal_servo_init();
  hal_rgb_init();
  hal_sensors_init();
  hal_comms_init();
  hal_ps2_init();
  hal_button_init();

  g_motor_cmd_queue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(MotorCommand));
  g_servo_cmd_queue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(ServoCommand));
  g_sensor_data_queue = xQueueCreate(SENSOR_DATA_QUEUE_LEN, sizeof(SensorDataPacket));
  g_ui_event_group = xEventGroupCreate();

  if (!g_motor_cmd_queue || !g_servo_cmd_queue || !g_sensor_data_queue || !g_ui_event_group) {
    Serial.println("FATAL: Failed to create RTOS objects.");
    while (1)
      ;
  }

  xTaskCreatePinnedToCore(
    vTaskMotorControl,
    "MotorControlTask",
    TASK_MOTOR_STACK_SIZE,
    NULL,
    TASK_MOTOR_PRIO,
    NULL,
    1);

  xTaskCreatePinnedToCore(
    vTaskControl,
    "PS2ControlTask",
    TASK_CONTROL_STACK_SIZE,
    NULL,
    TASK_CONTROL_PRIO,
    NULL,
    1);

  xTaskCreatePinnedToCore(
    vTaskSensorCollection,
    "SensorCollectionTask",
    TASK_SENSOR_COLLECTION_STACK_SIZE,
    NULL,
    TASK_SENSOR_COLLECTION_PRIO,
    NULL,
    1);

  xTaskCreatePinnedToCore(
    vTaskDataReporting,
    "DataReportingTask",
    TASK_DATA_REPORTING_STACK_SIZE,
    NULL,
    TASK_DATA_REPORTING_PRIO,
    NULL,
    1);

  xTaskCreatePinnedToCore(
    vTaskUI,
    "UITask",
    TASK_UI_STACK_SIZE,
    NULL,
    TASK_UI_PRIO,
    NULL,
    1);

  Serial.println("===== System Initialization Complete. Starting Tasks. =====");
}

/**
 * @brief 主循环（空，所有逻辑在任务中）
 */
void loop() {
  vTaskDelete(NULL);
}