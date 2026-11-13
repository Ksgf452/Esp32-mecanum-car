// =====================================================================
//  四麦轮小车 · 车载端（ESP32 + 双 TB6612FNG + MPU6050）
//  功能：ESP-NOW 接收摇杆 → 麦轮逆运动学 → MPU6050 偏航闭环补偿
//  布局：X 型辊子；方向校准：前进/旋转取反，平移保持
//  PID 补偿极性：已按实测取反（-correction）
// =====================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <Wire.h>

// =====================================================================
//  一、电机引脚
// =====================================================================
// TB6612 #1 —— 左侧：A=FL, B=BL
const int PWMA1 = 26, AIN1_1 = 27, AIN2_1 = 14;   // FL
const int PWMB1 = 12, BIN1_1 = 13, BIN2_1 = 15;   // BL
// TB6612 #2 —— 右侧：A=FR, B=BR
const int PWMA2 = 4,  AIN1_2 = 16, AIN2_2 = 17;   // FR
const int PWMB2 = 18, BIN1_2 = 19, BIN2_2 = 21;   // BR

const int STBY_PIN = 25;
const int LED_PIN  = 22;

// 方向校准：FL, FR, BL, BR。某轮反了改 -1
int8_t MOTOR_DIR[4] = {+1, +1, +1, +1};

// =====================================================================
//  二、MPU6050 引脚与 I2C
// =====================================================================
const int MPU_ADDR = 0x68;
const int I2C_SDA  = 23;
const int I2C_SCL  = 5;

// =====================================================================
//  三、摇杆数据包
// =====================================================================
typedef struct {
  int16_t left_x, left_y, right_x, right_y;
} JoystickPacket;

JoystickPacket rxData = {2048, 2048, 2048, 2048};  // 上电初始为中位

// =====================================================================
//  四、摇杆参数
// =====================================================================
const int ADC_CENTER = 2048;
const int DEADZONE   = 180;
const int ADC_MIN    = 0;
const int ADC_MAX    = 4095;

// =====================================================================
//  五、PWM 参数
// =====================================================================
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;        // 0–1023

// =====================================================================
//  六、运动学参数
// =====================================================================
// 旋转增益：1.0 = 旋转与满速前进等权；转太快降到 0.6~0.8
const float ROTATE_GAIN = 1.0f;

// =====================================================================
//  七、偏航 PID 参数
//  【PID 调参 1】按顺序整定，不要一上来全给大值：
//    - 先把 yawKi 和 yawKd 设为 0，yawKp 从 1.0 开始
//    - 做纯平移，观察车头偏转是否被抑制、是否低频摇摆
//        摇摆 → Kp 偏大；偏转仍在 → Kp 偏小
//    - Kp 调好后加入很小的 yawKi（0.1 起）消除残余偏转
//        Ki 过大会让车头缓慢"游走"
//    - 最后加 yawKd（0.05 起）抑制过冲；电机抖动就降回去
// =====================================================================
float yawKp = 1.0f;
float yawKi = 0.0f;
float yawKd = 0.0f;

// 积分限幅，防止饱和
const float YAW_INTEGRAL_LIMIT = 200.0f;

// 【PID 调参 2】补偿量限幅，对应 rotate 的 ±500 量级
// 纯平移时补偿"给不上劲"可适当加大；补偿过猛产生摆动则减小
const float YAW_CORRECTION_LIMIT = 300.0f;

// PID 状态
float yawIntegral  = 0;
float yawPrevError = 0;
float yawGyroOffset = 0;   // 陀螺仪零偏，setup 中校准

// =====================================================================
//  八、失联保护
// =====================================================================
volatile unsigned long lastRecvTime = 0;
volatile bool hasReceived = false;
const unsigned long TIMEOUT_MS = 500;

// =====================================================================
//  ESP-NOW 接收回调
// =====================================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(JoystickPacket)) {
    memcpy(&rxData, data, sizeof(rxData));
    lastRecvTime = millis();
    if (!hasReceived) {
      hasReceived = true;
      Serial.println("[车载端] 已收到首帧遥控数据");
    }
  }
}

// =====================================================================
//  MPU6050 读取偏航角速度（°/s，已减零偏）
// =====================================================================
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);              // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  int16_t raw = (Wire.read() << 8) | Wire.read();
  // ±500°/s 量程，灵敏度 65.5 LSB/(°/s)
  return (raw / 65.5f) - yawGyroOffset;
}

// =====================================================================
//  偏航补偿 PID
//  平移时期望偏航角速度为 0，实测非零 → 输出反向旋转补偿
//  【注意】返回值已取反（-correction），与 mecanumDrive 中 w=-rotate
//  的取反配合，使 PID 补偿的物理方向与摇杆旋转指令一致。
//  若实测发现平移时车头摆动加剧（正反馈），说明极性仍反，
//  把这里改回 constrain(correction, ...) 即可。
// =====================================================================
float computeYawCorrection(float dt) {
  float actualYawRate = readGyroZ();
  float targetYawRate = 0;
  float error = targetYawRate - actualYawRate;

  yawIntegral += error * dt;
  yawIntegral = constrain(yawIntegral, -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);

  float derivative = (error - yawPrevError) / dt;
  yawPrevError = error;

  float correction = yawKp * error + yawKi * yawIntegral + yawKd * derivative;
  return constrain(-correction, -YAW_CORRECTION_LIMIT, YAW_CORRECTION_LIMIT);
}

// =====================================================================
//  setup
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // ---- 电机方向脚先拉低 ----
  pinMode(AIN1_1, OUTPUT); digitalWrite(AIN1_1, LOW);
  pinMode(AIN2_1, OUTPUT); digitalWrite(AIN2_1, LOW);
  pinMode(BIN1_1, OUTPUT); digitalWrite(BIN1_1, LOW);
  pinMode(BIN2_1, OUTPUT); digitalWrite(BIN2_1, LOW);
  pinMode(AIN1_2, OUTPUT); digitalWrite(AIN1_2, LOW);
  pinMode(AIN2_2, OUTPUT); digitalWrite(AIN2_2, LOW);
  pinMode(BIN1_2, OUTPUT); digitalWrite(BIN1_2, LOW);
  pinMode(BIN2_2, OUTPUT); digitalWrite(BIN2_2, LOW);

  // ---- PWM 绑定并清零 ----
  ledcAttachChannel(PWMA1, PWM_FREQ, PWM_RES, 0);   // FL
  ledcAttachChannel(PWMA2, PWM_FREQ, PWM_RES, 1);   // FR
  ledcAttachChannel(PWMB1, PWM_FREQ, PWM_RES, 2);   // BL
  ledcAttachChannel(PWMB2, PWM_FREQ, PWM_RES, 3);   // BR
  ledcAttachChannel(LED_PIN, 5000, 8, 4);

  ledcWrite(PWMA1, 0); ledcWrite(PWMA2, 0);
  ledcWrite(PWMB1, 0); ledcWrite(PWMB2, 0);
  ledcWrite(LED_PIN, 0);

  // ---- STBY 最后使能 ----
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  delay(10);
  digitalWrite(STBY_PIN, HIGH);

  // ---- MPU6050 初始化 ----
  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // 唤醒
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG
  Wire.write(0x08);   // ±500°/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);   // CONFIG
  Wire.write(0x03);   // DLPF 44Hz，抑制噪声
  Wire.endTransmission();

  delay(100);

  // ---- 陀螺仪零偏校准：静止取平均 ----
  // 【重要】校准时小车必须完全静止放平，否则零偏会带错
  Serial.println("[车载端] 陀螺仪零偏校准中，请保持小车静止...");
  const int N = 200;
  float sum = 0;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    sum += (raw / 65.5f);
    delay(5);
  }
  yawGyroOffset = sum / N;
  Serial.printf("[车载端] 陀螺仪零偏: %.3f °/s\n", yawGyroOffset);

  // ---- Wi-Fi / ESP-NOW ----
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[车载端] ESP-NOW 初始化失败");
    return;
  }

  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    char s[18];
    snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("[车载端] 真实 MAC: "); Serial.println(s);
  }

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("[车载端] 就绪，等待遥控数据...");
}

// =====================================================================
//  loop
// =====================================================================
void loop() {
  if (!hasReceived || (millis() - lastRecvTime > TIMEOUT_MS)) {
    stopAllMotors();
    // 失联时清 PID 状态，避免重新连上时积分项残留
    yawIntegral = 0;
    yawPrevError = 0;
    return;
  }

  // 固定控制周期约 100Hz，保证 PID 的 dt 稳定
  static unsigned long lastControlTime = 0;
  unsigned long now = millis();
  float dt = (now - lastControlTime) / 1000.0f;
  if (dt < 0.01f) return;
  lastControlTime = now;

  // ---- 摇杆解析 ----
  int lx = applyDeadzone(rxData.left_x);    // 旋转
  int ly = rxData.left_y;                   // LED
  int rx = applyDeadzone(rxData.right_x);   // 左右平移
  int ry = applyDeadzone(rxData.right_y);   // 前后

  int forward =  ry * 500 / 2048;
  int strafe  =  rx * 500 / 2048;
  int rotate_user = lx * 500 / 2048;

  // ---- 偏航补偿 ----
  // 【PID 调参 3】纯平移时 rotate_user≈0，补偿量自动抵消跑偏；
  // 主动旋转时补偿量叠加在摇杆指令上，两者协同工作。
  int rotate_comp  = (int)computeYawCorrection(dt);
  int rotate_final = constrain(rotate_user + rotate_comp, -500, 500);

  // ---- 驱动 ----
  mecanumDrive(forward, strafe, rotate_final);
  setLEDBrightness(ly);

  // ---- 调试打印（限频 500ms）----
  // 【PID 调参 4】观察 comp 一栏：
  //   纯平移时 comp 应有明显非零值在"对抗"偏转；
  //   若 comp 长期贴到 ±300 限幅，说明 Kp 偏小或机械偏差过大。
  static unsigned long lastDbg = 0;
  if (now - lastDbg > 500) {
    Serial.printf("LX=%4d RY=%4d RX=%4d | fwd=%4d str=%4d rot_u=%4d comp=%4d rot_f=%4d gyro=%.1f\n",
                  rxData.left_x, rxData.right_y, rxData.right_x,
                  forward, strafe, rotate_user, rotate_comp, rotate_final,
                  readGyroZ());
    lastDbg = now;
  }
}

// =====================================================================
//  对称死区
// =====================================================================
int applyDeadzone(int raw) {
  raw = constrain(raw, ADC_MIN, ADC_MAX);
  int centered = raw - ADC_CENTER;
  if (abs(centered) <= DEADZONE) return 0;
  float norm = (float)(abs(centered) - DEADZONE) / (float)(ADC_CENTER - DEADZONE);
  norm = constrain(norm, 0.0f, 1.0f);
  int result = (int)(norm * 2048.0f);
  return (centered > 0) ? result : -result;
}

// =====================================================================
//  停车
// =====================================================================
void stopAllMotors() {
  digitalWrite(AIN1_1, LOW); digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, LOW); digitalWrite(BIN2_1, LOW);
  digitalWrite(AIN1_2, LOW); digitalWrite(AIN2_2, LOW);
  digitalWrite(BIN1_2, LOW); digitalWrite(BIN2_2, LOW);
  ledcWrite(PWMA1, 0); ledcWrite(PWMB1, 0);
  ledcWrite(PWMA2, 0); ledcWrite(PWMB2, 0);
}

// =====================================================================
//  单轮控制
//  wheel: 0=FL, 1=FR, 2=BL, 3=BR
// =====================================================================
void setMotor(int wheel, int pwmSigned) {
  int pwm = constrain(abs(pwmSigned), 0, 1023);
  switch (wheel) {
    case 0: // FL
      if (pwmSigned > 0)      { digitalWrite(AIN1_1, HIGH); digitalWrite(AIN2_1, LOW); }
      else if (pwmSigned < 0) { digitalWrite(AIN1_1, LOW);  digitalWrite(AIN2_1, HIGH); }
      else                    { digitalWrite(AIN1_1, LOW);  digitalWrite(AIN2_1, LOW); }
      ledcWrite(PWMA1, pwm); break;
    case 1: // FR
      if (pwmSigned > 0)      { digitalWrite(AIN1_2, HIGH); digitalWrite(AIN2_2, LOW); }
      else if (pwmSigned < 0) { digitalWrite(AIN1_2, LOW);  digitalWrite(AIN2_2, HIGH); }
      else                    { digitalWrite(AIN1_2, LOW);  digitalWrite(AIN2_2, LOW); }
      ledcWrite(PWMA2, pwm); break;
    case 2: // BL
      if (pwmSigned > 0)      { digitalWrite(BIN1_1, HIGH); digitalWrite(BIN2_1, LOW); }
      else if (pwmSigned < 0) { digitalWrite(BIN1_1, LOW);  digitalWrite(BIN2_1, HIGH); }
      else                    { digitalWrite(BIN1_1, LOW);  digitalWrite(BIN2_1, LOW); }
      ledcWrite(PWMB1, pwm); break;
    case 3: // BR
      if (pwmSigned > 0)      { digitalWrite(BIN1_2, HIGH); digitalWrite(BIN2_2, LOW); }
      else if (pwmSigned < 0) { digitalWrite(BIN1_2, LOW);  digitalWrite(BIN2_2, HIGH); }
      else                    { digitalWrite(BIN1_2, LOW);  digitalWrite(BIN2_2, LOW); }
      ledcWrite(PWMB2, pwm); break;
  }
}

// =====================================================================
//  麦轮逆运动学（X 型辊子布局，已按实测校准）
//  forward>0 前进；strafe>0 右移；rotate>0 顺时针
//  平移：前轮反转、后轮正转（已验证正确）
//  前进/旋转：取反修正物理方向
// =====================================================================
void mecanumDrive(int forward, int strafe, int rotate) {
  // 【方向校准】前进与旋转取反，平移保持不变
  float vx = -forward / 500.0f;
  float vy = strafe  / 500.0f;
  float w  = -rotate  / 500.0f;
  float K  = ROTATE_GAIN;

  float wFL = vx - vy + w * K;
  float wFR = vx - vy - w * K;
  float wBL = vx + vy + w * K;
  float wBR = vx + vy - w * K;

  // 归一化，保持方向不变
  float maxMag = 1.0f;
  maxMag = max(maxMag, fabs(wFL)); maxMag = max(maxMag, fabs(wFR));
  maxMag = max(maxMag, fabs(wBL)); maxMag = max(maxMag, fabs(wBR));
  if (maxMag > 1.0f) { wFL /= maxMag; wFR /= maxMag; wBL /= maxMag; wBR /= maxMag; }

  setMotor(0, (int)(wFL * 1023) * MOTOR_DIR[0]);
  setMotor(1, (int)(wFR * 1023) * MOTOR_DIR[1]);
  setMotor(2, (int)(wBL * 1023) * MOTOR_DIR[2]);
  setMotor(3, (int)(wBR * 1023) * MOTOR_DIR[3]);
}

// =====================================================================
//  LED 亮度
// =====================================================================
void setLEDBrightness(int rawY) {
  int brightness = map(constrain(rawY, 0, 4095), 0, 4095, 0, 255);
  ledcWrite(LED_PIN, brightness);
}