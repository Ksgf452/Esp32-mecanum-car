// =====================================================================
//  四麦轮小车 · 遥控端（ESP32 + 双轴摇杆 ×2）
//  功能：读取双摇杆 ADC → ESP-NOW 10ms 周期发送到车载端
//  接线：左摇杆 X→GPIO34(旋转) Y→GPIO35(LED)
//        右摇杆 X→GPIO32(左右平移) Y→GPIO33(前后)
// =====================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>

// ==================== 摇杆引脚 ====================
const int L_X_PIN = 34;   // 左摇杆 X → 原地旋转
const int L_Y_PIN = 35;   // 左摇杆 Y → LED 亮度
const int R_X_PIN = 32;   // 右摇杆 X → 左右平移
const int R_Y_PIN = 33;   // 右摇杆 Y → 前后平移

// ==================== 接收端 MAC ====================
// 【必须修改】填车载端串口打印的 STA MAC
// 示例：车载端 MAC 为 38:3E:51:58:1F:94 时填：
uint8_t carMAC[] = {0x38, 0x3E, 0x51, 0x58, 0x1F, 0x94};

// ==================== 数据包 ====================
typedef struct {
  int16_t left_x;
  int16_t left_y;
  int16_t right_x;
  int16_t right_y;
} JoystickPacket;

static JoystickPacket txPacket = {2048, 2048, 2048, 2048};

// ==================== 调度参数 ====================
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 10;   // 10ms → 100Hz
unsigned long lastErrPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  analogSetAttenuation(ADC_11db);
  pinMode(L_X_PIN, INPUT);
  pinMode(L_Y_PIN, INPUT);
  pinMode(R_X_PIN, INPUT);
  pinMode(R_Y_PIN, INPUT);

  // ---- Wi-Fi / ESP-NOW ----
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[遥控端] ESP-NOW 初始化失败");
    return;
  }

  // 强制锁定信道 1，两端必须一致
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, carMAC, 6);
  peerInfo.channel = 0;                    // 0 = 使用当前信道
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[遥控端] add_peer 失败，检查 MAC 地址");
    return;
  }
  Serial.println("[遥控端] peer 注册成功");

  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    char s[18];
    snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("[遥控端] 本机 MAC: "); Serial.println(s);
  }

  Serial.println("[遥控端] 就绪，开始发送");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSend >= SEND_INTERVAL) {
    txPacket.left_x  = analogRead(L_X_PIN);
    txPacket.left_y  = analogRead(L_Y_PIN);
    txPacket.right_x = analogRead(R_X_PIN);
    txPacket.right_y = analogRead(R_Y_PIN);

    esp_err_t r = esp_now_send(carMAC, (uint8_t *)&txPacket, sizeof(txPacket));

    if (r != ESP_OK && (now - lastErrPrint > 1000)) {
      Serial.printf("[遥控端] 发送失败, err=%d\n", r);
      lastErrPrint = now;
    }

    // 调试打印（限频 500ms）
    static unsigned long lastDbg = 0;
    if (now - lastDbg > 500) {
      Serial.printf("LX=%4d LY=%4d RX=%4d RY=%4d\n",
                    txPacket.left_x, txPacket.left_y,
                    txPacket.right_x, txPacket.right_y);
      lastDbg = now;
    }

    lastSend = now;
  }
}