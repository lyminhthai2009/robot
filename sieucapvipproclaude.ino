#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Bật/tắt cảm biến màu (comment dòng này nếu không dùng)
#define TCS_SENSOR

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ========== CẤU HÌNH PWM CHO ESP32 (Core 3.x) ==========
#define PWM_FREQ 1000      // Tần số PWM: 1000Hz (thử 5000Hz nếu muốn êm hơn)
#define PWM_RESOLUTION 8   // Độ phân giải 8-bit (0-255)

// Chân GPIO cho động cơ
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// Chân cảm biến IR
#define SENSOR_1_PIN 4   // A4
#define SENSOR_2_PIN 3   // A3
#define SENSOR_3_PIN 1   // A1
#define SENSOR_4_PIN 0   // A0

// Mảng chân cảm biến để dễ duyệt
const int sensorPins[4] = {SENSOR_1_PIN, SENSOR_2_PIN, SENSOR_3_PIN, SENSOR_4_PIN};

// LED
#define W_LED_ON 20
#define IR_LED_ON 21

// Mảng threshold cho từng cảm biến (sẽ được tính tự động)
int threshold[4] = {3890, 3890, 3890, 3890};  // Giá trị mặc định

// Hệ số PID
float Kp = 120.0;
float Kd = 109.5;
float Ki = 0.097;

float last_error = 0;
float integral = 0;
const float INTEGRAL_LIMIT = 500.0;  // Giới hạn integral

float thoigianre = 175;

// Tốc độ cơ bản động cơ
int baseSpeed = 255;

// ========== HÀM CALIBRATION ==========
void calibrateSensors() {
  int minVal[4] = {4095, 4095, 4095, 4095};  // ESP32 ADC 12-bit (0-4095)
  int maxVal[4] = {0, 0, 0, 0};
  
  Serial.println("\n=== BẮT ĐẦU CALIBRATION ===");
  Serial.println("Giai đoạn 1: Đặt robot trên NỀN TRẮNG...");
  
  // Nhấp nháy LED báo hiệu
  for (int blink = 0; blink < 3; blink++) {
    digitalWrite(W_LED_ON, HIGH);
    delay(200);
    digitalWrite(W_LED_ON, LOW);
    delay(200);
  }
  
  // Giai đoạn 1: Đọc giá trị trên nền TRẮNG
  Serial.println("Đang đọc giá trị nền TRẮNG...");
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {  // 3 giây
    for (int i = 0; i < 4; i++) {
      int value = analogRead(sensorPins[i]);
      if (value > maxVal[i]) maxVal[i] = value;
    }
    digitalWrite(W_LED_ON, (millis() / 100) % 2);
    delay(10);
  }
  digitalWrite(W_LED_ON, LOW);
  
  Serial.println("Giai đoạn 2: Di chuyển robot qua VẠCH ĐEN...");
  
  // Nhấp nháy LED báo hiệu
  for (int blink = 0; blink < 3; blink++) {
    digitalWrite(W_LED_ON, HIGH);
    delay(200);
    digitalWrite(W_LED_ON, LOW);
    delay(200);
  }
  
  // Giai đoạn 2: Đọc giá trị trên vạch ĐEN
  Serial.println("Đang đọc giá trị vạch ĐEN...");
  startTime = millis();
  while (millis() - startTime < 3000) {  // 3 giây
    for (int i = 0; i < 4; i++) {
      int value = analogRead(sensorPins[i]);
      if (value < minVal[i]) minVal[i] = value;
    }
    digitalWrite(W_LED_ON, (millis() / 100) % 2);
    delay(10);
  }
  digitalWrite(W_LED_ON, LOW);
  
  // Tính threshold cho từng cảm biến
  Serial.println("\n=== KẾT QUẢ CALIBRATION ===");
  for (int i = 0; i < 4; i++) {
    int range = maxVal[i] - minVal[i];
    
    // Kiểm tra xem có đủ độ tương phản không
    if (range < 500) {
      Serial.print("CẢNH BÁO: Sensor ");
      Serial.print(i + 1);
      Serial.println(" - Độ tương phản thấp!");
    }
    
    // Threshold = giá trị giữa + 10% về phía đen (an toàn hơn)
    threshold[i] = minVal[i] + (range / 2) + (range * 0.1);
    
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": Min=");
    Serial.print(minVal[i]);
    Serial.print(", Max=");
    Serial.print(maxVal[i]);
    Serial.print(", Range=");
    Serial.print(range);
    Serial.print(", Threshold=");
    Serial.println(threshold[i]);
  }
  
  // Báo hiệu hoàn thành
  digitalWrite(W_LED_ON, HIGH);
  delay(1000);
  digitalWrite(W_LED_ON, LOW);
  
  Serial.println("=== CALIBRATION HOÀN TẤT ===");
  Serial.println("Bắt đầu chạy sau 2 giây...\n");
  delay(2000);
}

// ========== HÀM SETUP PWM CHO ESP32 CORE 3.x ==========
void setupMotorPWM() {
  ledcAttach(PWM_PIN_L_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_L_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_R_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_PIN_R_B, PWM_FREQ, PWM_RESOLUTION);
  
  Serial.println("Motor PWM initialized");
}

// ========== HÀM ĐIỀU KHIỂN ĐỘNG CƠ (ESP32 Core 3.x) ==========
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Động cơ trái
  if (leftSpeed >= 0) {
    ledcWrite(PWM_PIN_L_A, leftSpeed);
    ledcWrite(PWM_PIN_L_B, 0);
  } else {
    ledcWrite(PWM_PIN_L_A, 0);
    ledcWrite(PWM_PIN_L_B, -leftSpeed);
  }

  // Động cơ phải
  if (rightSpeed >= 0) {
    ledcWrite(PWM_PIN_R_A, rightSpeed);
    ledcWrite(PWM_PIN_R_B, 0);
  } else {
    ledcWrite(PWM_PIN_R_A, 0);
    ledcWrite(PWM_PIN_R_B, -rightSpeed);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32-C3 Line Follower Starting...");

  // Setup PWM cho ESP32
  setupMotorPWM();

  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, LOW);
  digitalWrite(IR_LED_ON, HIGH);

  #ifdef TCS_SENSOR
  if (tcs.begin()) {
    Serial.println("TCS34725 found");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      digitalWrite(W_LED_ON, !digitalRead(W_LED_ON));
      delay(50);
    }
  }
  digitalWrite(W_LED_ON, LOW);
  #endif

  stop();
  delay(1000);
  
  // ===== CHẠY CALIBRATION =====
  calibrateSensors();
  
  Serial.println("Ready!");
}

void loop() {
  uint16_t r = 0, g = 0, b = 0, c = 0;
  
  #ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
  
  // Phát hiện màu đỏ (chướng ngại vật)
  if (r > (g * 2.6) && r > (b * 1.6)) {
    stop();
    Serial.println("STOP: Red detected");
    delay(200);
    return;
  }
  #endif

  // Đọc cảm biến IR với threshold tự động
  int s1 = (analogRead(SENSOR_1_PIN) <= threshold[0]) ? 1 : 0;
  int s2 = (analogRead(SENSOR_2_PIN) <= threshold[1]) ? 1 : 0;
  int s3 = (analogRead(SENSOR_3_PIN) <= threshold[2]) ? 1 : 0;
  int s4 = (analogRead(SENSOR_4_PIN) <= threshold[3]) ? 1 : 0;

  int sum = s1 + s2 + s3 + s4;

  // Phát hiện ngã tư (cả 4 cảm biến thấy line)
  if (sum == 4) {
    re_phai();
    return;
  }

  // Mất line
  if (sum == 0) {
    integral = 0;  // Reset integral
    stop();
    Serial.println("LOST LINE");
    return;
  }

  // Tính error
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // Tính Integral với anti-windup
  integral += error;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  // Reset integral khi gần đúng line
  if (abs(error) < 0.5) {
    integral *= 0.5;  // Giảm dần thay vì reset hoàn toàn
  }

  // Tính Derivative
  float derivative = error - last_error;
  last_error = error;

  // Tính PID
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Tính tốc độ động cơ
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Giới hạn tốc độ
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Điều khiển động cơ
  setMotorSpeed(leftSpeed, rightSpeed);

  delay(5);
}

void stop() {
  setMotorSpeed(0, 0);
}

void re_phai()
{
  // --- Hành động 1: Đi thẳng qua ngã tư (đã điều chỉnh) ---
  // GIẢM thời gian delay, thậm chí có thể bỏ nếu robot của bạn rẽ tốt hơn khi không có nó.
  // Bắt đầu bằng cách giảm tốc độ và thời gian.
  analogWrite(PWM_PIN_L_A, 80); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 80);
  analogWrite(PWM_PIN_R_B, 0);
  delay(10); // Giảm từ 100ms xuống 50ms, bạn có thể thử với 20ms hoặc 0ms.

  // --- Hành động 2: Xoay phải tại chỗ (đã điều chỉnh) ---
  // QUAN TRỌNG: GIẢM TỐC ĐỘ để tránh sụt áp và trượt bánh.
  // Tốc độ 180-200 thường là đủ mạnh để xoay mà không gây sụt áp.
  int turnSpeed = 150; 
  analogWrite(PWM_PIN_L_A, turnSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, turnSpeed);
  
  // Bạn có thể cần điều chỉnh lại thời gian delay này cho phù hợp với tốc độ xoay mới
  // để robot xoay được đúng góc 90 độ.
  delay(thoigianre); 

  // --- Hành động 3: Dò lại vạch sau khi xoay ---
  // Phần này giữ nguyên, nhưng có thể thêm một điều kiện an toàn để thoát vòng lặp
  // nếu không tìm thấy line sau một thời gian nhất định.
  unsigned long startTime = millis();
  while(millis() - startTime < 1000) // Thêm giới hạn thời gian 1 giây để tránh bị kẹt
  {
    int s2_check = (analogRead(SENSOR_2_PIN) <= threshold) ? 1 : 0;
    int s3_check = (analogRead(SENSOR_3_PIN) <= threshold) ? 1 : 0;

    // Nếu một trong 2 cảm biến giữa đã thấy vạch đen thì thoát
    if (s2_check == 1 || s3_check == 1) {
      break; 
    }

    // Nếu chưa thấy, tiếp tục đi thẳng với tốc độ chậm
    analogWrite(PWM_PIN_L_A, 100);
    analogWrite(PWM_PIN_L_B, 0);
    analogWrite(PWM_PIN_R_A, 100);
    analogWrite(PWM_PIN_R_B, 0);
  }
  
  // Dừng một chút để ổn định trước khi quay lại dò line
  stop();
  delay(25);
}
