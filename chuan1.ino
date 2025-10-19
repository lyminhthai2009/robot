#include <Wire.h>
#include "Adafruit_TCS34725.h"

// --- Cảm biến màu ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// --- Chân động cơ ---
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// --- Cảm biến IR ---
#define SENSOR_1_PIN A4 
#define SENSOR_2_PIN A3
#define SENSOR_3_PIN A1
#define SENSOR_4_PIN A0

// --- LED ---
#define W_LED_ON 20
#define IR_LED_ON 21

// --- Ngưỡng động (sẽ hiệu chỉnh sau) ---
int threshold1, threshold2, threshold3, threshold4;

// --- PID ---
float Kp = 120.0;
float Kd = 109.5;
float Ki = 0.097;

float last_error = 0;
float integral = 0;

// --- Thời gian rẽ ---
float thoigianre = 175;

// --- Tốc độ cơ bản ---
int baseSpeed = 255;

void setup()
{
  Serial.begin(115200);

  pinMode(PWM_PIN_L_A, OUTPUT);
  pinMode(PWM_PIN_L_B, OUTPUT);
  pinMode(PWM_PIN_R_A, OUTPUT);
  pinMode(PWM_PIN_R_B, OUTPUT);

  pinMode(W_LED_ON, OUTPUT);
  pinMode(IR_LED_ON, OUTPUT);
  digitalWrite(W_LED_ON, LOW);
  digitalWrite(IR_LED_ON, HIGH);

  #ifdef TCS_SENSOR
  if (tcs.begin()) {
    Serial.println("TCS34725 found");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (millis() < 10000) {
      digitalWrite(W_LED_ON, !digitalRead(W_LED_ON));
      delay(50);
    }
  }
  digitalWrite(W_LED_ON, LOW);
  #endif

  // --- Hiệu chỉnh cảm biến IR ---
  calibrateSensors();

  stop();
  delay(1000);
  Serial.println("Bắt đầu chạy...");
}

void loop()
{
  uint16_t r, g, b, c;
  #ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
  #endif

  // Dừng khi thấy màu đỏ (vật cản)
  if (r > (g * 2.6) && r > (b * 1.6)) {
    stop();
    delay(200);
    return;
  }

  // Đọc cảm biến IR theo ngưỡng đã hiệu chỉnh
  int s1 = (analogRead(SENSOR_1_PIN) <= threshold1) ? 1 : 0;
  int s2 = (analogRead(SENSOR_2_PIN) <= threshold2) ? 1 : 0;
  int s3 = (analogRead(SENSOR_3_PIN) <= threshold3) ? 1 : 0;
  int s4 = (analogRead(SENSOR_4_PIN) <= threshold4) ? 1 : 0;

  int sum = s1 + s2 + s3 + s4;

  // Ngã tư
  if (sum == 4) {
    re_phai();
    return;
  }

  // Mất line
  if (sum <= 1) {
    integral = 0;
    stop();
    return;
  }

  // Tính sai số theo trọng số
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // PID
  integral += error;
  if (abs(error) < 0.5) integral = 0;
  float derivative = error - last_error;
  last_error = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Tốc độ động cơ
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);

  delay(10);
}

// --- Hiệu chỉnh cảm biến ---
void calibrateSensors() {
  Serial.println("=== HIỆU CHỈNH CẢM BIẾN IR ===");
  delay(1000);

  int white1 = 0, white2 = 0, white3 = 0, white4 = 0;
  int black1 = 0, black2 = 0, black3 = 0, black4 = 0;

  Serial.println("👉 Đặt robot trên nền TRẮNG...");
  delay(3000);
  for (int i = 0; i < 50; i++) {
    white1 += analogRead(SENSOR_1_PIN);
    white2 += analogRead(SENSOR_2_PIN);
    white3 += analogRead(SENSOR_3_PIN);
    white4 += analogRead(SENSOR_4_PIN);
    delay(10);
  }

  Serial.println("👉 Đặt robot trên vạch ĐEN...");
  delay(3000);
  for (int i = 0; i < 50; i++) {
    black1 += analogRead(SENSOR_1_PIN);
    black2 += analogRead(SENSOR_2_PIN);
    black3 += analogRead(SENSOR_3_PIN);
    black4 += analogRead(SENSOR_4_PIN);
    delay(10);
  }

  threshold1 = (white1 / 50 + black1 / 50) / 2;
  threshold2 = (white2 / 50 + black2 / 50) / 2;
  threshold3 = (white3 / 50 + black3 / 50) / 2;
  threshold4 = (white4 / 50 + black4 / 50) / 2;

  Serial.println("✅ Hiệu chỉnh xong!");
  Serial.print("Ngưỡng 1: "); Serial.println(threshold1);
  Serial.print("Ngưỡng 2: "); Serial.println(threshold2);
  Serial.print("Ngưỡng 3: "); Serial.println(threshold3);
  Serial.print("Ngưỡng 4: "); Serial.println(threshold4);
  Serial.println("===============================");
}

// --- Stop robot ---
void stop() {
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
}

// --- Rẽ phải ---
void re_phai() {
  analogWrite(PWM_PIN_L_A, 80); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 80);
  analogWrite(PWM_PIN_R_B, 0);
  delay(10);

  int turnSpeed = 150; 
  analogWrite(PWM_PIN_L_A, turnSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, turnSpeed);
  
  delay(thoigianre); 

  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    int s2_check = (analogRead(SENSOR_2_PIN) <= threshold2) ? 1 : 0;
    int s3_check = (analogRead(SENSOR_3_PIN) <= threshold3) ? 1 : 0;
    if (s2_check == 1 || s3_check == 1) break;

    analogWrite(PWM_PIN_L_A, 100);
    analogWrite(PWM_PIN_L_B, 0);
    analogWrite(PWM_PIN_R_A, 100);
    analogWrite(PWM_PIN_R_B, 0);
  }
  
  stop();
  delay(25);
}
