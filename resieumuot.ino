#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// Chân cảm biến IR
#define SENSOR_1_PIN A4 
#define SENSOR_2_PIN A3
#define SENSOR_3_PIN A1
#define SENSOR_4_PIN A0

// LED
#define W_LED_ON 20
#define IR_LED_ON 21

#define threshold 3890

// ===== ĐIỀU CHỈNH PID CHO CUA GẮPP =====
float Kp = 85.0;      // GIẢM Kp để correction không quá mạnh
float Kd = 75.0;      // GIẢM Kd để giảm dao động
float Ki = 0.05;      // GIẢM Ki để tránh tích lũy quá nhanh

float last_error = 0;
float integral = 0;
float thoigianre = 175;

// ===== QUAN TRỌNG: TỐC ĐỘ CƠ BẢN VÀ TỐC ĐỘ TỐI THIỂU =====
int baseSpeed = 220;       // GIẢM từ 255 để còn "dư địa" điều chỉnh
int minSpeed = 80;         // TỐC ĐỘ TỐI THIỂU - bánh không bao giờ dừng hẳn!

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
  if (tcs.begin())
  {
    //("TCS34725 found");
  }
  else
  {
    //("No TCS34725 found ... check your connections");
    while (millis() < 10000)
    {
      digitalWrite(W_LED_ON, !digitalRead(W_LED_ON));
      delay(50);
    }
  }
  digitalWrite(W_LED_ON, LOW);
  #endif

  stop();
  delay(1000);
}

void loop()
{
  uint16_t r, g, b, c;
  #ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
  #endif

  // Phát hiện màu đỏ - dừng
  if (r > (g * 2.6) && r > (b * 1.6))
  {
    stop();
    delay(200);
    return;
  }

  // Đọc cảm biến IR
  int s1 = (analogRead(SENSOR_1_PIN) <= threshold) ? 1 : 0;
  int s2 = (analogRead(SENSOR_2_PIN) <= threshold) ? 1 : 0;
  int s3 = (analogRead(SENSOR_3_PIN) <= threshold) ? 1 : 0;
  int s4 = (analogRead(SENSOR_4_PIN) <= threshold) ? 1 : 0;

  int sum = s1 + s2 + s3 + s4;
  
  // Phát hiện ngã tư (4 sensor)
  if (sum == 4){
    re_phai();
    return;
  }

  // Mất line
  if (sum == 0)
  {
    integral = 0;  // Reset integral khi mất line
    stop();
    return;
  }

  // Tính error
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // ===== GIỚI HẠN INTEGRAL CHẶT CHẼ HƠN =====
  integral += error;
  
  // Reset integral khi gần đúng line
  if (abs(error) < 0.3) { 
    integral *= 0.5;  // Giảm dần thay vì reset đột ngột
  }
  
  // QUAN TRỌNG: Giới hạn integral để tránh windup
  integral = constrain(integral, -100, 100);

  // Tính đạo hàm
  float derivative = error - last_error;
  last_error = error;

  // Tính correction
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // ===== ĐIỀU CHỈNH TỐC ĐỘ THÔNG MINH =====
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // QUAN TRỌNG: Giới hạn tốc độ với minSpeed thay vì 0
  // Điều này đảm bảo cả 2 bánh luôn chạy, không bao giờ dừng hẳn
  leftSpeed = constrain(leftSpeed, minSpeed, 255);
  rightSpeed = constrain(rightSpeed, minSpeed, 255);

  // Điều khiển động cơ
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);

  delay(5);  // GIẢM delay để phản ứng nhanh hơn
}

void stop()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
}

void re_phai()
{
  // Đi thẳng qua ngã tư một chút
  analogWrite(PWM_PIN_L_A, 100); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 100);
  analogWrite(PWM_PIN_R_B, 0);
  delay(10);

  // Xoay phải với tốc độ vừa phải
  int turnSpeed = 150; 
  analogWrite(PWM_PIN_L_A, turnSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, turnSpeed);
  delay(thoigianre); 

  // Dò lại vạch
  unsigned long startTime = millis();
  while(millis() - startTime < 1000)
  {
    int s2_check = (analogRead(SENSOR_2_PIN) <= threshold) ? 1 : 0;
    int s3_check = (analogRead(SENSOR_3_PIN) <= threshold) ? 1 : 0;

    if (s2_check == 1 || s3_check == 1) {
      break; 
    }

    analogWrite(PWM_PIN_L_A, 100);
    analogWrite(PWM_PIN_L_B, 0);
    analogWrite(PWM_PIN_R_A, 100);
    analogWrite(PWM_PIN_R_B, 0);
  }
  
  stop();
  delay(25);
  
  // QUAN TRỌNG: Reset integral sau khi rẽ
  integral = 0;
  last_error = 0;
}
