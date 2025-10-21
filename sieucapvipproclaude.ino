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

#define threshold 3890  // Ngưỡng phân biệt line

// Hệ số PID
float Kp = 120.0;
float Kd = 109.5;
float Ki = 0.097;

float last_error = 0;
float integral = 0;

float thoigianre = 175;

// === THÊM CÁC THAM SỐ MỚI ===
int baseSpeedNormal = 255;    // Tốc độ khi đi thẳng
int baseSpeedCurve = 180;     // Tốc độ khi vào cua (giảm xuống)
float curveThreshold = 1.5;   // Ngưỡng error để phát hiện cua
int minSpeed = 50;            // Tốc độ tối thiểu để bánh không bị khựng

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

  // Nếu phát hiện nhiều màu đỏ (có thể chướng ngại vật), dừng robot
  if (r > (g * 2.6) && r > (b * 1.6))
  {
    stop();
    delay(200);
    return;
  }

  // Đọc cảm biến IR và chuẩn hóa (0 = trên line, 1 = ngoài line)
  int s1 = (analogRead(SENSOR_1_PIN) <= threshold) ? 1 : 0;
  int s2 = (analogRead(SENSOR_2_PIN) <= threshold) ? 1 : 0;
  int s3 = (analogRead(SENSOR_3_PIN) <= threshold) ? 1 : 0;
  int s4 = (analogRead(SENSOR_4_PIN) <= threshold) ? 1 : 0;

  int sum = s1 + s2 + s3 + s4;
  if (sum == 4){
    re_phai();
    return;
  }

  // Nếu không thấy line
  if (sum == 0)
  {
    integral = 0;
    stop();
    return;
  }

  // Tính error theo trọng số vị trí cảm biến
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // Tính tích phân (Integral)
  integral += error;
  
  // Chống "Integral Windup"
  if (abs(error) < 0.5) {
    integral = 0;
  }

  // Tính đạo hàm (derivative)
  float derivative = error - last_error;
  last_error = error;

  // === PHẦN MỚI: GIẢM TỐC KHI VÀO CUA ===
  int baseSpeed;
  if (abs(error) > curveThreshold) {
    // Đang vào cua (error lớn) → giảm tốc
    baseSpeed = baseSpeedCurve;
  } else {
    // Đi thẳng hoặc cua nhẹ → tốc độ bình thường
    baseSpeed = baseSpeedNormal;
  }

  // Tính hiệu chỉnh theo PID
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Tính tốc độ động cơ trái và phải
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // === QUAN TRỌNG: ĐẢM BẢO TỐC ĐỘ TỐI THIỂU ===
  // Không để bánh nào xuống dưới minSpeed để tránh bị khựng
  leftSpeed = constrain(leftSpeed, minSpeed, 255);
  rightSpeed = constrain(rightSpeed, minSpeed, 255);

  // Điều khiển động cơ tiến
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);

  delay(10);
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
  // Đi thẳng qua ngã tư
  analogWrite(PWM_PIN_L_A, 80); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 80);
  analogWrite(PWM_PIN_R_B, 0);
  delay(10);

  // Xoay phải tại chỗ
  int turnSpeed = 150; 
  analogWrite(PWM_PIN_L_A, turnSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, turnSpeed);
  delay(thoigianre); 

  // Dò lại vạch sau khi xoay
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
}
