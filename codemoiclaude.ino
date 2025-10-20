#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

#define SENSOR_1_PIN A4 
#define SENSOR_2_PIN A3
#define SENSOR_3_PIN A1
#define SENSOR_4_PIN A0

#define W_LED_ON 20
#define IR_LED_ON 21

#define threshold 3890

// ===== THÔNG SỐ PID TỐI ƯU CHO VÀO CUA MƯỢT =====
float Kp = 45.0;     // Giảm mạnh để phản ứng mượt hơn
float Kd = 25.0;     // Giảm để không bị "cứng"
float Ki = 0.05;     // Giảm nhẹ để tránh tích lũy sai số

float last_error = 0;
float integral = 0;

// ===== ĐIỀU CHỈNH TỐC ĐỘ =====
int baseSpeed = 200;    // Giảm từ 255 xuống để vào cua dễ hơn
int minSpeed = 120;     // Tốc độ tối thiểu để motor không bị yếu
int maxSpeed = 255;     // Giữ nguyên

float thoigianre = 175;

// ===== LÀM MƯỢT ĐẠO HÀM =====
float smoothDerivative = 0;
float alpha = 0.6;  // Hệ số làm mượt (0.5-0.8 là tốt)

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
    Serial.println("TCS34725 found");
  }
  else
  {
    Serial.println("No TCS34725 found");
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

  // Phát hiện màu đỏ
  if (r > (g * 2.6) && r > (b * 1.6))
  {
    stop();
    Serial.println("RED DETECTED - STOP");
    delay(200);
    return;
  }

  // Đọc cảm biến IR
  int s1 = (analogRead(SENSOR_1_PIN) <= threshold) ? 1 : 0;
  int s2 = (analogRead(SENSOR_2_PIN) <= threshold) ? 1 : 0;
  int s3 = (analogRead(SENSOR_3_PIN) <= threshold) ? 1 : 0;
  int s4 = (analogRead(SENSOR_4_PIN) <= threshold) ? 1 : 0;

  int sum = s1 + s2 + s3 + s4;
  
  // Phát hiện ngã tư
  if (sum == 4){
    re_phai();
    return;
  }

  // Mất line
  if (sum == 0)
  {
    integral = 0;
    stop();
    Serial.println("LOST LINE");
    return;
  }

  // Tính error
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // Tính tích phân với chống windup
  integral += error;
  
  // Reset integral khi gần đúng line
  if (abs(error) < 0.5) {
    integral *= 0.8; // Giảm dần thay vì reset đột ngột
  }
  
  // Giới hạn integral để tránh tích lũy quá mức
  integral = constrain(integral, -100, 100);

  // Tính đạo hàm và làm mượt
  float derivative = error - last_error;
  smoothDerivative = alpha * smoothDerivative + (1 - alpha) * derivative;
  last_error = error;

  // Tính correction với đạo hàm đã làm mượt
  float correction = Kp * error + Ki * integral + Kd * smoothDerivative;
  
  // Giới hạn correction để tránh thay đổi quá đột ngột
  correction = constrain(correction, -120, 120);

  // ===== ĐIỀU CHỈNH TỐC ĐỘ ĐỘNG =====
  // Giảm tốc khi cua gấp (error lớn)
  int dynamicSpeed = baseSpeed;
  if (abs(error) > 2.0) {
    dynamicSpeed = baseSpeed * 0.8; // Giảm 20% khi cua gấp
  } else if (abs(error) > 1.5) {
    dynamicSpeed = baseSpeed * 0.9; // Giảm 10% khi cua vừa
  }

  // Tính tốc độ motor
  int leftSpeed = dynamicSpeed - correction;
  int rightSpeed = dynamicSpeed + correction;

  // Giới hạn tốc độ với minSpeed
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  // Điều khiển motor
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);

  // Debug
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | I: ");
  Serial.print(integral);
  Serial.print(" | D: ");
  Serial.print(smoothDerivative);
  Serial.print(" | Corr: ");
  Serial.print(correction);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" | R: ");
  Serial.println(rightSpeed);

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
  // Đi thẳng qua ngã tư với tốc độ vừa phải
  analogWrite(PWM_PIN_L_A, 100); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 100);
  analogWrite(PWM_PIN_R_B, 0);
  delay(50);

  // Xoay phải với tốc độ vừa để tránh trượt
  int turnSpeed = 160; 
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
  
  // Reset PID sau khi rẽ
  integral = 0;
  last_error = 0;
  smoothDerivative = 0;
}
