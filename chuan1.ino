// V2.0 - Line follower với PD controller + cảm biến màu TCS34725

#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Cấu hình cảm biến màu
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Chân PWM điều khiển động cơ
#define PWM_PIN_L_A 2
#define PWM_PIN_L_B 10
#define PWM_PIN_R_A 6
#define PWM_PIN_R_B 5

// Chân cảm biến IR
#define SENSOR_1_PIN A4  // Analog pin 4
#define SENSOR_2_PIN A3
#define SENSOR_3_PIN A1
#define SENSOR_4_PIN A0

// LED
#define W_LED_ON 20
#define IR_LED_ON 21

#define threshold 3000  // Ngưỡng phân biệt line

// Hệ số PD
float Kp = 112.0;
float Kd = 90.0;

float last_error = 0;

// Tốc độ cơ bản động cơ
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
  if (tcs.begin())
  {
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
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

  // In giá trị cảm biến màu để debug

  // Nếu phát hiện nhiều màu đỏ (có thể chướng ngại vật), dừng robot
  if (r > (g * 1.6) && r > (b * 1.6))
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

  // Nếu không thấy line
  if (sum == 0)
  {
    stop();
    return;
  }

  // Tính error theo trọng số vị trí cảm biến
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // Tính đạo hàm (derivative)
  float derivative = error - last_error;
  last_error = error;

  // Tính hiệu chỉnh theo PD
  float correction = Kp * error + Kd * derivative;

  // Tính tốc độ động cơ trái và phải
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Giới hạn tốc độ trong khoảng 0-255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Điều khiển động cơ tiến
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);
  delay(1);
}

void stop()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
}
