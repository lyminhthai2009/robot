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

#define threshold 3120  // Ngưỡng phân biệt line

// Hệ số PID
float Kp = 120.0;
float Kd = 109.5;
float Ki = 0.097;   // Bắt đầu với một giá trị Ki nhỏ

float last_error = 0;
float integral = 0; // Biến lưu trữ tổng sai số (thành phần I)

float thoigianre = 175;

// Tốc độ cơ bản động cơ
int baseSpeed = 255;

// Tốc độ tối thiểu để cả 2 bánh luôn quay khi vào cua
int minSpeed = 50; // Điều chỉnh từ 60-120 tùy robot của bạn

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
  
  // ===== KHỞI ĐỘNG MỀM - Tăng tốc từ từ để tránh sụt áp =====
  for(int speed = 0; speed <= minSpeed; speed += 10) {
    analogWrite(PWM_PIN_L_A, speed);
    analogWrite(PWM_PIN_L_B, 0);
    analogWrite(PWM_PIN_R_A, speed);
    analogWrite(PWM_PIN_R_B, 0);
    delay(30);
  }
  // ===== HẾT PHẦN KHỞI ĐỘNG MỀM =====
}

void loop()
{
  uint16_t r, g, b, c;
  #ifdef TCS_SENSOR
  tcs.getRawData(&r, &g, &b, &c);
  #endif

  // In giá trị cảm biến màu để debug

  // Nếu phát hiện nhiều màu đỏ (có thể chướng ngại vật), dừng robot
  if (r > (g * 2.6) && r > (b * 1.6))
  {
    stop();
    //(" - STOP: Red detected");
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
    // Khi mất line, reset integral để tránh robot bị "nhớ" sai số cũ
    integral = 0;
    stop();
    //(" - LOST LINE");
    return;
  }

  // Tính error theo trọng số vị trí cảm biến
  float error = (s1 * -3 + s2 * -1 + s3 * 1 + s4 * 3) / (float)sum;

  // Tính tích phân (Integral) bằng cách cộng dồn sai số
  integral += error;
  
  // Chống "Integral Windup": Nếu robot đi gần đúng vạch, reset integral để tránh nó tăng quá lớn
  if (abs(error) < 0.5) { // Nếu sai số rất nhỏ (robot gần như đi thẳng)
    integral = 0;
  }

  // Tính đạo hàm (derivative)
  float derivative = error - last_error;
  last_error = error;

  // Tính hiệu chỉnh theo PID
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // Tính tốc độ động cơ trái và phải
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Giới hạn tốc độ trong khoảng minSpeed-255 để cả 2 bánh luôn quay
  leftSpeed = constrain(leftSpeed, minSpeed, 255);
  rightSpeed = constrain(rightSpeed, minSpeed, 255);

  // Điều khiển động cơ tiến
  analogWrite(PWM_PIN_L_A, leftSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, rightSpeed);
  analogWrite(PWM_PIN_R_B, 0);

  //(" | Error=");
  //(error);
  //(" Correction=");
  //(correction);
  //(" LeftSpeed=");
  //(leftSpeed);
  //(" RightSpeed=");
  //(rightSpeed);

  delay(10);
}

void stop()
{
  analogWrite(PWM_PIN_L_A, 0);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, 0);
  //("STOP");
}

void re_phai()
{
  // --- Hành động 1: Đi thẳng qua ngã tư (đã điều chỉnh) ---
  analogWrite(PWM_PIN_L_A, 80); 
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 80);
  analogWrite(PWM_PIN_R_B, 0);
  delay(10);

  // --- Hành động 2: Xoay phải tại chỗ (đã điều chỉnh) ---
  int turnSpeed = 180; 
  analogWrite(PWM_PIN_L_A, turnSpeed);
  analogWrite(PWM_PIN_L_B, 0);
  analogWrite(PWM_PIN_R_A, 0);
  analogWrite(PWM_PIN_R_B, turnSpeed);
  
  delay(thoigianre); 

  // --- Hành động 3: Dò lại vạch sau khi xoay ---
  unsigned long startTime = millis();
  while(millis() - startTime < 1000)
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
