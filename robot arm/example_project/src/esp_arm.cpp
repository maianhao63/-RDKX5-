#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define MAIX_RX_PIN 14   // 更换为安全引脚，避开BOOT引脚13
HardwareSerial maixSerial(2);
Adafruit_PWMServoDriver pwm(0x40);

#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST       3
#define WRIST_ROT   4
#define CLAW        5

const float L1 = 4.0f;
const float L2 = 10.0f;
const float L3 = 10.0f;
const float L4 = 4.0f;
const float L5 = 6.0f;
const float TOTAL_FRONT = L4 + L5;

const float TARGET_Z = 0.0f;
#define DEG(rad) ((rad) * 180.0f / M_PI)

// 误差参数
#define X_ERROR_MAX  0.3f
#define Y_ERROR_MAX  0.3f
#define STABLE_TIME  500

bool canReceive = false;
int grabCount = 0;
float lastX = 0, lastY = 0;
unsigned long detectTime = 0;
bool isDetecting = false;

float j_base, j_shoulder, j_elbow, j_wrist;

int angle2pwm(int a) {
  a = constrain(a, -30, 200);
  return map(a, -30, 200, 150, 600);
}

void setServo(int ch, int ang) {
  pwm.setPWM(ch, 0, angle2pwm(ang));
}

bool ik(float X, float Y, float Z) {
  j_base = atan2(X, Y);
  float r = sqrt(X*X + Y*Y);
  float z_abs = Z - L1;

  float r_target = r - TOTAL_FRONT;
  float z_target = z_abs;
  float D = sqrt(r_target*r_target + z_target*z_target);

  if (D < 2.0f || D > 28.0f) return false;

  float cos_e = (L2*L2 + L3*L3 - D*D) / (2*L2*L3);
  cos_e = constrain(cos_e, -0.98f, 0.98f);
  j_elbow = acos(cos_e);

  float alpha = atan2(z_target, r_target);
  float beta = acos((L2*L2 + D*D - L3*L3) / (2*L2*D));
  j_shoulder = alpha - beta;

  j_wrist = (M_PI / 2.0f) - j_shoulder - j_elbow;

  j_base = DEG(j_base);
  j_shoulder = DEG(j_shoulder);
  j_elbow = DEG(j_elbow);
  j_wrist = DEG(j_wrist);

  return true;
}

void goXYZ(float X, float Y, float Z) {
  bool ok = ik(X, Y, Z);
  if (!ok) return;

  int base     = 98+ j_base;
  int shoulder = 60 + j_shoulder;
  int elbow    = 95 - j_elbow;
  int wrist    = 160 - j_wrist;
  int wrot     = 90;

  setServo(BASE, base);          
  setServo(SHOULDER, shoulder);  
  setServo(ELBOW, elbow);        
  setServo(WRIST, wrist);       
  setServo(WRIST_ROT, wrot);     
}

void homeArmOnly() {
  setServo(BASE,      65);   delay(500);
  setServo(WRIST_ROT, 90);delay(500);
  setServo(WRIST,      180);   delay(500);
  setServo(ELBOW,    135);   delay(500);
  setServo(SHOULDER, 80);     delay(2000);
  setServo(CLAW,     -30);
  Serial.println("✅ 手臂回到常规HOME");
  delay(500);
    setServo(WRIST_ROT, 90);   delay(500);
    setServo(WRIST,    10);    delay(500);
  setServo(CLAW,     20);   delay(200); 

  
  
  setServo(ELBOW,    -45);   delay(500);
  setServo(SHOULDER, 100);   delay(500);
  setServo(BASE,      90);   delay(500);
  setServo(CLAW,     -50);   delay(200);
}

void greenPose() {
  setServo(BASE,      115);   delay(500);
  setServo(WRIST_ROT, 90);delay(500);
  setServo(WRIST,      180);   delay(500);
  setServo(ELBOW,    135);   delay(500);
  setServo(SHOULDER, 80);     delay(2000);
  setServo(CLAW,     -30);
  Serial.println("绿色目标：转到指定姿态");
  delay(500);
  setServo(WRIST_ROT, 90);   delay(500);
    setServo(ELBOW,    -45);   delay(500);
  setServo(CLAW,     20);   delay(200);
  
 

  setServo(WRIST,     10);    delay(500);
  setServo(SHOULDER, 100);   delay(500);
  setServo(BASE,      90);   delay(500);
   setServo(CLAW,     -50);   delay(200);
}

void homeAll() {
  setServo(BASE,      90);   delay(200);
  setServo(SHOULDER, 100);   delay(200);
  setServo(ELBOW,    -20);   delay(200);
  setServo(WRIST,     10);   delay(200);
  setServo(WRIST_ROT, 90);   delay(200);
  setServo(CLAW,     -50);   delay(200);
  Serial.println("✅ 上电完成，夹爪张开");
}

void grabOnce(float x, float y, char color) {
  Serial.println("🚀 运动到目标位置...");
  goXYZ(x, y, TARGET_Z);
  
  Serial.println("⏳ 等待 1 秒...");
  delay(1000);

  Serial.println("🔴 夹爪闭合");
  setServo(CLAW, 60);
  delay(800);

  if(color == 'R')
  {
    homeArmOnly();
  }
  else if(color == 'G')
  {
    greenPose();
  }

  delay(1000);
  grabCount++;
  Serial.print("📌 当前已完成抓取次数：");
  Serial.println(grabCount);
}

void receiveMaixData() {

  if (!canReceive) return;
  if (!maixSerial.available()) return;

  while(maixSerial.available() > 128)
  {
    maixSerial.read();
  }

  String data = maixSerial.readStringUntil('\n');
  data.trim();

  if(data.length() < 6 || data.length() > 64) return;

  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  if(firstComma == -1 || secondComma == -1) return;

  float x = data.substring(0, firstComma).toFloat();
  float y = data.substring(firstComma+1, secondComma).toFloat();
  char color = data.charAt(secondComma + 1);

  Serial.print("📥 收到: X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" 颜色=");
  Serial.println(color);

  if (!isDetecting) {
    lastX = x;
    lastY = y;
    detectTime = millis();
    isDetecting = true;
    Serial.println("⏳ 检测稳定性...");
  } else {
    if (millis() - detectTime >= STABLE_TIME) {
      float dx = abs(x - lastX);
      float dy = abs(y - lastY);

      Serial.print("📏 误差 X=");
      Serial.print(dx);
      Serial.print(" Y=");
      Serial.println(dy);

      if (dx <= X_ERROR_MAX && dy <= Y_ERROR_MAX) {
        Serial.println("✅ 坐标稳定，执行抓取");
        grabOnce(x, y, color);
      } else {
        Serial.println("❌ 坐标不稳定，跳过");
      }
      isDetecting = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  maixSerial.begin(115200, SERIAL_8N1, MAIX_RX_PIN, -1);
  
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(1000);

  while(maixSerial.available())
  {
    maixSerial.read();
  }

  homeAll();
  Serial.println("⏳ 等待 1 秒后开始接收坐标...");
  delay(2000);
  canReceive = true;
  grabCount = 0;
  Serial.println("========================================");
  Serial.println("🎯 无限抓取模式，持续等待目标");
  Serial.println("========================================");
}

void loop() {
  receiveMaixData();
}
