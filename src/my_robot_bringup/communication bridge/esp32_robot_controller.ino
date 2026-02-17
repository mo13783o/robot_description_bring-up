#include <Wire.h>

// ================= MPU6050 =================
#define MPU6050_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

// ================= MOTOR PINS =================
#define MOTOR_LEFT_PWM 25
#define MOTOR_LEFT_DIR 26
#define MOTOR_RIGHT_PWM 27
#define MOTOR_RIGHT_DIR 14

// PWM channels
#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_LEFT 0
#define CH_RIGHT 1

// ================= ENCODERS =================
#define ENCODER_LEFT_A 32
#define ENCODER_LEFT_B 33
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35

volatile long encoder_left = 0;
volatile long encoder_right = 0;

// ================= TIMING =================
unsigned long last_publish = 0;
unsigned long last_cmd_time = 0;

const unsigned long PUBLISH_INTERVAL = 50;   // 20 Hz
const unsigned long CMD_TIMEOUT = 500;       // stop motors if no cmd

// ================= FILTER =================
float alpha = 0.3; // low-pass filter
float fax, fay, faz, fgx, fgy, fgz;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // PWM setup
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_LEFT_PWM, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_RIGHT_PWM, CH_RIGHT);

  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);

  // Encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, CHANGE);

  // I2C
  Wire.begin();

  // Wake MPU6050
  writeMPU(0x6B, 0);
  writeMPU(0x1B, 0x00); // gyro ±250 dps
  writeMPU(0x1C, 0x00); // accel ±2g
  writeMPU(0x1A, 0x03); // DLPF

  Serial.println("ESP32 Ready");
}

// ================= LOOP =================
void loop() {

  readSerial();

  // Safety timeout
  if (millis() - last_cmd_time > CMD_TIMEOUT) {
    setMotorSpeed(0,0);
  }

  if (millis() - last_publish >= PUBLISH_INTERVAL) {
    publishSensorData();
    last_publish = millis();
  }
}

// ================= SERIAL =================
void readSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  int comma = cmd.indexOf(',');
  if (comma < 0) return;

  int L = cmd.substring(0,comma).toInt();
  int R = cmd.substring(comma+1).toInt();

  setMotorSpeed(L,R);
  last_cmd_time = millis();
}

// ================= MOTORS =================
void setMotorSpeed(int L, int R) {
  digitalWrite(MOTOR_LEFT_DIR, L>=0);
  digitalWrite(MOTOR_RIGHT_DIR, R>=0);

  ledcWrite(CH_LEFT, constrain(abs(L),0,255));
  ledcWrite(CH_RIGHT, constrain(abs(R),0,255));
}

// ================= IMU =================
void writeMPU(byte reg, byte data){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readIMU(float &ax,float &ay,float &az,float &gx,float &gy,float &gz){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14,true);

  int16_t axr=Wire.read()<<8|Wire.read();
  int16_t ayr=Wire.read()<<8|Wire.read();
  int16_t azr=Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read();
  int16_t gxr=Wire.read()<<8|Wire.read();
  int16_t gyr=Wire.read()<<8|Wire.read();
  int16_t gzr=Wire.read()<<8|Wire.read();

  ax = (axr/16384.0)*9.81;
  ay = (ayr/16384.0)*9.81;
  az = (azr/16384.0)*9.81;

  gx = (gxr/131.0)*(PI/180.0);
  gy = (gyr/131.0)*(PI/180.0);
  gz = (gzr/131.0)*(PI/180.0);

  // Low-pass filter
  fax = alpha*ax + (1-alpha)*fax;
  fay = alpha*ay + (1-alpha)*fay;
  faz = alpha*az + (1-alpha)*faz;
  fgx = alpha*gx + (1-alpha)*fgx;
  fgy = alpha*gy + (1-alpha)*fgy;
  fgz = alpha*gz + (1-alpha)*fgz;
}

// ================= PUBLISH =================
void publishSensorData(){
  float ax,ay,az,gx,gy,gz;
  readIMU(ax,ay,az,gx,gy,gz);

  noInterrupts();
  long e1=encoder_left;
  long e2=encoder_right;
  interrupts();

  Serial.printf("%ld,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                e1,e2,fax,fay,faz,fgx,fgy,fgz);
}

// ================= ENCODERS =================
void IRAM_ATTR encoderLeftISR(){
  encoder_left += (digitalRead(ENCODER_LEFT_A)==digitalRead(ENCODER_LEFT_B))?1:-1;
}

void IRAM_ATTR encoderRightISR(){
  encoder_right += (digitalRead(ENCODER_RIGHT_A)==digitalRead(ENCODER_RIGHT_B))?1:-1;
}
