// ==========================================================================
//  ESP32-S3  |  Hoverboard ESC (UART)  |  Quadrature Encoders  |  MPU6050
//  Combines: hoverboard UART protocol, full quadrature encoders,
//            speed ramp, MPU6050 IMU with low-pass filter,
//            500 ms safety timeout, 20 Hz ROS serial publish
// ==========================================================================

#include <Wire.h>
#include <HardwareSerial.h>

// ===================== HOVERBOARD UART =====================================
// UART1 on ESP32-S3
HardwareSerial HoverSerial(1);

#define HOVER_RX        20      // ESP32-S3 RX  ← Hoverboard TX
#define HOVER_TX        19      // ESP32-S3 TX  → Hoverboard RX
#define HOVER_BAUD      115200
#define SERIAL_BAUD     115200  // USB ↔ PC / ROS

#define START_FRAME     0xABCD
#define MAX_PWR         1000    // hoverboard power units  (-1000 … +1000)

// ===================== ENCODERS ============================================
// Left encoder  (quadrature – both channels)
#define ENC_L_A   36
#define ENC_L_B   37

// Right encoder (quadrature – both channels)
#define ENC_R_A   16
#define ENC_R_B   17

volatile long leftPos  = 0;
volatile long rightPos = 0;

// Ticks per full wheel revolution (gear-ratio-corrected, per wheel)
const long  TICKS_PER_REV_L = 875;
const long  TICKS_PER_REV_R = 798;

// Physical dimensions (metres)
const float WHEEL_RADIUS = 0.1f;   // 20 cm diameter → 1cm radius
const float WHEEL_BASE   = 0.60f;    // 60 cm between wheels

// ===================== MPU6050 =============================================
#define MPU6050_ADDR  0x68
#define ACCEL_XOUT_H  0x3B

// Low-pass filter coefficient (0 = ignore new sample, 1 = no filter)
float alpha = 0.3f;
float fax, fay, faz, fgx, fgy, fgz;   // filtered IMU values

// ===================== HOVERBOARD PROTOCOL STRUCTS ========================
struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
};
SerialCommand Command;

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// Hoverboard RX state machine
uint8_t  rxIdx        = 0;
uint16_t bufStartFrame = 0;
uint8_t *rxPtr;
uint8_t  inByte = 0, prevByte = 0;

// ===================== SPEED RAMP ==========================================
int16_t tgtL = 0, tgtR = 0;   // target power  (-MAX_PWR … +MAX_PWR)
int16_t curL = 0, curR = 0;   // current ramped power

const int16_t RAMP_STEP = 20; // max change per 20 ms tick

// ===================== TIMING ==============================================
const unsigned long SEND_INTERVAL_MS  = 20;   // 50 Hz  → hoverboard
const unsigned long PUBLISH_INTERVAL  = 50;   // 20 Hz  → ROS serial
const unsigned long CMD_TIMEOUT       = 500;  // ms before safety stop

unsigned long lastSendMs  = 0;
unsigned long lastPubMs   = 0;
unsigned long lastCmdMs   = 0;

// ==========================================================================
//  HELPERS
// ==========================================================================

static inline int16_t clampPwr(int32_t v) {
  if (v >  MAX_PWR) v =  MAX_PWR;
  if (v < -MAX_PWR) v = -MAX_PWR;
  return (int16_t)v;
}

// Send raw steer/speed to hoverboard over UART
void SendRaw(int16_t uSteer, int16_t uSpeed) {
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

// Convert independent Left/Right wheel power → hoverboard speed+steer
//   speed = (L + R) / 2
//   steer = (R - L) / 2
void SendLR(int16_t pwrL, int16_t pwrR) {
  int32_t speed = ((int32_t)pwrL + (int32_t)pwrR) / 2;
  int32_t steer = ((int32_t)pwrR - (int32_t)pwrL) / 2;
  SendRaw(clampPwr(steer), clampPwr(speed));
}

// ==========================================================================
//  SERIAL COMMAND PARSER  (from ROS / PC)
//  Protocol: "L,R\n"   values in range -250 … +250
//  Values are scaled from ±250 → ±MAX_PWR internally.
// ==========================================================================
void readSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  int comma = cmd.indexOf(',');
  if (comma < 0) return;

  int16_t cmdL = (int16_t)cmd.substring(0, comma).toInt();
  int16_t cmdR = (int16_t)cmd.substring(comma + 1).toInt();

  cmdL = constrain(cmdL, -250, 250);
  cmdR = constrain(cmdR, -250, 250);

  // Scale ±250 → ±MAX_PWR
  tgtL = clampPwr((long)cmdL * MAX_PWR / 250);
  tgtR = clampPwr((long)cmdR * MAX_PWR / 250);

  lastCmdMs = millis();
}

// ==========================================================================
//  HOVERBOARD FEEDBACK RECEIVER  (optional – enable in loop if needed)
// ==========================================================================
void Receive() {
  while (HoverSerial.available()) {
    inByte = HoverSerial.read();
    bufStartFrame = (uint16_t(inByte) << 8) | prevByte;

    if (bufStartFrame == START_FRAME) {
      rxPtr  = (uint8_t*)&NewFeedback;
      *rxPtr++ = prevByte;
      *rxPtr++ = inByte;
      rxIdx  = 2;
    } else if (rxIdx >= 2 && rxIdx < sizeof(SerialFeedback)) {
      *rxPtr++ = inByte;
      rxIdx++;
    }

    if (rxIdx == sizeof(SerialFeedback)) {
      uint16_t cs =
        NewFeedback.start    ^ NewFeedback.cmd1      ^ NewFeedback.cmd2   ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage  ^ NewFeedback.boardTemp   ^ NewFeedback.cmdLed;

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        Feedback = NewFeedback;
      }
      rxIdx = 0;
    }
    prevByte = inByte;
  }
}

// ==========================================================================
//  MPU6050
// ==========================================================================
void writeMPU(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  int16_t axr = Wire.read() << 8 | Wire.read();
  int16_t ayr = Wire.read() << 8 | Wire.read();
  int16_t azr = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();                        // skip temperature bytes
  int16_t gxr = Wire.read() << 8 | Wire.read();
  int16_t gyr = Wire.read() << 8 | Wire.read();
  int16_t gzr = Wire.read() << 8 | Wire.read();

  // Convert raw → SI units
  ax = (axr / 16384.0f) * 9.81f;   // m/s²
  ay = (ayr / 16384.0f) * 9.81f;
  az = (azr / 16384.0f) * 9.81f;
  gx = (gxr / 131.0f) * (PI / 180.0f);  // rad/s
  gy = (gyr / 131.0f) * (PI / 180.0f);
  gz = (gzr / 131.0f) * (PI / 180.0f);

  // First-order low-pass filter
  fax = alpha * ax + (1.0f - alpha) * fax;
  fay = alpha * ay + (1.0f - alpha) * fay;
  faz = alpha * az + (1.0f - alpha) * faz;
  fgx = alpha * gx + (1.0f - alpha) * fgx;
  fgy = alpha * gy + (1.0f - alpha) * fgy;
  fgz = alpha * gz + (1.0f - alpha) * fgz;
}

// ==========================================================================
//  PUBLISH  (20 Hz → ROS bridge)
//  Format:  enc_left,enc_right,ax,ay,az,gx,gy,gz
// ==========================================================================
void publishSensorData() {
  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);

  // FIX: Snapshot encoder counts atomically to avoid race with ISRs
  noInterrupts();
  long e1 = leftPos;
  long e2 = rightPos;
  interrupts();

  Serial.printf("%ld,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                e1, e2, fax, fay, faz, fgx, fgy, fgz);
}

// ==========================================================================
//  ENCODER ISRs  –  full quadrature (both A and B channels)
//
//  FIX: The right-wheel encoder is mechanically mirrored relative to the
//       left (the motor shaft rotates in the opposite direction for forward
//       motion). Negate the right-encoder tick direction so that BOTH
//       encoders count POSITIVE when the robot moves FORWARD.
//       Without this fix the odometry sees d_left > 0 and d_right < 0
//       during straight-line travel, which computes a non-zero d_theta
//       and causes the ghost rotation seen in RViz.
// ==========================================================================

// Left – channel A triggers
void IRAM_ATTR onLeftA() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  leftPos += (A == B) ? 1 : -1;
}

// Left – channel B triggers
void IRAM_ATTR onLeftB() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  leftPos += (A != B) ? 1 : -1;
}

// Right – channel A triggers  (sign NEGATED relative to left)
void IRAM_ATTR onRightA() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  rightPos += (A == B) ? -1 : 1;   // FIX: inverted for mirrored mounting
}

// Right – channel B triggers  (sign NEGATED relative to left)
void IRAM_ATTR onRightB() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  rightPos += (A != B) ? -1 : 1;   // FIX: inverted for mirrored mounting
}

// ==========================================================================
//  SETUP
// ==========================================================================
void setup() {
  // USB serial  (→ PC / ROS)
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);

  // Hoverboard UART
  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX, HOVER_TX);

  // Encoders – full quadrature on both channels
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), onLeftB,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), onRightB, CHANGE);

  leftPos  = 0;
  rightPos = 0;

  // I2C + MPU6050
  Wire.begin();
  writeMPU(0x6B, 0x00);  // wake up
  writeMPU(0x1B, 0x00);  // gyro  ±250 dps
  writeMPU(0x1C, 0x00);  // accel ±2 g
  writeMPU(0x1A, 0x03);  // DLPF  ~44 Hz

  lastCmdMs = millis();

  Serial.println("[ESP32-S3] Hoverboard + Encoders + IMU READY");
  Serial.println("CMD format: L,R  (range -250 to 250)");
}

// ==========================================================================
//  LOOP
// ==========================================================================
void loop() {

  // 1. Parse incoming ROS command
  readSerial();

  // 2. Safety timeout – stop if no command received recently
  if (millis() - lastCmdMs > CMD_TIMEOUT) {
    tgtL = 0;
    tgtR = 0;
  }

  unsigned long now = millis();

  // 3. Send ramped command to hoverboard at 50 Hz
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    // Ramp curL toward tgtL
    int16_t dL = tgtL - curL;
    dL = constrain(dL, -RAMP_STEP, RAMP_STEP);
    curL += dL;

    // Ramp curR toward tgtR
    int16_t dR = tgtR - curR;
    dR = constrain(dR, -RAMP_STEP, RAMP_STEP);
    curR += dR;

    SendLR(curL, curR);
  }

  // 4. Publish encoder + IMU data to ROS at 20 Hz
  if (now - lastPubMs >= PUBLISH_INTERVAL) {
    lastPubMs = now;
    publishSensorData();
  }

  // 5. (Optional) read hoverboard feedback – uncomment to enable
  // Receive();
}
