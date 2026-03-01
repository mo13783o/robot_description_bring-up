// ===================== HARDWARE SERIAL FOR ESP32-S3 ======================
#include <HardwareSerial.h>

// نستخدم UART1 على ESP32-S3
HardwareSerial HoverSerial(1);

// Pins المتوصلة بالـ Hoverboard
#define HOVER_RX 20   // ESP32 RX  ← Hoverboard TX
#define HOVER_TX 19   // ESP32 TX  → Hoverboard RX

// ===================== USER CONFIG ======================
#define SERIAL_BAUD 115200     // USB Serial مع الـ PC
#define HOVER_BAUD  115200     // UART مع لوحة الـ Hoverboard

#define START_FRAME 0xABCD     // لازم يطابق الـ firmware
#define MAX_PWR     1000
#define SEND_HZ     50         // 50Hz → كل 20ms
// ========================================================================


// ===================== DATA STRUCTS =====================
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

// ===================== RX STATE (لـ Feedback – ممكن نحتاجه بعدين) ========
uint8_t   rxIdx = 0;
uint16_t  bufStartFrame = 0;
uint8_t  *rxPtr;
uint8_t   inByte = 0, prevByte = 0;

// ===================== TIMERS ==========================
const unsigned long SEND_INTERVAL_MS = 1000UL / SEND_HZ;
unsigned long lastSendMs = 0;
unsigned long lastFBms   = 0;
// ========================================================================


// ===================== TARGETS & RAMP ==================
int16_t tgtR = 0;     // target power للعجلة اليمين
int16_t tgtL = 0;     // target power للعجلة الشمال
int16_t curR = 0;     // القيمة الفعلية اللي بتتبعت (بعد الـ ramp)
int16_t curL = 0;

const int16_t rampStep = 20;   // نعومة الحركة
// ========================================================================


// ===================== EXTERNAL ENCODERS (QUADRATURE) ====================
// Left encoder
#define ENC_L_A 36
#define ENC_L_B 37

// Right encoder
#define ENC_R_A 16
#define ENC_R_B 17

// cumulative counts من بداية التشغيل
volatile long leftPos  = 0;
volatile long rightPos = 0;

// قيم عدد النبضات لكل لفة (من كلامك)
const long  TICKS_PER_REV_L = 875;
const long  TICKS_PER_REV_R = 798;

const float WHEEL_RADIUS = 0.075f;  // 15 cm / 2
const float WHEEL_BASE   = 0.60f;   // 60 cm

// تايمر لطباعة الـ ENC L R
unsigned long lastEncMs = 0;
const unsigned long ENC_PRINT_INTERVAL_MS = 100;   // 100 ms
// ========================================================================


// ===================== HELPERS ==========================
static inline int16_t clampPwr(int32_t v) {
  if (v >  MAX_PWR) v =  MAX_PWR;
  if (v < -MAX_PWR) v = -MAX_PWR;
  return (int16_t)v;
}

void SendRaw(int16_t uSteer, int16_t uSpeed) {
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

/*
   نحول Right/Left wheel power إلى speed & steer:
      speed = (L + R) / 2
      steer = (R - L) / 2
*/
void SEND_R_L(int16_t PWR_R, int16_t PWR_L) {
  int32_t speed = ((int32_t)PWR_L + (int32_t)PWR_R) / 2;
  int32_t steer = ((int32_t)PWR_R - (int32_t)PWR_L) / 2;

  int16_t uSpeed = clampPwr(speed);
  int16_t uSteer = clampPwr(steer);

  SendRaw(uSteer, uSpeed);
}
// ========================================================================


// ===================== PC COMMAND PARSER =======================
void readPcCommand()
{
  if (Serial.available() > 0) {
    int16_t cmdR = (int16_t)Serial.parseInt();
    int16_t cmdL = (int16_t)Serial.parseInt();

    if (Serial.available()) {
      Serial.read();
    }

    cmdR = constrain(cmdR, -250, 250);
    cmdL = constrain(cmdL, -250, 250);

    long scaledR = (long)cmdR * MAX_PWR / 250;
    long scaledL = (long)cmdL * MAX_PWR / 250;

    tgtR = clampPwr(scaledR);
    tgtL = clampPwr(scaledL);

    Serial.print("New targets → R=");
    Serial.print(tgtR);
    Serial.print("  L=");
    Serial.println(tgtL);
  }
}
// ========================================================================


// ===================== RECEIVE FEEDBACK (اختياري) ========================
void Receive() {
  while (HoverSerial.available()) {
    inByte = HoverSerial.read();
    bufStartFrame = (uint16_t(inByte) << 8) | prevByte;

    if (bufStartFrame == START_FRAME) {
      rxPtr = (uint8_t*)&NewFeedback;
      *rxPtr++ = prevByte;
      *rxPtr++ = inByte;
      rxIdx = 2;
    }
    else if (rxIdx >= 2 && rxIdx < sizeof(SerialFeedback)) {
      *rxPtr++ = inByte;
      rxIdx++;
    }

    if (rxIdx == sizeof(SerialFeedback)) {
      uint16_t cs =
        NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

      if (NewFeedback.start == START_FRAME && cs == NewFeedback.checksum) {
        Feedback = NewFeedback;
        lastFBms = millis();
      }
      rxIdx = 0;
    }

    prevByte = inByte;
  }
}
// ========================================================================


// ===================== ENCODER ISRs (نفس منطق سكيتش الاختبار) ===========

// Left encoder
void IRAM_ATTR onLeftA() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);

  if (A == B)
    leftPos++;
  else
    leftPos--;
}

void IRAM_ATTR onLeftB() {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);

  if (A != B)
    leftPos++;
  else
    leftPos--;
}

// Right encoder
void IRAM_ATTR onRightA() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);

  if (A == B)
    rightPos++;
  else
    rightPos--;
}

void IRAM_ATTR onRightB() {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);

  if (A != B)
    rightPos++;
  else
    rightPos--;
}
// ========================================================================


// ===================== SETUP =======================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);

  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX, HOVER_TX);

  // إعداد الإنكودرات
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), onRightB, CHANGE);

  leftPos  = 0;
  rightPos = 0;
  lastEncMs = millis();

  Serial.println("\n[ESP32-S3] Hoverboard + Encoders READY");
  Serial.println("Send: R L  (e.g. 100 100)");
}
// ========================================================================


// ===================== LOOP =======================
void loop() 
{
  readPcCommand();
  //Receive();   // لو حابب تفعّل الريتينج من الهفر بورد

  unsigned long now = millis();

  // 1) إرسال أوامر الهفر بورد كل 20 ms
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    int16_t dR = tgtR - curR;
    int16_t dL = tgtL - curL;

    if (dR >  rampStep) dR =  rampStep;
    if (dR < -rampStep) dR = -rampStep;
    if (dL >  rampStep) dL =  rampStep;
    if (dL < -rampStep) dL = -rampStep;

    curR += dR;
    curL += dL;

    SEND_R_L(curR, curL);

    //Serial.print("CMD → R=");
    //Serial.print(curR);
    //Serial.print("  L=");
    //Serial.println(curL);
  }

  // 2) إرسال قيم الإنكودر كل 100 ms على نفس الـ Serial (لـ ROS)
  if (now - lastEncMs >= ENC_PRINT_INTERVAL_MS) {
    lastEncMs = now;

    noInterrupts();
    long L = leftPos;
    long R = rightPos;
    interrupts();
    Serial.print("ENC ");
    Serial.print(L);
    Serial.print(" ");
    Serial.println(R);
  }
}
// ========================================================================

