/* autopilot_arduino.ino
   Minimal DIY autopilot:
   - ESC control using Servo library (pwm microseconds)
   - MPU6050 IMU raw read (I2C)
   - Complementary filter for roll/pitch estimation
   - PID loops for roll & pitch stabilization
   - Serial command interface with simple protocol:
       ARM
       DISARM
       SET ROLL PITCH YAW THR   (values in degrees for ROLL/PITCH/YAW and 1000-2000 for THR)
   Safety: output is blocked until ARM is received.
*/

#include <Wire.h>
#include <Servo.h>

// ===== Replace pins as per your wiring =====
const int ESC_FR = 3;  // Front-Right motor (PWM pin)
const int ESC_FL = 5;  // Front-Left
const int ESC_BR = 6;  // Back-Right
const int ESC_BL = 9;  // Back-Left

Servo escFR, escFL, escBR, escBL;

// ===== MPU6050 registers (we're using minimal raw reads) =====
const int MPU_ADDR = 0x68;
long gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;

// ===== Complementary filter / AHRS =====
float dt = 0.01;             // loop period (s) -- adjust with accurate timing
unsigned long lastLoopMicros = 0;
float roll = 0.0, pitch = 0.0; // estimated angles (deg)
float alpha = 0.98;          // complementary filter factor

// ===== PID controllers for roll & pitch =====
struct PID {
  float Kp, Ki, Kd;
  float integrator;
  float prevErr;
  float outLimit;
  PID(float p=0.7, float i=0.01, float d=0.02) {
    Kp=p; Ki=i; Kd=d; integrator=0; prevErr=0; outLimit=200;
  }
  float update(float setpoint, float measure, float dt) {
    float err = setpoint - measure;
    integrator += err * dt;
    float deriv = (err - prevErr) / dt;
    prevErr = err;
    float out = Kp*err + Ki*integrator + Kd*deriv;
    if (out > outLimit) out = outLimit;
    if (out < -outLimit) out = -outLimit;
    return out;
  }
};

PID pidRoll(3.0, 0.01, 0.12);   // initial gains - tune later
PID pidPitch(3.0, 0.01, 0.12);

// ===== Higher level setpoints sent from companion =====
float setRoll = 0.0;
float setPitch = 0.0;
float setYaw = 0.0;     // not used in this simple mixer
int setThrottle = 1000; // microseconds (1000-2000 typical)
bool isArmed = false;

// ===== Utility mapping =====
int constrainMicro(int v) {
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return v;
}

// ===== I2C MPU6050 helpers (minimal) =====
void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuReadBytes(byte reg, int len, byte *buf) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, len);
  int i = 0;
  while (Wire.available()) {
    buf[i++] = Wire.read();
  }
}

void calibrateGyro() {
  const int samples = 200;
  long gx=0, gy=0, gz=0;
  for (int i=0; i<samples; i++) {
    byte buf[14];
    mpuReadBytes(0x3B, 14, buf);
    int16_t gx_raw = ((int16_t)buf[8] << 8) | buf[9];
    int16_t gy_raw = ((int16_t)buf[10] << 8) | buf[11];
    int16_t gz_raw = ((int16_t)buf[12] << 8) | buf[13];
    gx += gx_raw; gy += gy_raw; gz += gz_raw;
    delay(5);
  }
  gyroXoffset = gx / samples;
  gyroYoffset = gy / samples;
  gyroZoffset = gz / samples;
  Serial.print("Gyro offsets: "); Serial.print(gyroXoffset); Serial.print(", "); Serial.println(gyroYoffset);
}

void setupMPU() {
  Wire.begin();
  mpuWrite(0x6B, 0x00); // Wake up
  mpuWrite(0x1B, 0x08); // Gyro range ±500°/s (0x08)
  mpuWrite(0x1C, 0x10); // Accel range ±8g (0x10)
  delay(100);
  calibrateGyro();
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Autopilot Arduino booting...");
  escFR.attach(ESC_FR);
  escFL.attach(ESC_FL);
  escBR.attach(ESC_BR);
  escBL.attach(ESC_BL);

  // send low throttle to ESCs to initialize (safe)
  escFR.writeMicroseconds(1000);
  escFL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);

  setupMPU();

  lastLoopMicros = micros();
}

// ===== Read raw MPU6050 and compute angles (deg) =====
void readIMU(float &roll_out, float &pitch_out, float &gx_deg_s, float &gy_deg_s) {
  byte buf[14];
  mpuReadBytes(0x3B, 14, buf);
  int16_t ax = ((int16_t)buf[0] << 8) | buf[1];
  int16_t ay = ((int16_t)buf[2] << 8) | buf[3];
  int16_t az = ((int16_t)buf[4] << 8) | buf[5];
  int16_t gx_raw = ((int16_t)buf[8] << 8) | buf[9];
  int16_t gy_raw = ((int16_t)buf[10] << 8) | buf[11];
  int16_t gz_raw = ((int16_t)buf[12] << 8) | buf[13];

  // convert raw to physical units:
  // accel scale: for ±8g, sensitivity ~4096 LSB/g (varies), we only need angles
  float axg = (float)ax / 4096.0;
  float ayg = (float)ay / 4096.0;
  float azg = (float)az / 4096.0;

  // compute accel angles (deg)
  float roll_acc = atan2(ayg, azg) * 57.29578;   // deg
  float pitch_acc = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 57.29578;

  // gyro rates in deg/s: gyro_raw / sensitivity. For ±500 dps sensitivity ~65.5 LSB/(deg/s)
  float gx = ((float)gx_raw - gyroXoffset) / 65.5;
  float gy = ((float)gy_raw - gyroYoffset) / 65.5;
  // integrate gyro to update angle
  unsigned long now = micros();
  float dt_local = (now - lastLoopMicros) / 1e6;
  if (dt_local <= 0 || dt_local > 0.1) dt_local = 0.01;
  lastLoopMicros = now;

  // complementary filter
  roll = alpha * (roll + gx * dt_local) + (1 - alpha) * roll_acc;
  pitch = alpha * (pitch + gy * dt_local) + (1 - alpha) * pitch_acc;

  roll_out = roll;
  pitch_out = pitch;
  gx_deg_s = gx;
  gy_deg_s = gy;
}

// ===== Mixer =====
// For a quad X config, mixing:
void mixAndWrite(int thrust, float rollCorr, float pitchCorr) {
  // rollCorr and pitchCorr are in microsecond offsets (positive means roll right / pitch forward)
  int fr = constrainMicro(thrust - (int)rollCorr - (int)pitchCorr); // front-right
  int fl = constrainMicro(thrust + (int)rollCorr - (int)pitchCorr); // front-left
  int br = constrainMicro(thrust - (int)rollCorr + (int)pitchCorr); // back-right
  int bl = constrainMicro(thrust + (int)rollCorr + (int)pitchCorr); // back-left

  escFR.writeMicroseconds(fr);
  escFL.writeMicroseconds(fl);
  escBR.writeMicroseconds(br);
  escBL.writeMicroseconds(bl);
}

// ===== Serial protocol parser =====
void handleSerialCommand(String cmd) {
  cmd.trim();
  if (cmd == "ARM") {
    isArmed = true;
    Serial.println("ARMED");
    // small safe throttle after arming
    escFR.writeMicroseconds(1100);
    escFL.writeMicroseconds(1100);
    escBR.writeMicroseconds(1100);
    escBL.writeMicroseconds(1100);
    return;
  }
  if (cmd == "DISARM") {
    isArmed = false;
    Serial.println("DISARMED");
    // set to low
    escFR.writeMicroseconds(1000);
    escFL.writeMicroseconds(1000);
    escBR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000);
    return;
  }
  if (cmd.startsWith("SET")) {
    // Example: SET 1.0 -0.5 0 1400
    // SET <roll_deg> <pitch_deg> <yaw_deg> <throttle>
    float r=0, p=0, y=0; int thr=1000;
    int idx = 0;
    // parse
    char buf[64];
    cmd.toCharArray(buf, 64);
    sscanf(buf, "SET %f %f %f %d", &r, &p, &y, &thr);
    setRoll = r;
    setPitch = p;
    setYaw = y;
    setThrottle = thr;
    Serial.print("SETPOINTS: ");
    Serial.print(setRoll); Serial.print(","); Serial.print(setPitch); Serial.print(","); Serial.println(setThrottle);
    return;
  }
  // unknown
  Serial.print("UNKNOWN CMD: "); Serial.println(cmd);
}

// ===== Main loop =====
String serialBuffer = "";

void loop() {
  // read serial input lines
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\\n' || c == '\\r') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  // read IMU and compute angles
  float curRoll=0, curPitch=0, gx=0, gy=0;
  readIMU(curRoll, curPitch, gx, gy);

  // compute PID outputs (units: deg -> convert to microsecond offsets)
  float rollOut = pidRoll.update(setRoll, curRoll, dt);   // in deg-terms scaled
  float pitchOut = pidPitch.update(setPitch, curPitch, dt);

  // convert degree error outputs to microsecond offsets (scale factor)
  // scaling factor chosen so that ~10deg -> 200us (start small, tune later)
  float scale = 20.0; // 1 deg -> 20 microseconds (tune later)
  float rollMicros = rollOut * scale;
  float pitchMicros = pitchOut * scale;

  if (!isArmed) {
    // keep ESCs at safe idle
    escFR.writeMicroseconds(1000);
    escFL.writeMicroseconds(1000);
    escBR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000);
  } else {
    // mix and write outputs
    mixAndWrite(setThrottle, rollMicros, pitchMicros);
  }

  // debug prints every 100ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("ROLL:"); Serial.print(curRoll,2);
    Serial.print(" PITCH:"); Serial.print(curPitch,2);
    Serial.print(" SETR:"); Serial.print(setRoll); Serial.print(" SETP:"); Serial.print(setPitch);
    Serial.print(" THR:"); Serial.println(setThrottle);
  }

  delay((int)(dt*1000)); // keep loop roughly dt seconds
}
