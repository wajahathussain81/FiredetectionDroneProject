/**************************************************************
   ESP32 Quadcopter Example with iBus Failsafe + MPU6050 + PID
   ------------------------------------------------------------
   1) Reads iBus (FS-iA10B) for roll/pitch/yaw/throttle/armSwitch
   2) Reads MPU6050 via I2C for orientation
   3) Basic PID on roll, pitch, yaw
   4) Drives 4 ESCs on pins: FL=19, FR=23, RL=18, RR=5
   5) Failsafe: if kill switch is low or iBus lost → motors = min
***************************************************************/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// -------- iBus Library (Example: IBusBM) --------
#include <IBusBM.h>  
IBusBM iBus;  // Create iBus object

// -------- MPU6050 Setup --------
Adafruit_MPU6050 mpu;

// -------- Pins for ESCs --------
#define ESC_FL 19  // Front Left
#define ESC_FR 23  // Front Right
#define ESC_RL 18  // Rear Left
#define ESC_RR 5   // Rear Right

// Create Servo (ESC) objects
Servo escFL, escFR, escRL, escRR;

// -------- PID Gains (Tune these!) --------
float Kp_roll  = 2.5;
float Ki_roll  = 0.02;
float Kd_roll  = 1.3;

float Kp_pitch = 2.5;
float Ki_pitch = 0.02;
float Kd_pitch = 1.3;

float Kp_yaw   = 2.0;
float Ki_yaw   = 0.0;
float Kd_yaw   = 1.0;

// -------- PID State Variables --------
float errorRoll = 0, lastErrorRoll = 0, integralRoll = 0;
float errorPitch = 0, lastErrorPitch = 0, integralPitch = 0;
float errorYaw = 0, lastErrorYaw = 0, integralYaw = 0;

unsigned long prevPIDTime = 0;

// -------- Angle Estimates --------
float pitchEst = 0.0;
float rollEst  = 0.0;
float yawEst   = 0.0;  // For a basic example, yaw might be just gyro integration

// Complementary filter factor (for roll/pitch)
float alpha = 0.98;

// -------- iBus Channels (typical mapping) --------
int chRoll     = 0;  // Aileron
int chPitch    = 1;  // Elevator
int chThrottle = 2;
int chYaw      = 3;  // Rudder
int chAux1     = 6;  // Suppose Aux1 is kill switch

#define FAILSAFE_VALUE 1000

// Channel values read from iBus (1000 - 2000 typically)
uint16_t rollIn     = 1500;
uint16_t pitchIn    = 1500;
uint16_t throttleIn = 1000;
uint16_t yawIn      = 1500;
uint16_t aux1In     = 1000;

// -------- Motor outputs (1000 - 2000us) --------
int motorFL, motorFR, motorRL, motorRR;

// -------- Desired angles (from RC) --------
float targetRoll  = 0.0;
float targetPitch = 0.0;
float targetYaw   = 0.0;  // Usually 0 or from yaw RC

// Min/Max Pulse for ESCs
int MIN_PULSE = 1000;
int MAX_PULSE = 2000;

// Kill switch logic: if < 1500 => disarmed
bool armed = false;

// -------- Function Declarations --------
float computePID(float setpoint, float current,
                 float &integral, float &lastError,
                 float kp, float ki, float kd, float dt);

// ---------------------------------------
//                SETUP
// ---------------------------------------
void setup() {
  Serial.begin(115200);

  // 1) Initialize I2C for MPU6050
  Wire.begin(21, 22); // SDA=21, SCL=22 on ESP32
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1);
  }
  Serial.println("MPU6050 connected.");

  // Optional config
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 2) Initialize ESC pins
  escFL.attach(ESC_FL, MIN_PULSE, MAX_PULSE);
  escFR.attach(ESC_FR, MIN_PULSE, MAX_PULSE);
  escRL.attach(ESC_RL, MIN_PULSE, MAX_PULSE);
  escRR.attach(ESC_RR, MIN_PULSE, MAX_PULSE);

  // Send minimum pulses to arm ESCs
  escFL.writeMicroseconds(MIN_PULSE);
  escFR.writeMicroseconds(MIN_PULSE);
  escRL.writeMicroseconds(MIN_PULSE);
  escRR.writeMicroseconds(MIN_PULSE);

  delay(3000); // wait for ESCs to initialize

  // 3) Initialize iBus on a hardware serial port
  //    e.g., UART2: RX on GPIO16, TX on GPIO17 (adjust as needed)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  iBus.begin(Serial2);

  prevPIDTime = millis();

  Serial.println("Setup complete. Waiting for iBus signals...");
}

// ---------------------------------------
//                LOOP
// ---------------------------------------
void loop() {
  // 1) Read iBus data
 
    rollIn     = iBus.readChannel(chRoll);
    pitchIn    = iBus.readChannel(chPitch);
    throttleIn = iBus.readChannel(chThrottle);
    yawIn      = iBus.readChannel(chYaw);
    aux1In     = iBus.readChannel(chAux1);

  // 2) Check kill switch / failsafe
  //    Suppose if Aux1 < 1500 => disarm
  armed = (aux1In > 1500);

  // 3) Map RC channels to target angles
  //    Example: rollIn=1000..2000 => targetRoll=-20..+20 deg
  //    pitchIn=1000..2000 => targetPitch=-20..+20 deg
  //    throttleIn=1000..2000 => motor base speed
  //    yawIn=1000..2000 => targetYaw (or direct yaw rate)
  targetRoll  = mapFloat(rollIn, 1000, 2000, -20.0, 20.0);
  targetPitch = mapFloat(pitchIn, 1000, 2000, -20.0, 20.0);
  // For yaw, you might treat it as a rate or angle. Let's keep it simple:
  targetYaw   = 0.0;  // or map it similarly if you want yaw angle control

  // 4) Read MPU6050 for actual orientation
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Current time
  unsigned long now = millis();
  float dt = (now - prevPIDTime) / 1000.0;
  prevPIDTime = now;

  // Convert gyro (rad/s) to deg/s
  float gyroXdeg = gyro.gyro.x * (180.0 / M_PI); // rotation around X
  float gyroYdeg = gyro.gyro.y * (180.0 / M_PI); // rotation around Y
  float gyroZdeg = gyro.gyro.z * (180.0 / M_PI); // rotation around Z

  // Accelerometer-based angles
  // pitch ~ atan2(accelX, sqrt(Y^2 + Z^2)) in degrees
  // roll  ~ atan2(accelY, sqrt(X^2 + Z^2)) in degrees
  float accPitch = atan2(accel.acceleration.x,
                         sqrt(accel.acceleration.y * accel.acceleration.y +
                              accel.acceleration.z * accel.acceleration.z)) * (180.0 / M_PI);

  float accRoll  = atan2(accel.acceleration.y,
                         sqrt(accel.acceleration.x * accel.acceleration.x +
                              accel.acceleration.z * accel.acceleration.z)) * (180.0 / M_PI);

  // Complementary filter for pitch
  float pitchGyro = pitchEst + gyroXdeg * dt; // X gyro affects pitch
  pitchEst = alpha * pitchGyro + (1 - alpha) * accPitch;

  // Complementary filter for roll
  float rollGyro = rollEst - gyroYdeg * dt;   // Y gyro often inversely affects roll sign
  rollEst = alpha * rollGyro + (1 - alpha) * accRoll;

  // Yaw estimate (very rough, typically integrated from gyroZ)
  yawEst += gyroZdeg * dt;

  // 5) Compute PID for roll, pitch, yaw
  float rollPID  = computePID(targetRoll, rollEst,
                              integralRoll, lastErrorRoll,
                              Kp_roll, Ki_roll, Kd_roll, dt);

  float pitchPID = computePID(targetPitch, pitchEst,
                              integralPitch, lastErrorPitch,
                              Kp_pitch, Ki_pitch, Kd_pitch, dt);

  float yawPID   = computePID(targetYaw, yawEst,
                              integralYaw, lastErrorYaw,
                              Kp_yaw, Ki_yaw, Kd_yaw, dt);

  // 6) Calculate base throttle from throttleIn (1000-2000 mapped to 1000-2000)
  //    or apply a smaller range if needed
  int baseThrottle = throttleIn; // direct pass for example

  // 7) Mix PID outputs into motors
  //    Typical quad X mixing:
  //    Front Left  = throttle + pitchPID - rollPID + yawPID
  //    Front Right = throttle + pitchPID + rollPID - yawPID
  //    Rear Left   = throttle - pitchPID - rollPID - yawPID
  //    Rear Right  = throttle - pitchPID + rollPID + yawPID
  //
  //    Adjust signs as needed for your motor layout/direction
  motorFL = baseThrottle + (int)pitchPID - (int)rollPID + (int)yawPID;
  motorFR = baseThrottle + (int)pitchPID + (int)rollPID - (int)yawPID;
  motorRL = baseThrottle - (int)pitchPID - (int)rollPID - (int)yawPID;
  motorRR = baseThrottle - (int)pitchPID + (int)rollPID + (int)yawPID;

  // 8) Constrain motor outputs to valid PWM range
  motorFL = constrain(motorFL, MIN_PULSE, MAX_PULSE);
  motorFR = constrain(motorFR, MIN_PULSE, MAX_PULSE);
  motorRL = constrain(motorRL, MIN_PULSE, MAX_PULSE);
  motorRR = constrain(motorRR, MIN_PULSE, MAX_PULSE);

  // 9) Failsafe: if not armed, set all motors to MIN
  if (!armed) {
    motorFL = MIN_PULSE;
    motorFR = MIN_PULSE;
    motorRL = MIN_PULSE;
    motorRR = MIN_PULSE;
  }

  // 10) Write to ESCs
  escFL.writeMicroseconds(motorFL);
  escFR.writeMicroseconds(motorFR);
  escRL.writeMicroseconds(motorRL);
  escRR.writeMicroseconds(motorRR);

  // Debug prints
  Serial.print("Armed: "); Serial.print(armed ? "YES" : "NO");
  Serial.print(" | Roll: ");  Serial.print(rollEst);
  Serial.print(" | Pitch: "); Serial.print(pitchEst);
  Serial.print(" | Yaw: ");   Serial.print(yawEst);
  Serial.print(" | FL: ");    Serial.print(motorFL);
  Serial.print(" | FR: ");    Serial.print(motorFR);
  Serial.print(" | RL: ");    Serial.print(motorRL);
  Serial.print(" | RR: ");    Serial.println(motorRR);

  delay(10); // 100 Hz loop
}

// --------------------------------------------------
//  PID Compute Function
// --------------------------------------------------
float computePID(float setpoint, float current,
                 float &integral, float &lastError,
                 float kp, float ki, float kd, float dt)
{
  float error = setpoint - current;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = (kp * error) + (ki * integral) + (kd * derivative);
  lastError = error;
  return output;
}

// --------------------------------------------------
//  Map float range helper (similar to Arduino map)
// --------------------------------------------------
float mapFloat(float x, float in_min, float in_max,
               float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min)
         / (in_max - in_min) + out_min;
}



