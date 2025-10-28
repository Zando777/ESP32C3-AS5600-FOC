#include <Arduino.h>
#include <Wire.h>

#define DIR_PIN     0
#define STEP_PIN    1
#define SDA_PIN     8
#define SCL_PIN     9
#define AS5600_ADDR 0x36

// Stepper configuration
const int stepsPerRev = 200;
const int microsteps = 8;
const int totalSteps = stepsPerRev * microsteps;  // 1600 steps/rev

// Control parameters
const float Kp = 2.0;  // proportional gain, tune experimentally
const float loopDelay = 2; // ms between steps (basic speed control)

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(200);
  Serial.println("Closed-loop stepper control using AS5600 I2C encoder");
}

// Read encoder angle from AS5600 via I2C (0–360°)
float readAngleI2C() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // angle high byte
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() < 2) return NAN;

  uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
  rawAngle &= 0x0FFF; // 12-bit
  return (rawAngle * 360.0) / 4096.0;
}

// Send one step pulse
void stepPulse(bool dir) {
  digitalWrite(DIR_PIN, dir);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(50); // short pulse
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(50);
}

void loop() {
  static float targetAngle = 180.0; // target in degrees
  float currentAngle = readAngleI2C();

  if (isnan(currentAngle)) return;

  // Calculate error
  float error = targetAngle - currentAngle;

  // Wrap error into -180 to 180
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;

  // Determine step count using proportional control
  float stepCommand = Kp * error;  // P controller
  int stepsToMove = (int)((stepCommand / 360.0) * totalSteps);

  bool dir = stepsToMove >= 0;
  stepsToMove = abs(stepsToMove);

  for (int i = 0; i < stepsToMove; i++) {
    stepPulse(dir);
    delay(loopDelay); // spacing between steps
  }

  // Optional: print current and target
  Serial.print("Target: "); Serial.print(targetAngle);
  Serial.print(" | Current: "); Serial.println(currentAngle);

  delay(10); // loop delay
}
