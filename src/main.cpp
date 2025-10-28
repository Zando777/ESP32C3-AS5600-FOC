#include <Arduino.h>
#include <Wire.h>

#define DIR_PIN     4
#define SDA_PIN     8
#define SCL_PIN     9
#define ANALOG_PIN  3
#define AS5600_ADDR 0x36

// ---------- Function Prototypes ----------
float readAngleI2C();
float readAngleAnalogRaw();
float unwrapAngle(float newAngle);

// ---------- Global Variables ----------
float angleOffset = 0.0;
float prevAngle = 0.0;
float turns = 0.0;

// ---------- Setup ----------
void setup() {
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);  // Set rotation direction

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(200);

  Serial.println("AS5600 I2C + Continuous Analog Angle Output");

  // Stabilize ADC
  for (int i = 0; i < 10; i++) {
    analogRead(ANALOG_PIN);
    delay(10);
  }

  // --- Calibration ---
  float angleI2C = readAngleI2C();
  float angleAnalog = readAngleAnalogRaw();
  angleOffset = angleI2C - angleAnalog;
  if (angleOffset < 0) angleOffset += 360.0;

  prevAngle = angleAnalog + angleOffset;
  if (prevAngle >= 360.0) prevAngle -= 360.0;

  Serial.print("Calibration complete. Offset: ");
  Serial.println(angleOffset, 2);
}

// ---------- Read I2C Angle ----------
float readAngleI2C() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() < 2) return NAN;

  uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
  rawAngle &= 0x0FFF; // 12-bit angle
  return (rawAngle * 360.0) / 4096.0;
}

// ---------- Read Analog Raw Angle ----------
float readAngleAnalogRaw() {
  int rawADC = analogRead(ANALOG_PIN);  // 0–4095 (12-bit)
  return (rawADC * 360.0) / 4095.0;
}

// ---------- Unwrap Continuous Angle ----------
float unwrapAngle(float newAngle) {
  float delta = newAngle - prevAngle;
  if (delta > 180.0)      turns -= 1;   // wrapped backward
  else if (delta < -180.0) turns += 1;  // wrapped forward
  prevAngle = newAngle;
  return newAngle + (turns * 360.0);
}

// ---------- Main Loop ----------
void loop() {
  float angleI2C = readAngleI2C();
  float analogRaw = readAngleAnalogRaw();

  // Apply calibration offset
  float calibratedAngle = analogRaw + angleOffset;
  if (calibratedAngle >= 360.0) calibratedAngle -= 360.0;
  if (calibratedAngle < 0.0) calibratedAngle += 360.0;

  // Continuous unwrapped analog angle
  float continuousAngle = unwrapAngle(calibratedAngle);

  // Output I2C (absolute) + analog (continuous)
  if (!isnan(angleI2C)) {
    Serial.print(angleI2C, 2);
    Serial.print(",");
    Serial.println(continuousAngle, 2);
  } else {
    Serial.println("Error");
  }

  delay(50);
}
