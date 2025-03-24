#include <Wire.h>
#include <Keyboard.h>
#include <Mouse.h>

// Constants
const int f1 = A0, f2 = A1, f3 = A2, f4 = A3;
const int tapThreshold = 250; // Threshold for recognizing a tap on f3 and f4
const unsigned long tapInterval = 300; // Max time (ms) between taps for double-tap detection
unsigned long lastTapTime = 0; // Time of last tap for double-tap detection
int tapCount = 0; // Counts taps within the interval
const int threshold = 500;


const float alpha = 0.92;  // Complementary filter constant for both axes
const float movementThresholdX = 0.5;
const float movementThresholdY = 3;  // Minimum change to trigger an update, lowered for vertical sensitivity
const float sensitivityX = 1.5;  // Horizontal (left-right) mouse sensitivity
const float sensitivityY = 0.8;  // Vertical (up-down) mouse sensitivity, adjusted for more responsiveness

// State variables
float RateRoll, RatePitch;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0;
float accCalibrationX = 0, accCalibrationY = 0, accCalibrationZ = 0;
float complementaryRoll = 0, complementaryPitch = 0;
float accAngleX, accAngleY;
unsigned long previousTime = 0;

// Calibration parameters
int RateCalibrationNumber = 2000;  // Number of samples for gyroscope calibration

void setup() {
  Serial.begin(57600);
  delay(500);

  Wire.setClock(100000);
  Wire.begin();

  pinMode(f1, INPUT);
  pinMode(f2, INPUT);
  pinMode(f3, INPUT);
  pinMode(f4, INPUT);

  Serial.println("Initializing MPU6050...");
  if (!initializeMPU6050()) {
    Serial.println("MPU6050 Initialization Failed! Check wiring and connection.");
    while (true);  // Halt if MPU6050 fails to initialize
  }

  Mouse.begin();
  Keyboard.begin();

  Serial.println("Starting Calibration");
  calibrateAccelerometer();
  calibrateGyroscope();
  Serial.println("Setup Complete!");
}

void loop() {
  gyroSignals();
  processMouseMovement();
  handleSensorInputs();
}

bool initializeMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up MPU6050
  if (Wire.endTransmission() != 0) {
    return false;  // Failed to communicate with MPU6050
  }
  return true;
}

void calibrateAccelerometer() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // Set accelerometer range to +/- 8g
  Wire.endTransmission();

  accCalibrationX = accCalibrationY = accCalibrationZ = 0;

  for (int i = 0; i < 2000; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // Start with accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6);  // Request 6 bytes for accelerometer (X, Y, Z)

    if (Wire.available() < 6) {
      Serial.println("Failed to read accelerometer data");
      continue;
    }

    int16_t accX = Wire.read() << 8 | Wire.read();
    int16_t accY = Wire.read() << 8 | Wire.read();
    int16_t accZ = Wire.read() << 8 | Wire.read();

    accCalibrationX += accX;
    accCalibrationY += accY;
    accCalibrationZ += accZ;
    delay(1);  // Small delay for stability
  }

  accCalibrationX /= 2000;
  accCalibrationY /= 2000;
  accCalibrationZ = (accCalibrationZ / 2000) - 16384;

  Serial.println("Accelerometer Calibration Complete");
}

void calibrateGyroscope() {
  RateCalibrationRoll = RateCalibrationPitch = 0;

  for (int i = 0; i < RateCalibrationNumber; i++) {
    gyroSignals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
  }

  RateCalibrationRoll /= RateCalibrationNumber;
  RateCalibrationPitch /= RateCalibrationNumber;

  Serial.println("Gyroscope Calibration Complete");
}

void updateDeltaTime() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  complementaryRoll = alpha * (complementaryRoll + (RateRoll - RateCalibrationRoll) * dt) + (1 - alpha) * accAngleX;
  complementaryPitch = alpha * (complementaryPitch + (RatePitch - RateCalibrationPitch) * dt) + (1 - alpha) * accAngleY;
}

void gyroSignals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Access accelerometer and gyroscope data registers
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  unsigned long timeout = millis();
  while (Wire.available() < 14) {
    if (millis() - timeout > 100) {  // Timeout to prevent freezes
      Serial.println("Error: Timeout reading data from MPU6050");
      return;
    }
  }

  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();

  float calibratedAccX = (accX - accCalibrationX) / 16384.0;
  float calibratedAccY = (accY - accCalibrationY) / 16384.0;
  float calibratedAccZ = (accZ - accCalibrationZ) / 16384.0;

  RateRoll = gyroX / 65.6;
  RatePitch = gyroY / 65.6;

  if (sqrt(pow(calibratedAccX, 2) + pow(calibratedAccZ, 2)) != 0) {
    accAngleX = atan(calibratedAccY / sqrt(pow(calibratedAccX, 2) + pow(calibratedAccZ, 2))) * 180 / PI;
  }
  if (sqrt(pow(calibratedAccY, 2) + pow(calibratedAccZ, 2)) != 0) {
    accAngleY = atan(-calibratedAccX / sqrt(pow(calibratedAccY, 2) + pow(calibratedAccZ, 2))) * 180 / PI;
  }
}

void processMouseMovement() {
  if (abs(complementaryRoll) > movementThresholdX || abs(complementaryPitch) > movementThresholdY) {
    int moveX = complementaryRoll * sensitivityX;  // Separate sensitivity for horizontal movement
    int moveY = complementaryPitch * sensitivityY;  // Separate sensitivity for vertical movement

    if (abs(moveX) < 2) moveX = 0;
    if (abs(moveY) < 2) moveY = 0;

    if (moveX != 0 || moveY != 0) {
      Mouse.move(-moveX, -moveY);
      Serial.print("Mouse Move - X: ");
      Serial.print(moveX);
      Serial.print(" Y: ");
      Serial.println(moveY);
    }
  }
}

void handleSensorInputs() {
  int v1 = analogRead(f1);
  int v2 = analogRead(f2);
  int v3 = analogRead(f3);
  int v4 = analogRead(f4);

  Keyboard.releaseAll();

  Movement controls
  if (v1 > threshold && v4 > threshold) keyboardCombo('w', 'd', "Move front-right");
  else if (v1 > threshold && v3 > threshold) keyboardCombo('w', 'a', "Move front-left");
  else if (v2 > threshold && v4 > threshold) keyboardCombo('s', 'd', "Move back-right");
  else if (v2 > threshold && v3 > threshold) keyboardCombo('s', 'a', "Move back-left");

  else if (v1 > threshold) keyboardSingle('w', "Move forward");
  else if (v2 > threshold) keyboardSingle('s', "Move backward");
  else if (v4 > threshold) keyboardSingle('d', "Move right");
  else if (v3 > threshold) keyboardSingle('a', "Move left");

  if (v1 > threshold && v2 > threshold) handleJumpOrFlyMode();


  
}


void keyboardSingle(char key, const char* action) {
  Keyboard.press(key);
  delay(50);  // Short press duration
  Keyboard.release(key);
  Serial.println(action);
}

void keyboardCombo(char key1, char key2, const char* action) {
  Keyboard.press(key1);
  Keyboard.press(key2);
  delay(50);  // Short press duration
  Keyboard.releaseAll();
  Serial.println(action);
}

int getSmoothedValue(int pin) {
  int total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(pin);
  }
  return total / 10;
}
