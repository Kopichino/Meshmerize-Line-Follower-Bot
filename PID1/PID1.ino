#include <Arduino.h>

// ==================== QTRX Sensor Setup ====================
const int sensorPins[] = {A1, A3, A4, A6}; // S2, S4, S5, S7 only
const int NUM_SENSORS = sizeof(sensorPins) / sizeof(sensorPins[0]);
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorValues[NUM_SENSORS];

// ==================== Motor Pins ============================
const int M1_IN1 = 5;
const int M1_IN2 = 6;
const int M2_IN1 = 9;
const int M2_IN2 = 10;
const int M3_IN1 = 3;
const int M3_IN2 = 4;
const int M4_IN1 = 7;
const int M4_IN2 = 8;

// ==================== PID Parameters =======================
float Kp = 0.25;
float Ki = 0.0;
float Kd = 0.12;
float lastError = 0;
float integral = 0;

// ==================== Motor Speed ==========================
const int baseSpeed = 200;  // Base speed of motors (0-255)

// ==================== Setup ================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  int motorPins[] = {M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2};
  for (int i = 0; i < 8; i++) pinMode(motorPins[i], OUTPUT);

  // Initialize sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.println("QTRX-MD08RC Calibration Starting - Move bot over black & white areas");
  calibrateSensors();
}

// ==================== Sensor Calibration ===================
void calibrateSensors() {
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 seconds calibration
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val; // white
      if (val > sensorMax[i]) sensorMax[i] = val; // black
    }
    delay(10);
  }

  Serial.println("Calibration Complete!");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S"); Serial.print(i+1);
    Serial.print(" Min:"); Serial.print(sensorMin[i]);
    Serial.print(" Max:"); Serial.println(sensorMax[i]);
  }
}

// ==================== Read Sensors ========================
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int norm = map(raw, sensorMin[i], sensorMax[i], 0, 1000);
    sensorValues[i] = constrain(norm, 0, 1000);
  }
}

// ==================== Line Position Calculation ===========
float calculateError() {
  // Weighted average method
  float num = 0;
  float denom = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    num += sensorValues[i] * (i * 1000); // weights: 0, 1000, 2000, 3000
    denom += sensorValues[i];
  }
  if (denom == 0) return 0; // all white line
  return num / denom - 1500; // center is 1500
}

// ==================== Motor Control ========================
void setMotorSpeed(int pinA, int pinB, int speed) {
  if (speed > 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

// Set speed for all 4 motors with correction
void driveMotors(int correction) {
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Left motors: M1, M3
  setMotorSpeed(M1_IN1, M1_IN2, leftSpeed);
  setMotorSpeed(M3_IN1, M3_IN2, leftSpeed);

  // Right motors: M2, M4
  setMotorSpeed(M2_IN1, M2_IN2, rightSpeed);
  setMotorSpeed(M4_IN1, M4_IN2, rightSpeed);
}

// Stop all motors
void stopAllMotors() {
  setMotorSpeed(M1_IN1, M1_IN2, 0);
  setMotorSpeed(M2_IN1, M2_IN2, 0);
  setMotorSpeed(M3_IN1, M3_IN2, 0);
  setMotorSpeed(M4_IN1, M4_IN2, 0);
}

// ==================== Junction Detection ==================
bool checkJunction() {
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > 700) count++; // black threshold
  }
  return (count >= 3); // junction detected if 3+ sensors detect black
}

// ==================== Main Loop ===========================
void loop() {
  readSensors();

  // Stop if all sensors are white
  bool allWhite = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > 50) { // threshold for white
      allWhite = false;
      break;
    }
  }
  if (allWhite) {
    stopAllMotors();
    Serial.println("All White - Stopping Bot");
    while (1);
  }

  // PID control
  float error = calculateError();
  integral += error;
  float derivative = error - lastError;
  float correction = Kp*error + Ki*integral + Kd*derivative;
  lastError = error;

  driveMotors(correction);

  // Junction detection
  if (checkJunction()) {
    Serial.println("Junction Detected!");
    delay(200); // small pause for junction
  }

  // Serial print for debugging
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorValues[i]), Serial.print(" ");
  Serial.print(" | Correction: "); Serial.println(correction);

  delay(10);
}