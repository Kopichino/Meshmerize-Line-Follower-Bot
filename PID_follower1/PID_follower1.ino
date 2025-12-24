#include <EEPROM.h>

// ------------ CONFIG -------------
const int NUM_SENSORS = 5;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};
const int positions[NUM_SENSORS] = {0, 1000, 2000, 3000, 4000}; // weight each sensor

const int LEFT_MOTOR_PWM  = 9;   // Left motor PWM pin
const int RIGHT_MOTOR_PWM = 10;  // Right motor PWM pin

// PID constants – tune for your bot
float Kp = 0.06;  
float Ki = 0.00;  
float Kd = 0.25;  

int baseSpeed = 120;   // LED brightness in sim / motor speed in real bot (0-255)

// EEPROM storage for calibration
int sMin[NUM_SENSORS];
int sMax[NUM_SENSORS];

// PID vars
float integral = 0;
float prevError = 0;
unsigned long prevTime = 0;

int lastPosition = 2000; // assume center at start
// ---------------------------------

// ------------ FUNCTIONS -----------
void setMotors(int left, int right) {
  left  = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_MOTOR_PWM, left);
  analogWrite(RIGHT_MOTOR_PWM, right);
}

bool loadCalibrationFromEEPROM() {
  int addr = 0;
  int tMin[NUM_SENSORS], tMax[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) { EEPROM.get(addr, tMin[i]); addr += sizeof(int); }
  for (int i = 0; i < NUM_SENSORS; i++) { EEPROM.get(addr, tMax[i]); addr += sizeof(int); }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (tMax[i] <= tMin[i]) return false;
  }
  for (int i = 0; i < NUM_SENSORS; i++) { sMin[i] = tMin[i]; sMax[i] = tMax[i]; }
  return true;
}

int readLinePosition() {
  long numerator = 0, denominator = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int range = sMax[i] - sMin[i];
    int val = (range > 0) ? (raw - sMin[i]) * 1000 / range : 0;
    if (val < 0) val = 0;
    if (val > 1000) val = 1000;

    numerator += (long)val * positions[i];
    denominator += val;
  }

  if (denominator == 0) return lastPosition; // line lost
  lastPosition = numerator / denominator;
  return lastPosition;
}
// ---------------------------------

void setup() {
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  Serial.begin(115200);

  if (!loadCalibrationFromEEPROM()) {
    Serial.println("Calibration not found. Please run calibration first.");
    while (1); // halt
  }
  prevTime = millis();
}

void loop() {
  // ---- Get line position ----
  int pos = readLinePosition(); // 0..4000
  float error = (float)pos - 2000.0; // center is 2000

  // ---- PID Calculation ----
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0; // in seconds
  prevTime = now;

  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  // ---- Motor Speed Adjust ----
  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;
  setMotors(leftSpeed, rightSpeed);

  // Debug output
  Serial.print("Pos: "); Serial.print(pos);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);

  prevError = error;
  delay(10); // loop ~100Hz
}
