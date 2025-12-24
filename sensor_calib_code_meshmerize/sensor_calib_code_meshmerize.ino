#include <Arduino.h>

const int NUM_SENSORS = 8;

// --- Updated Pin Connections ---
const int sensorPins[NUM_SENSORS] = {A0,A1,A2,A3,A4,A5,A6,A7};
const int IR_ENABLE_PIN = 10; // optional, if using emitter control

// --- Calibration data ---
int sensorMin[NUM_SENSORS];   // white reference
int sensorMax[NUM_SENSORS];   // black reference

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IR_ENABLE_PIN, OUTPUT);
  digitalWrite(IR_ENABLE_PIN, HIGH); // Turn ON emitters

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023; // start assuming max analog = white
    sensorMax[i] = 0;    // start assuming min analog = black
  }

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println(F("QTRX-MD08A 8-Channel Sensor Calibration"));
  Serial.println(F("Place sensors over white and black surfaces as instructed."));
  delay(2000);

  calibrateSensors();
}

// --- Calibration Routine ---
void calibrateSensors() {
  Serial.println(F("Starting calibration... Move sensors over black and white areas."));

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 seconds calibration
    digitalWrite(LED_BUILTIN, (millis() / 200) % 2); // Blink LED

    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPins[i]);

      if (val < sensorMin[i]) sensorMin[i] = val; // track white
      if (val > sensorMax[i]) sensorMax[i] = val; // track black
    }
    delay(10);
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("Calibration complete!"));
  Serial.println(F("Sensor states (Min=white / Max=black):"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S"); Serial.print(i+1);
    Serial.print(": Min="); Serial.print(sensorMin[i]);
    Serial.print(", Max="); Serial.println(sensorMax[i]);
  }
}

// --- Main Loop: Print Normalized Sensor Readings 0-1000 ---
void loop() {
  Serial.print("S: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int norm = map(raw, sensorMin[i], sensorMax[i], 0, 1000); // normalize
    norm = constrain(norm, 0, 1000); // ensure within 0-1000
    Serial.print(norm);
    Serial.print(" ");
  }
  Serial.println();
  delay(200);
}