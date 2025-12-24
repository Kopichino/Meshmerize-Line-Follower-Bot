#include <Arduino.h>

const int NUM_SENSORS = 8;

// --- HY-S301 Sensor Connections ---
const int IR_ENABLE_PIN = 2;                     // IR emitter control (optional)
const int sensorPins[NUM_SENSORS] = {3,4,5,6,7,8,9,10};

// --- Calibration data (not analog, but we track stability) ---
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(IR_ENABLE_PIN, OUTPUT);
  digitalWrite(IR_ENABLE_PIN, HIGH); // Turn ON emitters

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorMin[i] = 1;  // digital LOW (black)
    sensorMax[i] = 0;  // digital HIGH (white)
  }

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println(F("HY-S301 8-Channel Sensor Calibration"));
  Serial.println(F("Place sensor over white and black surfaces as instructed."));
  delay(2000);

  calibrateSensors();
}

// --- Calibration routine ---
void calibrateSensors() {
  Serial.println(F("Starting calibration... Move the sensor over black and white areas."));

  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 seconds calibration
    digitalWrite(LED_BUILTIN, (millis() / 200) % 2); // Blink LED

    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = digitalRead(sensorPins[i]);

      // Even though HY-S301 is digital, we’ll just observe transitions
      if (val == LOW) sensorMin[i] = 0;   // black detected
      if (val == HIGH) sensorMax[i] = 1;  // white detected
    }
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("Calibration complete!"));
  Serial.println(F("Sensor states (Min/Max):"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(": Min="); Serial.print(sensorMin[i]);
    Serial.print(", Max="); Serial.println(sensorMax[i]);
  }
}

// --- Main Loop: Print sensor readings ---
void loop() {
  Serial.print("S: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = digitalRead(sensorPins[i]);
    Serial.print(val);
    Serial.print(" ");
  }
  Serial.println();
  delay(200);
}
