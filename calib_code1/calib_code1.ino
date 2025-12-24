#include <Arduino.h>
#include <EEPROM.h>

const uint8_t NUM_SENSORS = 5;
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

// Calibration
uint16_t sMin[NUM_SENSORS];
uint16_t sMax[NUM_SENSORS];

// Weighted positions
const int32_t positions[NUM_SENSORS] = {0,1000,2000,3000,4000};
int32_t lastPosition = 2000;

// Motor pins
const uint8_t LEFT_PWM  = 9;
const uint8_t RIGHT_PWM = 10;
const uint8_t END_LED   = 13;

// PID
float Kp = 0.06f;
float Ki = 0.00f;
float Kd = 0.25f;

int   baseSpeed = 120;
float integral  = 0.0f;
float prevError = 0.0f;
unsigned long prevMs = 0;

// ---------------------
void setMotors(int left, int right) {
  left  = constrain(left,  0, 255);
  right = constrain(right, 0, 255);
  analogWrite(LEFT_PWM,  left);
  analogWrite(RIGHT_PWM, right);
}

void saveCalibrationToEEPROM() {
  int addr = 0;
  for (int i=0;i<NUM_SENSORS;i++) { EEPROM.put(addr, sMin[i]); addr += 2; }
  for (int i=0;i<NUM_SENSORS;i++) { EEPROM.put(addr, sMax[i]); addr += 2; }
}

bool loadCalibrationFromEEPROM() {
  int addr = 0;
  uint16_t tMin[NUM_SENSORS], tMax[NUM_SENSORS];
  for (int i=0;i<NUM_SENSORS;i++) { EEPROM.get(addr, tMin[i]); addr += 2; }
  for (int i=0;i<NUM_SENSORS;i++) { EEPROM.get(addr, tMax[i]); addr += 2; }
  for (int i=0;i<NUM_SENSORS;i++) {
    if (tMax[i] <= tMin[i] || tMax[i] > 1023) return false;
  }
  memcpy(sMin, tMin, sizeof(sMin));
  memcpy(sMax, tMax, sizeof(sMax));
  return true;
}

void calibrate(unsigned long ms = 5000) {
  for (int i=0;i<NUM_SENSORS;i++) { sMin[i]=1023; sMax[i]=0; }
  unsigned long start=millis();
  while (millis()-start < ms) {
    digitalWrite(END_LED, ((millis()/200)%2)?HIGH:LOW);
    for (int i=0;i<NUM_SENSORS;i++) {
      int v=analogRead(sensorPins[i]);
      if (v<sMin[i]) sMin[i]=v;
      if (v>sMax[i]) sMax[i]=v;
    }
  }
  digitalWrite(END_LED, LOW);
  saveCalibrationToEEPROM();
  Serial.println(F("Calibration done."));
  for (int i=0;i<NUM_SENSORS;i++) {
    Serial.print(i); Serial.print(": ");
    Serial.print(sMin[i]); Serial.print(", "); Serial.println(sMax[i]);
  }
}

int32_t readLinePosition() {
  long num=0, den=0;
  for (int i=0;i<NUM_SENSORS;i++) {
    int raw=analogRead(sensorPins[i]);
    int range=sMax[i]-sMin[i];
    long val=(range<=0)?0:((raw-sMin[i])*1000L/range);
    val=constrain(val,0,1000);
    num+=val*positions[i];
    den+=val;
  }
  if (den==0) return lastPosition;
  lastPosition=num/den;
  return lastPosition;
}

void setup() {
  pinMode(LEFT_PWM,OUTPUT);
  pinMode(RIGHT_PWM,OUTPUT);
  pinMode(END_LED,OUTPUT);
  analogWriteResolution(8);
  analogWriteFrequency(9,20000);
  analogWriteFrequency(10,20000);
  Serial.begin(115200);
  delay(200);
  Serial.println(F("QTRX calibration + PID (Teensy4.1)"));

  if (!loadCalibrationFromEEPROM()) {
    Serial.println(F("No valid calibration. Calibrating 5s..."));
    calibrate(5000);
  } else Serial.println(F("Loaded calibration."));

  prevMs=millis();
}

void loop() {
  int32_t pos=readLinePosition();
  float error=pos-2000.0f;

  unsigned long now=millis();
  float dt=(now-prevMs)/1000.0f; if(dt<=0)dt=0.001f;
  prevMs=now;

  integral+=error*dt;
  integral=constrain(integral,-5000,5000);
  float derivative=(error-prevError)/dt;
  float correction=Kp*error+Ki*integral+Kd*derivative;

  int left=(int)(baseSpeed-correction);
  int right=(int)(baseSpeed+correction);
  setMotors(left,right);

  Serial.print("pos:");Serial.print(pos);
  Serial.print(" err:");Serial.print(error);
  Serial.print(" L:");Serial.print(left);
  Serial.print(" R:");Serial.println(right);

  prevError=error;
  delay(10);
}
