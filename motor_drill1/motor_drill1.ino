#include <Arduino.h>

// Motor pins
const int M1_IN1 = 5;  // Motor 1
const int M1_IN2 = 6;
const int M2_IN1 = 9;  // Motor 2
const int M2_IN2 = 10;
const int M3_IN1 = 3;  // Motor 3 (second DRV8833)
const int M3_IN2 = 4;
const int M4_IN1 = 7;  // Motor 4
const int M4_IN2 = 8;

// PWM speed settings
const int minSpeed = 50;    // minimal speed to overcome stiction
const int maxSpeed = 255;   // full speed
const int step = 20;        // speed ramp increment

void setup() {
  Serial.begin(115200);

  // Motor pins setup
  int motorPins[] = {M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2};
  for(int i=0; i<8; i++) pinMode(motorPins[i], OUTPUT);

  Serial.println("4-Motor N12 Test - DRV8833");
}

void loop() {
  // Motor 1
  Serial.println("Motor 1 Forward");
  runMotor(M1_IN1, M1_IN2, true);
  Serial.println("Motor 1 Backward");
  runMotor(M1_IN1, M1_IN2, false);

  // Motor 2
  Serial.println("Motor 2 Forward");
  runMotor(M2_IN1, M2_IN2, true);
  Serial.println("Motor 2 Backward");
  runMotor(M2_IN1, M2_IN2, false);

  // Motor 3
  Serial.println("Motor 3 Forward");
  runMotor(M3_IN1, M3_IN2, true);
  Serial.println("Motor 3 Backward");
  runMotor(M3_IN1, M3_IN2, false);

  // Motor 4
  Serial.println("Motor 4 Forward");
  runMotor(M4_IN1, M4_IN2, true);
  Serial.println("Motor 4 Backward");
  runMotor(M4_IN1, M4_IN2, false);

  Serial.println("All motors stopped");
  stopAllMotors();
  delay(3000); // wait before repeating
}

// Function to run motor with gradual ramp
void runMotor(int pinA, int pinB, bool forward){
  if(forward){
    for(int speed=minSpeed; speed<=maxSpeed; speed+=step){
      analogWrite(pinA, speed);
      analogWrite(pinB, 0);
      delay(300);
    }
  } else {
    for(int speed=minSpeed; speed<=maxSpeed; speed+=step){
      analogWrite(pinA, 0);
      analogWrite(pinB, speed);
      delay(300);
    }
  }
  stopMotor(pinA, pinB); // stop after running
}

// Function to stop a single motor
void stopMotor(int pinA, int pinB){
  analogWrite(pinA, 0);
  analogWrite(pinB, 0);
}

// Function to stop all motors
void stopAllMotors(){
  stopMotor(M1_IN1, M1_IN2);
  stopMotor(M2_IN1, M2_IN2);
  stopMotor(M3_IN1, M3_IN2);
  stopMotor(M4_IN1, M4_IN2);
}
