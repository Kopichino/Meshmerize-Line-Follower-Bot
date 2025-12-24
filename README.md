# Meshmerize 2025 - Autonomous Line Follower

This repository contains the firmware and calibration scripts for our autonomous line follower robot, designed for the **Meshmerize** competition at **Techfest 2025-26 (IIT Bombay)**.

## 🏆 Competition Objective
The goal is to build an autonomous robot that can:
1.  **Dry Run:** Navigate a maze of white lines on a black arena to analyze the path from 'Start' to 'End'.
2.  **Actual Run:** Calculate and execute the shortest path from memory in the minimum possible time.
3.  **Constraints:** The bot must fit within a 220x220x220 mm box and complete the dry run under 3 minutes.

## 🛠️ Hardware Specifications
Based on the driver code included in this repo:
* **Microcontrollers:** Teensy 4.1  & Arduino.
* **Sensors:** * QTRX-MD08A / MD08RC (8-Channel Reflectance Sensor Array).
    * HY-S301 IR Sensors (Digital).
* **Motor Drivers:** DRV8833 (controlling 4 motors).
* **Motors:** N12 Motors.

## 📂 File Structure

| File Name | Description |
| :--- | :--- |
| `PID_follower1.ino` | Main PID control loop using 5 weighted sensors and EEPROM calibration storage. |
| `calib_code1.ino` | Specialized calibration routine for Teensy 4.1; calculates sensor min/max and stores in EEPROM. |
| `sensor_calib_code_meshmerize.ino` | Basic analog calibration script for QTRX-MD08A sensors (normalizes readings 0-1000). |
| `PID1.ino` | Alternative PID logic handling 4-motor drive and junction detection. |
| `motor_drill1.ino` | Hardware test script for ramping N12 motors up/down to check direction and wiring. |
| `sensor_calib_code_meshmerize_hys301.ino` | Digital sensor test code for HY-S301 modules. |

## ⚙️ Key Logic & Features

### 1. Calibration
The robot uses a calibration phase (approx. 5 seconds) to record the Minimum (White) and Maximum (Black) reflectance values.
* **Storage:** Values are saved to **EEPROM** so calibration persists after power cycling.
* **Normalization:** Sensor readings are mapped to a 0-1000 range for consistent PID calculation.

### 2. PID Control
The robot maintains its position on the line using a Proportional-Integral-Derivative controller:
* **Error Calculation:** Derived from the weighted average of active sensors.
* **Motor Mixing:** The PID output adjusts the PWM duty cycle for the left and right motors to correct course.
* **Tunable Constants:** `Kp`, `Ki`, and `Kd` values are defined in the headers for easy tuning.

### 3. Safety & Rules Compliance
* **Dimensions:** Code assumes a chassis fitting the 220mm limit.
* **Voltage:** Hardware configuration respects the 24V potential difference limit.

## 🚀 Usage
1.  **Motor Check:** Flash `motor_drill1.ino` to verify wheel direction and motor driver wiring.
2.  **Calibration:** Flash `calib_code1.ino`. Place the bot on the line and move it back and forth over black/white areas until the LED stops blinking.
3.  **Run:** Flash `PID_follower1.ino` (or your preferred version). The bot will load saved calibration data and begin tracking.

## 📜 Credits
* **Team:** ATOM ROBOTICS, VIT Chennai
* **Event:** Techfest 2025-26, IIT Bombay.
