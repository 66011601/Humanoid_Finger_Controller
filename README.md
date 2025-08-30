# Humanoid_Finger_Controller

# Hand Moving & Data Sending Demo

This repository contains Arduino sketches for:
- **Hand_Moving.ino** – Controls servo/actuator movements based on input.
- **Sending_Data_Demo.ino** – Demonstrates sending sensor data over serial (or wireless, depending on setup).

### Libraries
Make sure to install the following libraries in the Arduino IDE:

- [Wire](https://www.arduino.cc/en/reference/wire) (I²C communication)
- [Adafruit PWM Servo Driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)  
  `#include <Adafruit_PWMServoDriver.h>`
- [HX711](https://github.com/bogde/HX711)  
  `#include "HX711.h"`
- [SPI](https://www.arduino.cc/en/reference/SPI) (Serial Peripheral Interface)

## Setup
1. Clone this repository:
   ```bash
   git clone https://github.com/66011601/Humanoid_Finger_Controller.git
