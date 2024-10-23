# Soccer Robot Controlled via PS4 Controller using ESP32 and BTS7960 Motor Drivers

## Table of Contents

- [Introduction](#introduction)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Circuit Diagram](#circuit-diagram)
- [Installation and Setup](#installation-and-setup)
  - [1. Setting Up the Arduino IDE](#1-setting-up-the-arduino-ide)
  - [2. Installing ESP32 Board Support](#2-installing-esp32-board-support)
  - [3. Installing Required Libraries](#3-installing-required-libraries)
- [Wiring Instructions](#wiring-instructions)
  - [1. ESP32 to BTS7960 Motor Drivers](#1-esp32-to-bts7960-motor-drivers)
  - [2. Motor Connections](#2-motor-connections)
  - [3. Power Supply](#3-power-supply)
- [Code Explanation](#code-explanation)
  - [1. Overview](#1-overview)
  - [2. Code Breakdown](#2-code-breakdown)
- [Usage Instructions](#usage-instructions)
  - [1. Pairing the PS4 Controller](#1-pairing-the-ps4-controller)
  - [2. Operating the Robot](#2-operating-the-robot)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)
- [License](#license)
- [Credits](#credits)

---

## Introduction

This project involves building a soccer robot controlled by a PS4 DualShock controller using an ESP32 microcontroller and BTS7960 motor drivers. The robot utilizes four DC motors to maneuver, making it suitable for robotics competitions or educational purposes.

---

## Hardware Requirements

- **ESP32 Development Board**
  - A microcontroller with built-in Wi-Fi and Bluetooth capabilities.

- **BTS7960 Motor Drivers (2x)**
  - High-current motor drivers capable of handling up to 43A, suitable for controlling DC motors.

- **DC Motors (4x)**
  - Ensure they are compatible with the voltage and current ratings of the BTS7960 drivers.

- **PS4 DualShock Controller**
  - For wireless control of the robot via Bluetooth.

- **Power Supply**
  - Batteries or a power source capable of supplying adequate current to the motors and electronics.

- **Jumper Wires**
  - For making connections between components.

- **Logic Level Converters (Optional)**
  - To safely interface 3.3V logic (ESP32) with 5V logic (BTS7960).

- **Miscellaneous Components**
  - Breadboard or PCB, connectors, resistors (if needed), and mounting hardware.

---

## Software Requirements

- **Arduino IDE**
  - Version 1.8.13 or newer recommended.

- **ESP32 Board Support Package**
  - Adds ESP32 compatibility to the Arduino IDE.

- **PS4-ESP32 Library**
  - Enables communication between the ESP32 and the PS4 controller.

---

## Circuit Diagram

*Note: Actual circuit diagrams are not included here. Please refer to wiring instructions in the next section for detailed connections.*

---

## Installation and Setup

### 1. Setting Up the Arduino IDE

1. **Download and Install Arduino IDE**
   - [Arduino IDE Download Page](https://www.arduino.cc/en/software)

2. **Launch the Arduino IDE**

### 2. Installing ESP32 Board Support

1. **Open Preferences**
   - On macOS: **Arduino IDE** > **Preferences...**
   - On Windows/Linux: **File** > **Preferences**

2. **Add Additional Boards Manager URL**
   - In the **"Additional Boards Manager URLs:"** field, enter:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Click **OK**.

3. **Install ESP32 Boards via Boards Manager**
   - Go to **Tools** > **Board** > **Boards Manager...**
   - Search for **"esp32"**.
   - Install **"esp32 by Espressif Systems"**.

4. **Select Your ESP32 Board**
   - Go to **Tools** > **Board** > **ESP32 Arduino**.
   - Select **"ESP32 Dev Module"** or the board that matches your hardware.

### 3. Installing Required Libraries

1. **Install PS4-ESP32 Library**
   - Go to **Sketch** > **Include Library** > **Manage Libraries...**
   - Search for **"PS4-ESP32"**.
   - Install the library by **aed3**.

2. **Verify Installation**
   - The library should now be available for inclusion in your sketches.

---

## Wiring Instructions

### 1. ESP32 to BTS7960 Motor Drivers

**Note:** You need to make connections for two BTS7960 motor drivers—one for the left motors and one for the right motors.

#### **ESP32 to Left BTS7960 Driver**

- **PWM Pin (RPWM)**
  - `GPIO 18` (leftMotorPwmPin) to **RPWM** on BTS7960.

- **Direction Pin (LPWM)**
  - `GPIO 19` (leftMotorDirectionPin) to **LPWM** on BTS7960.

- **Enable Pins (R_EN and L_EN)**
  - **Option 1:** Connect both **R_EN** and **L_EN** to `GPIO 23` (leftMotorEnablePin). Set pin HIGH in code to enable.
  - **Option 2:** Tie both **R_EN** and **L_EN** directly to 5V (always enabled).

- **VCC and GND**
  - Connect **VCC** to 5V (logic supply).
  - Connect **GND** to common ground.

#### **ESP32 to Right BTS7960 Driver**

- **PWM Pin (RPWM)**
  - `GPIO 21` (rightMotorPwmPin) to **RPWM** on BTS7960.

- **Direction Pin (LPWM)**
  - `GPIO 22` (rightMotorDirectionPin) to **LPWM** on BTS7960.

- **Enable Pins (R_EN and L_EN)**
  - **Option 1:** Connect both **R_EN** and **L_EN** to `GPIO 25` (rightMotorEnablePin). Set pin HIGH in code to enable.
  - **Option 2:** Tie both **R_EN** and **L_EN** directly to 5V (always enabled).

- **VCC and GND**
  - Connect **VCC** to 5V (logic supply).
  - Connect **GND** to common ground.

#### **Logic Level Considerations**

- **ESP32 Logic Level:** 3.3V
- **BTS7960 Logic Level:** 5V
- **Solution:** Use logic level converters to safely interface the ESP32 with the BTS7960 driver if necessary.

### 2. Motor Connections

- **Left Motors**
  - Connect the two left motors in parallel to the **M+** and **M-** terminals of the left BTS7960 driver.

- **Right Motors**
  - Connect the two right motors in parallel to the **M+** and **M-** terminals of the right BTS7960 driver.

### 3. Power Supply

- **Motor Power Supply**
  - Connect the positive terminal to **B+** and negative terminal to **B-** on both BTS7960 drivers.
  - Ensure the supply voltage matches the motors' voltage rating.

- **ESP32 Power Supply**
  - Power the ESP32 via USB or a separate regulated 5V supply.
  - **Important:** Do not power the ESP32 directly from the motor power supply.

- **Common Ground**
  - Connect all grounds (ESP32, BTS7960 drivers, and power supplies) together to maintain a common reference point.

---

## Code Explanation

### 1. Overview

The provided code initializes the ESP32, sets up motor control via PWM, and interfaces with the PS4 controller to control the robot's movement based on analog stick inputs.

### 2. Code Breakdown

```cpp
// Include necessary libraries
#include <Arduino.h>
#include <PS4Controller.h>
```

- Includes the core Arduino library and the PS4 controller library.

```cpp
// Define motor control pins
const int leftMotorPwmPin = 18;
const int leftMotorDirectionPin = 19;
const int rightMotorPwmPin = 21;
const int rightMotorDirectionPin = 22;
```

- Assigns GPIO pins for motor PWM and direction control.

```cpp
// Enable pins (if used)
const int leftMotorEnablePin = 23;
const int rightMotorEnablePin = 25;
```

- Assigns GPIO pins for enabling the motor drivers.

```cpp
// PWM settings
const int freq = 5000;
const int pwmResolution = 8;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
```

- Configures PWM frequency, resolution, and channels.

```cpp
void setup() {
  Serial.begin(115200);
  // Initialize motor control pins
  pinMode(leftMotorDirectionPin, OUTPUT);
  pinMode(rightMotorDirectionPin, OUTPUT);
  // Initialize motor enable pins
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  digitalWrite(leftMotorEnablePin, HIGH);
  digitalWrite(rightMotorEnablePin, HIGH);
  // Configure PWM functionalities
  ledcSetup(pwmChannelLeft, freq, pwmResolution);
  ledcSetup(pwmChannelRight, freq, pwmResolution);
  // Attach the PWM channels
  ledcAttachPin(leftMotorPwmPin, pwmChannelLeft);
  ledcAttachPin(rightMotorPwmPin, pwmChannelRight);
  // Initialize PS4 controller
  if (PS4.begin()) {
    Serial.println("PS4 controller is ready to connect.");
  } else {
    Serial.println("Failed to initialize PS4 controller.");
  }
}
```

- Sets up serial communication, initializes pins, configures PWM, and starts the PS4 controller connection.

```cpp
void loop() {
  if (PS4.isConnected()) {
    // Read analog stick values
    int leftStickY = PS4.LStickY();
    int rightStickY = PS4.RStickY();
    // Map stick values to motor speeds
    int leftMotorSpeed = map(leftStickY, -128, 127, -255, 255);
    int rightMotorSpeed = map(rightStickY, -128, 127, -255, 255);
    // Control motors
    controlMotor(pwmChannelLeft, leftMotorDirectionPin, leftMotorSpeed);
    controlMotor(pwmChannelRight, rightMotorDirectionPin, rightMotorSpeed);
  }
}
```

- In the main loop, checks for PS4 controller connection, reads analog stick inputs, maps them to motor speeds, and controls the motors accordingly.

```cpp
void controlMotor(int pwmChannel, int directionPin, int speed) {
  if (speed >= 0) {
    digitalWrite(directionPin, HIGH);
    ledcWrite(pwmChannel, speed);
  } else {
    digitalWrite(directionPin, LOW);
    ledcWrite(pwmChannel, -speed);
  }
}
```

- Defines a function to control motor speed and direction based on the input speed value.

---

## Usage Instructions

### 1. Pairing the PS4 Controller

1. **Put the PS4 Controller in Pairing Mode**
   - Press and hold the **Share** and **PS** buttons simultaneously.
   - The light bar will start flashing, indicating it's in pairing mode.

2. **Power On the ESP32**
   - Ensure the ESP32 is powered and running the uploaded code.
   - The ESP32 will automatically connect to the PS4 controller.

3. **Confirm Connection**
   - Open the Serial Monitor in the Arduino IDE (baud rate 115200).
   - Look for the message **"PS4 controller is ready to connect."**
   - Once connected, the message **"PS4 controller connected successfully."** should appear.

### 2. Operating the Robot

- **Left Analog Stick**
  - Controls the speed and direction of the left motors.
  - Pushing forward moves the left motors forward; pulling back reverses them.

- **Right Analog Stick**
  - Controls the speed and direction of the right motors.
  - Similar to the left stick but for the right motors.

- **Maneuvering**
  - **Forward Movement:** Push both sticks forward.
  - **Reverse Movement:** Pull both sticks backward.
  - **Turning Left:** Push the right stick forward and the left stick neutral or backward.
  - **Turning Right:** Push the left stick forward and the right stick neutral or backward.

---

## Troubleshooting

- **Compilation Errors**
  - **`ledcSetup` Not Declared:**
    - Ensure ESP32 board support is installed and selected in the Arduino IDE.
  - **Library Not Found:**
    - Verify that the PS4-ESP32 library is correctly installed.

- **PS4 Controller Not Connecting**
  - **Incorrect Pairing Mode:**
    - Ensure the controller's light bar is flashing rapidly.
  - **Bluetooth Issues:**
    - Verify that the ESP32's Bluetooth is functioning.

- **Motors Not Responding**
  - **Wiring Issues:**
    - Double-check all connections between the ESP32, BTS7960 drivers, and motors.
  - **Enable Pins Not Set:**
    - Ensure the enable pins are set HIGH or connected to 5V.
  - **Power Supply:**
    - Confirm that the power supply is adequate for the motors.

- **Motor Direction Incorrect**
  - Swap the motor connections on the BTS7960 driver or adjust the logic in the `controlMotor` function.

- **ESP32 Resets or Freezes**
  - **Power Issues:**
    - Use a separate, stable power supply for the ESP32.
  - **Noise Interference:**
    - Add decoupling capacitors and ensure proper grounding.

---

## Additional Resources

- **ESP32 Documentation**
  - [ESP32 Arduino Core Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
  - [ESP32 PWM (LEDC) API Reference](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/pwm.html)

- **PS4-ESP32 Library**
  - [PS4-ESP32 GitHub Repository](https://github.com/aed3/PS4-esp32)

- **BTS7960 Motor Driver Information**
  - [BTS7960 Datasheet](https://www.infineon.com/dgdl/Infineon-BTS7960_DS-2014.pdf)

- **Arduino IDE Download**
  - [Arduino IDE](https://www.arduino.cc/en/software)

---

## License

This project is open-source and available under the [MIT License](https://opensource.org/licenses/MIT).

---

## Credits

- **Project Creator:** *Your Name Here*
- **Assistance and Guidance:** OpenAI's ChatGPT
- **Libraries Used:**
  - PS4-ESP32 Library by aed3
- **Special Thanks:**
  - Arduino Community
  - ESP32 Developers
  - Robotics Enthusiasts

---

**Disclaimer:** This project involves working with electronic components and power supplies. Ensure all safety precautions are followed. The creator is not responsible for any damage or injury resulting from the use of this guide.
