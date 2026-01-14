# Motorized Table Deck Controller

A smart Arduino-based controller for a motorized table deck featuring height monitoring via ultrasonic sensing, safety limits, and non-volatile memory storage.

## 🚀 Overview

This project provides a robust control system for a table that moves up and down. It uses an ultrasonic sensor to monitor height in real-time, ensuring the table stays within safe physical limits. The controller features a toggle-based movement logic, allowing for easy operation with simple button presses.

## ✨ Key Features

- **Toggle Mode Operation**: Press a button once to start moving, and press any button again to stop.
- **Precision Height Sensing**: Uses the HC-SR04 ultrasonic sensor with a median filter to eliminate noise and erratic readings.
- **Safety Limits**: 
    - **Minimum Height**: 67.0 cm
    - **Maximum Height**: 111.0 cm
    - Automatic stop when limits are reached or if sensor readings become untrusted.
- **Persistent Storage**: Current height is saved to EEPROM upon stopping, preventing data loss during power cycles.
- **Safety Start Delay**: 2-second safety delay on power-up with LED status indication.
- **EEPROM Reset**: Long-press (3s) both UP and DOWN buttons to reset the stored height to the current measurement.
- **Visual Feedback**: 
    - **Green LED**: Blinks during movement.
    - **Red LED**: Signals safety locks or limit reaches.
    - **Yellow LED**: Dedicated memory indicator.

## 🛠 Hardware Requirements

### Components
- **Microcontroller**: Arduino (e.g., Uno, Nano)
- **Sensor**: HC-SR04 Ultrasonic Sensor
- **Motor Control**: 2x Relay Module (configured for polarity switching)
- **Power Supply**: 12V 2A AC-DC Converter
- **Step-Down Converter**: DC-DC Converter (12V to 5V) to power the Arduino and logic
- **Buttons**: 3x Push Buttons (UP, DOWN, MEMORY)
- **Status LEDs**: Red, Green, Yellow

### Pin Mapping

| Component | Pin | Note |
| :--- | :--- | :--- |
| **HC-SR04 TRIG** | 3 | Trigger pulse |
| **HC-SR04 ECHO** | 2 | Echo return |
| **Relay A (Up)** | 4 | Forward Direction |
| **Relay B (Down)** | 6 | Reverse Direction |
| **UP Button** | 11 | Active LOW (Internal Pullup) |
| **DOWN Button** | 12 | Active LOW (Internal Pullup) |
| **MEMORY Button**| 10 | Reserved |
| **RED LED** | 9 | Security/Error indicator |
| **GREEN LED** | 8 | Movement indicator |
| **YELLOW LED** | 7 | Memory indicator |

## 🔌 Setup & Installation

1.  **Wiring**: Connect the components according to the Pin Mapping table.
    - *Note*: Ensure the dual relay module is wired to allow polarity reversal for the motor.
    - *Power*: Connect the 12V 2A supply to the relay common terminals for the motor, and use the DC-DC converter to provide stable 5V to the Arduino and sensors.
2.  **Arduino IDE**:
    - Install the Arduino IDE.
    - Ensure the standard `EEPROM.h` library is available (included by default).
3.  **Upload**:
    - Open `table_motorized_01.ino`.
    - Select your board and port.
    - Click **Upload**.

## 📖 Usage Instructions

- **Move Up/Down**: Press the UP or DOWN button once to begin movement.
- **Stop**: Press **Up** or **Down**  button while the motor is running to stop the table immediately.
- **Limits**: The motor will automatically stop if the table reaches 111 cm (MAX) or 67 cm (MIN).
- **Reset Memory**: If you need to re-calibrate the stored distance, hold both **UP** and **DOWN** buttons for 3 seconds. The LEDs will blink to confirm the reset to the current height.

## ⚠️ Safety Precautions

- **Load Capacity**: Ensure your motor and relays are rated for the weight of your table deck.
- **Sensor Obstruction**: Keep the path of the ultrasonic sensor clear for accurate height measurements.
- **Safety Lock**: If the controller detects a sudden jump in sensor readings (e.g., > 2cm tolerance), it will trigger a safety lock and refuse to move until the reading stabilizes.
