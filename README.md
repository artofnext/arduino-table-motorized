
# Motorized Table Deck Controller

A smart, Arduino-powered controller for a motorized sit/stand desktop with ultrasonic height sensing, memory presets, hardware-interrupt controls, and a clean OLED UI.

## Overview

This project provides a high-performance control system for a vertically actuated table deck.

Features include:

- Precise ultrasonic height measurements  
- Fully non-blocking motor control  
- Hardware I2C OLED display  
- Three programmable memory positions  
- Auto-detection of saved memory heights  
- Interrupt-driven UP/DOWN buttons  
- Safety limits and error messages  

---

## Key Features

### Advanced Motion Control
- Instant reaction time using interrupt-driven buttons
- Non-blocking motor start protection delay
- Automatic stopping at min/max boundaries
- Stable, filtered height measurements

### Height Sensing
- HC-SR04 ultrasonic sensor
- Median filtering when stationary
- Fast single-shot reading when moving
- Sensor error detection with OLED warnings

### Safety
- Min height: 67.0 cm  
- Max height: 111.0 cm  
- Stops automatically at boundaries  
- OLED error messages:
  - Sensor error
  - Out-of-range
  - Future motor stall detection capability

### Memory System (3 Slots)
- Memory slots M1, M2, M3
- Saved in EEPROM
- Selectable via MEMORY button
- Save/erase via long press
- Adjustable tolerance (default: ±1.0 cm)
- Auto-detect ANY saved memory height:
  - Stops automatically
  - Turns on yellow LED

### Memory Button Behavior
- Short press cycles:

OFF -> M1 -> M2 -> M3 -> OFF ...

- Long press (>= 2 seconds):
- If empty: saves current height
- If filled: erases slot
- OLED displays "Saved" or "Erased"

### Visual Indicators
| LED | Meaning |
|-----|---------|
| Green | Blinks during movement |
| Yellow | Near a saved memory height |
| Red | Reserved for future safety |

### OLED Display
Shows:
- Current height
- Movement status (UP / DN / STOP)
- Selected memory slot
- Heartbeat activity
- Error messages when needed

---

## Hardware Requirements

### Components
- Arduino Uno or Nano  
- HC-SR04 ultrasonic sensor  
- L298N or similar H-bridge motor driver  
- OLED SSD1306 128x32 (I2C)  
- Buttons: UP, DOWN, MEMORY  
- LEDs: Red, Green, Yellow  
- 12V power supply for motor  
- 5V regulator for logic  
- Wiring suitable for motor load  

---

## Pin Mapping

| Component | Pin | Description |
|----------|-----|-------------|
| HC-SR04 TRIG | 3 | Trigger |
| HC-SR04 ECHO | 2 | Echo |
| Motor Dir A | 4 | Forward |
| Motor Dir B | 6 | Reverse |
| UP Button | 11 | Interrupt pin |
| DOWN Button | 12 | Interrupt pin |
| MEMORY Button | 10 | Active LOW |
| RED LED | 9 | Reserved |
| GREEN LED | 8 | Motor activity |
| YELLOW LED | 7 | Memory indicator |
| OLED SDA | A4 | I2C Data |
| OLED SCL | A5 | I2C Clock |

---

## Setup & Installation

1. Wire components according to the pin table.  
2. Ensure the motor driver is wired for polarity reversal.  
3. Supply 12V for motor, and regulated 5V for Arduino and logic.  
4. Install Arduino libraries:
 - U8g2  
 - EEPROM (built-in)
5. Open and upload the firmware `.ino` file.

---

## Usage

### Moving the Table
- Press UP or DOWN to start movement.
- Press again to stop.
- Movement stops automatically at limits.

### Memory System

#### Short Press (cycle)

OFF -> M1 -> M2 -> M3 -> OFF ...
#### Long Press (>= 2 seconds)
- If slot empty -> saves height  
- If slot filled -> erases slot  
- OLED shows confirmation message

#### Auto-Detection
If current height matches any saved memory (within tolerance):
- Motor stops
- Yellow LED turns ON

### EEPROM Reset
Hold **UP + DOWN** for 3 seconds to recalibrate stored height.

---

## Safety

- Ensure motor/load rating is sufficient.
- Ultrasonic sensor must have a clear view.
- Avoid exceeding table's mechanical range.
- Provide proper fusing and wire gauge for motor current.

---

## Changelog (Latest Firmware)

- Hardware I2C OLED (fast)
- Interrupt UP/DOWN buttons
- 3 slot memory system with save/erase
- Auto-detect memory heights
- Adjustable memory tolerance
- Non-blocking architecture
- Fast ultrasonic when moving
- Median ultrasonic when idle
- OLED error reporting
- EEPROM write minimization
- Complete project refactor


