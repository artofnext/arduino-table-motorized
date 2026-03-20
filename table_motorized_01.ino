#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// ----------------- PIN DEFINITIONS -----------------
const int ENCODER_PIN_A = 2; // INT0
const int ENCODER_PIN_B = 3; // INT1
const int LIMIT_LOW_PIN = 4; // Low Limit Switch (PCINT)

const int MOTOR_DIR_A_PIN = 5;
const int MOTOR_DIR_B_PIN = 6;

const int BTN_UP_PIN = 12;
const int BTN_DOWN_PIN = 11;
const int BTN_MEMORY_PIN = 10;

const int LED_RED_PIN = 9;
const int LED_GREEN_PIN = 8;
const int LED_YELLOW_PIN = 7;

// -------------- OLED (Hardware I2C A4/A5) -----------
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

// ---------------- CONTROL PARAMETERS ----------------
const int EEPROM_COUNT_ADDR = 0; // Saved encoder count (long)

// EEPROM memory slots (saved as encoder counts)
const int EEPROM_M1_ADDR = 10;
const int EEPROM_M2_ADDR = 20;
const int EEPROM_M3_ADDR = 30;

// Configurable constants
// min height 71 cm, max height 119 cm, total rotations of the shaft 105, encoder 10 pulse/rotation
const float ENCODER_CM_PER_COUNTS = 0.0457; // CM per count (Multiplier)
const long MIN_COUNT = 0;                  // Homing point
const long MAX_COUNT = 1050;               // Maximum travel steps

const float MIN_DISTANCE_CM = 71.0; // Reference min height in cm
const float MAX_DISTANCE_CM = 119.0;

const unsigned long DISPLAY_REFRESH_INTERVAL = 300;
const unsigned long LED_BLINK_INTERVAL = 200;
const unsigned long RESET_HOLD_TIME = 3000;
const unsigned long MEMORY_LONG_PRESS = 2000;
const unsigned long SLEEP_TIMEOUT = 30000; // 30 seconds to sleep

const long MEMORY_TOLERANCE_COUNT = 10; // ± steps for memory stop

// -------------------- STATE -------------------------
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN, HOMING_DOWN, HOMING_UP };
MotorState motorState = STOPPED;

volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastPulseTime = 0;
float currentDistance = 0.0;
float previousDisplayedDistance = -999;
MotorState lastDisplayedState = STOPPED;

unsigned long lastBlinkTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastActivityTime = 0;
bool isAsleep = false;
bool greenLedState = false;

bool dualButtonActive = false;
unsigned long dualButtonStartTime = 0;

// Interrupt flags
volatile bool upPressedISR = false;
volatile bool downPressedISR = false;
volatile bool limitHitISR = false;

// Memory system
long memSlots[3] = {0, 0, 0};
int selectedSlot = -1; // -1 = OFF

unsigned long memButtonDownTime = 0;
bool memButtonWasHeld = false;

// Motor start delay
bool pendingMotorStart = false;
MotorState requestedMotorState = STOPPED;
unsigned long motorStartRequestTime = 0;

// Error state
char errorMessage[20] = "";
unsigned long errorStartTime = 0;
const unsigned long ERROR_DISPLAY_DURATION = 1500;

// Override to allow moving away from a memory position
bool ignoreMemoryStop = false;

// ======================== INTERRUPTS =========================
void setupPCINT() {
  // Group 0: Pins 8-13 (Buttons)
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2); // pin 10 (BTN_MEMORY)
  PCMSK0 |= (1 << PCINT3); // pin 11 (BTN_DOWN)
  PCMSK0 |= (1 << PCINT4); // pin 12 (BTN_UP)

  // Group 2: Pins 0-7 (Limit Switch on Pin 4)
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20); // pin 4 (LIMIT_LOW)
}

ISR(PCINT0_vect) {
  static unsigned long lastInterrupt = 0;
  unsigned long now = millis();
  if (now - lastInterrupt < 30)
    return;

  if (!digitalRead(BTN_UP_PIN))
    upPressedISR = true;
  if (!digitalRead(BTN_DOWN_PIN))
    downPressedISR = true;

  lastInterrupt = now;
}

ISR(PCINT2_vect) {
  if (!digitalRead(LIMIT_LOW_PIN)) {
    limitHitISR = true;
  }
}

// Encoder Quadrature Decoding
void encoderISR() {
  static uint8_t lastState = 0;
  uint8_t currentState =
      (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);

  // Standard quadrature table
  if (lastState == 0b00) {
    if (currentState == 0b01)
      encoderCount--;
    else if (currentState == 0b10)
      encoderCount++;
  } else if (lastState == 0b01) {
    if (currentState == 0b11)
      encoderCount--;
    else if (currentState == 0b00)
      encoderCount++;
  } else if (lastState == 0b11) {
    if (currentState == 0b10)
      encoderCount--;
    else if (currentState == 0b01)
      encoderCount++;
  } else if (lastState == 0b10) {
    if (currentState == 0b00)
      encoderCount--;
    else if (currentState == 0b11)
      encoderCount++;
  }
  lastState = currentState;
}

void setupEncoder() {
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);
}

// ======================== POWER MANAGEMENT =========================
void wakeUp() {
  if (isAsleep) {
    isAsleep = false;
    u8g2.setPowerSave(0);
    lastActivityTime = millis();
    Serial.println(F("WAKE"));
  }
}

void checkSleep() {
  if (!isAsleep && motorState == STOPPED &&
      (millis() - lastActivityTime > SLEEP_TIMEOUT)) {
    isAsleep = true;
    u8g2.setPowerSave(1);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);
    Serial.println(F("SLEEP"));
  }
}

// ======================== ALERTS =========================
void blinkRedLed(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_RED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_RED_PIN, LOW);
    if (i < times - 1) delay(150);
  }
}

// ======================== MOTOR CONTROL =========================
void beginMotorMove(MotorState s) {
  requestedMotorState = s;
  motorStartRequestTime = millis();
  pendingMotorStart = true;
  lastPulseTime = millis();
  lastEncoderCount = encoderCount;
}

void applyMotorStartIfReady() {
  if (!pendingMotorStart)
    return;
  if (millis() - motorStartRequestTime < 500)
    return;

  if (requestedMotorState == MOVING_UP || requestedMotorState == HOMING_UP) {
    digitalWrite(MOTOR_DIR_A_PIN, HIGH);
    digitalWrite(MOTOR_DIR_B_PIN, LOW);
    Serial.println(requestedMotorState == MOVING_UP ? F("M:UP") : F("M:H_UP"));
  } else if (requestedMotorState == MOVING_DOWN ||
             requestedMotorState == HOMING_DOWN) {
    digitalWrite(MOTOR_DIR_A_PIN, LOW);
    digitalWrite(MOTOR_DIR_B_PIN, HIGH);
    Serial.println(requestedMotorState == MOVING_DOWN ? F("M:DN")
                                                      : F("M:H_DN"));
  }

  motorState = requestedMotorState;
  pendingMotorStart = false;
  lastActivityTime = millis();
  lastPulseTime = millis();
  lastEncoderCount = encoderCount;
}

void stopMotor() {
  if (motorState != STOPPED) {
    Serial.println(F("M:STOP"));
    if (motorState != HOMING_DOWN && motorState != HOMING_UP) {
      EEPROM.put(EEPROM_COUNT_ADDR, encoderCount);
    }
  }
  motorState = STOPPED;
  digitalWrite(MOTOR_DIR_A_PIN, LOW);
  digitalWrite(MOTOR_DIR_B_PIN, LOW);
  lastActivityTime = millis();
}

// ======================== DISPLAY =========================
void showError(const char *message) {
  strncpy(errorMessage, message, sizeof(errorMessage) - 1);
  errorMessage[sizeof(errorMessage) - 1] = '\0'; // Ensure null termination
  errorStartTime = millis();
  wakeUp();
}

void loadMemorySlots() {
  EEPROM.get(EEPROM_M1_ADDR, memSlots[0]);
  EEPROM.get(EEPROM_M2_ADDR, memSlots[1]);
  EEPROM.get(EEPROM_M3_ADDR, memSlots[2]);
}

void updateDisplay(long count, MotorState state) {
  if (isAsleep)
    return;

  float dist = MIN_DISTANCE_CM + (float)count * ENCODER_CM_PER_COUNTS;
  currentDistance = dist; // Update global for other checks

  // Check for active error
  if (errorStartTime > 0) {
    if (millis() - errorStartTime < ERROR_DISPLAY_DURATION) {
      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_6x13_tr);
        u8g2.drawStr(0, 16, errorMessage);
      } while (u8g2.nextPage());
      return; // Skip normal display
    } else {
      errorStartTime = 0; // Error expired
      errorMessage[0] = '\0';
      previousDisplayedDistance = -999;
    }
  }

  if (abs(dist - previousDisplayedDistance) < 0.1 &&
      state == lastDisplayedState)
    return;

  previousDisplayedDistance = dist;
  lastDisplayedState = state;

  static bool heartbeat = false;
  heartbeat = !heartbeat;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tr);

    u8g2.setCursor(0, 12);
    u8g2.print(F("Height: "));
    u8g2.print(dist, 1);
    u8g2.print(F(" cm"));

    // Slot indicator
    u8g2.setCursor(100, 12);
    if (selectedSlot == -1)
      u8g2.print(F("[--]"));
    else
      u8g2.print(selectedSlot == 0   ? "[M1]"
                 : selectedSlot == 1 ? "[M2]"
                                     : "[M3]");

    // Movement
    u8g2.setCursor(0, 28);
    if (state == MOVING_UP)
      u8g2.print(F("UP"));
    else if (state == MOVING_DOWN)
      u8g2.print(F("DOWN"));
    else
      u8g2.print(F("STOP"));

    // Heartbeat pixel
    if (heartbeat)
      u8g2.drawPixel(127, 0);

  } while (u8g2.nextPage());
}

// ======================== SETUP =========================
void setup() {
  setupEncoder();
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(LIMIT_LOW_PIN, INPUT_PULLUP);

  pinMode(MOTOR_DIR_A_PIN, OUTPUT);
  pinMode(MOTOR_DIR_B_PIN, OUTPUT);

  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_MEMORY_PIN, INPUT_PULLUP);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);

  Serial.begin(9600);
  setupPCINT();
  u8g2.begin();

  // Show Initializing message
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.drawStr(0, 16, "Initializing...");
  } while (u8g2.nextPage());

  loadMemorySlots();
  EEPROM.get(EEPROM_COUNT_ADDR, encoderCount);

  // LED Initialization Sequence
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);

  lastActivityTime = millis();
  updateDisplay(encoderCount, motorState);

  Serial.println(F("RDY"));
}

// ======================== LOOP =========================
void loop() {
  unsigned long now = millis();

  // 1. Dual-Button Homing Detection
  bool upPressed = !digitalRead(BTN_UP_PIN);
  bool downPressed = !digitalRead(BTN_DOWN_PIN);
  if (upPressed && downPressed) {
    if (dualButtonStartTime == 0) {
      dualButtonStartTime = now;
    } else if (now - dualButtonStartTime >= 3000 && motorState == STOPPED) {
      showError("Homing...");
      beginMotorMove(HOMING_DOWN);
      dualButtonStartTime = 0; // Reset to avoid re-triggering
    }
  } else {
    dualButtonStartTime = 0;
  }

  // 2. Handle Homing Sequence Logic
  if (motorState == HOMING_DOWN) {
    if (limitHitISR || !digitalRead(LIMIT_LOW_PIN)) {
      limitHitISR = false;
      stopMotor();
      blinkRedLed(3);
      delay(500); // Brief pause before reversal
      beginMotorMove(HOMING_UP);
    }
  } else if (motorState == HOMING_UP) {
    if (digitalRead(LIMIT_LOW_PIN)) { // Switch released (hysteresis)
      stopMotor();
      encoderCount = MIN_COUNT;
      EEPROM.put(EEPROM_COUNT_ADDR, encoderCount);
      showError("Homed");
    }
  }

  // 3. Normal Limit Switch Stop (Safety) & Pulse Bounds
  if (limitHitISR) {
    limitHitISR = false;
    if (motorState == MOVING_DOWN) {
      stopMotor();
      blinkRedLed(3);
      Serial.println(F("LIM:LOW"));
    }
  }

  if (encoderCount >= MAX_COUNT && motorState == MOVING_UP) {
    stopMotor();
    blinkRedLed(3);
    Serial.println(F("LIM:MAX"));
  }
  if (encoderCount <= MIN_COUNT && motorState == MOVING_DOWN) {
    stopMotor();
    blinkRedLed(3);
    Serial.println(F("LIM:MIN"));
  }

  // 4. Stall Detection (Jam Protection)
  if (motorState != STOPPED) {
    if (encoderCount != lastEncoderCount) {
      lastEncoderCount = encoderCount;
      lastPulseTime = now;
    } else if (now - lastPulseTime > 1000) {
      stopMotor();
      showError("ERROR: JAM");
      blinkRedLed(5);
    }
  }

  // 5. Activity and Display
  if (motorState != STOPPED) {
    lastActivityTime = now;
  }
  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    updateDisplay(encoderCount, motorState);
    lastDisplayUpdate = now;
  }

  // 6. Memory Button Logic
  bool memPressed = !digitalRead(BTN_MEMORY_PIN);
  if (memPressed && memButtonDownTime == 0) {
    wakeUp();
    memButtonDownTime = now;
    memButtonWasHeld = false;
  }
  if (!memPressed && memButtonDownTime != 0) {
    if (!memButtonWasHeld) {
      selectedSlot = (selectedSlot + 2) % 4 - 1; // Cycles: -1, 0, 1, 2
    }
    memButtonDownTime = 0;
  }
  if (memPressed && now - memButtonDownTime >= MEMORY_LONG_PRESS &&
      !memButtonWasHeld) {
    memButtonWasHeld = true;
    if (selectedSlot == -1) {
      showError("Select M1-M3");
    } else {
      memSlots[selectedSlot] = encoderCount;
      EEPROM.put(EEPROM_M1_ADDR + selectedSlot * 10, memSlots[selectedSlot]);
      showError("Saved");
    }
  }

  // 7. Auto-Detect Memory Stop
  bool nearAnyMemory = false;
  for (int i = 0; i < 3; i++) {
    if (abs(encoderCount - memSlots[i]) <= MEMORY_TOLERANCE_COUNT) {
      nearAnyMemory = true;
      if (motorState != STOPPED && motorState != HOMING_DOWN &&
          motorState != HOMING_UP && !ignoreMemoryStop) {
        stopMotor();
      }
    }
  }
  if (!nearAnyMemory)
    ignoreMemoryStop = false;

  if (!isAsleep)
    digitalWrite(LED_YELLOW_PIN, nearAnyMemory ? HIGH : LOW);

  // 8. Button Interrupts (from PCINT)
  if (upPressedISR) {
    upPressedISR = false;
    wakeUp();
    if (motorState == STOPPED) {
      if (encoderCount >= MAX_COUNT) {
        blinkRedLed(3);
        showError("At Max");
      } else {
        ignoreMemoryStop = true;
        beginMotorMove(MOVING_UP);
      }
    } else if (motorState == MOVING_UP || motorState == MOVING_DOWN) {
      stopMotor();
    }
  }

  if (downPressedISR) {
    downPressedISR = false;
    wakeUp();
    if (motorState == STOPPED) {
      if (encoderCount <= MIN_COUNT) {
        blinkRedLed(3);
        showError("At Min");
      } else {
        ignoreMemoryStop = true;
        beginMotorMove(MOVING_DOWN);
      }
    } else if (motorState == MOVING_UP || motorState == MOVING_DOWN) {
      stopMotor();
    }
  }

  // 9. Background Tasks
  applyMotorStartIfReady();
  checkSleep();

  // LED blinking while moving
  if (motorState != STOPPED && !isAsleep) {
    if (now - lastBlinkTime >= LED_BLINK_INTERVAL) {
      greenLedState = !greenLedState;
      digitalWrite(LED_GREEN_PIN, greenLedState);
      lastBlinkTime = now;
    }
  } else if (!isAsleep) {
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}
