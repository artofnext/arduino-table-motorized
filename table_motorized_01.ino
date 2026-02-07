
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// ----------------- PIN DEFINITIONS -----------------
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

const int MOTOR_DIR_A_PIN = 5;
const int MOTOR_DIR_B_PIN = 6;

const int BTN_UP_PIN = 11;
const int BTN_DOWN_PIN = 12;
const int BTN_MEMORY_PIN = 10;

const int LED_RED_PIN = 9;
const int LED_GREEN_PIN = 8;
const int LED_YELLOW_PIN = 7;

// -------------- OLED (Hardware I2C A4/A5) -----------
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

// ---------------- CONTROL PARAMETERS ----------------
const int EEPROM_DISTANCE_ADDR = 0;

// EEPROM memory slots
const int EEPROM_M1_ADDR = 10;
const int EEPROM_M2_ADDR = 20;
const int EEPROM_M3_ADDR = 30;

const float MIN_DISTANCE_CM = 67.0;
const float MAX_DISTANCE_CM = 111.0;

const unsigned long DISPLAY_REFRESH_INTERVAL = 300;
const unsigned long LED_BLINK_INTERVAL = 200;
const unsigned long RESET_HOLD_TIME = 3000;
const unsigned long MEMORY_LONG_PRESS = 2000;

const float MEMORY_TOLERANCE_CM = 1.0; // ±1.0 cm configurable

// -------------------- STATE -------------------------
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN };
MotorState motorState = STOPPED;

float currentDistance = 0.0;
float previousDistance = 0.0;
float previousDisplayedDistance = -999;
MotorState lastDisplayedState = STOPPED;

unsigned long lastBlinkTime = 0;
unsigned long lastDisplayUpdate = 0;
bool greenLedState = false;

bool dualButtonActive = false;
unsigned long dualButtonStartTime = 0;

// Interrupt flags
volatile bool upPressedISR = false;
volatile bool downPressedISR = false;

// Memory system
float memSlots[3] = {NAN, NAN, NAN};
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
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT2); // pin 10
  PCMSK0 |= (1 << PCINT3); // pin 11
  PCMSK0 |= (1 << PCINT4); // pin 12
}

ISR(PCINT0_vect) {
  static unsigned long lastInterrupt = 0;
  unsigned long now = millis();
  if (now - lastInterrupt < 30)
    return;

  if (!digitalRead(BTN_UP_PIN)) {
    upPressedISR = true;
    Serial.println("[ISR] UP button interrupt");
  }
  if (!digitalRead(BTN_DOWN_PIN)) {
    downPressedISR = true;
    Serial.println("[ISR] DOWN button interrupt");
  }

  lastInterrupt = now;
}

// ======================== ULTRASONIC =========================
float readUltrasonicFast() {
  static unsigned long lastTrigger = 0;
  unsigned long now = millis();

  // Enforce 60ms cycle (Datasheet requirement + echo dissipation)
  if (now - lastTrigger < 60) {
    return currentDistance; // Return last known good distance during cooling
                            // period
  }

  // Internal retry burst (up to 3 attempts to handle relay spikes)
  for (int retry = 0; retry < 3; retry++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Disable interrupts briefly to ensure pulseIn timing accuracy
    noInterrupts();
    long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
    interrupts();

    if (duration > 0) {
      lastTrigger = millis();
      float distance = duration * 0.0343 / 2.0;
      Serial.print("[Sensor] Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      return distance;
    }

    if (retry < 2)
      delay(10); // Short gap between internal retries
  }

  Serial.println("[Sensor] ERROR: All 3 retries failed");
  return NAN; // True error only if 3 pulses fail
}

float readUltrasonicMedian() {
  float arr[5];
  int validCount = 0;

  for (int i = 0; i < 5; i++) {
    float val = readUltrasonicFast();
    if (!isnan(val)) {
      arr[validCount] = val;
      validCount++;
    }
    // readUltrasonicFast already manages the 60ms delay
  }

  if (validCount == 0)
    return NAN;

  for (int i = 0; i < validCount - 1; i++) {
    for (int j = 0; j < validCount - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }

  return arr[validCount / 2];
}

// ======================== MOTOR CONTROL =========================
void beginMotorMove(MotorState s) {
  requestedMotorState = s;
  motorStartRequestTime = millis();
  pendingMotorStart = true;
  Serial.print("[Motor] Requested: ");
  Serial.println(s == MOVING_UP ? "MOVING_UP" : "MOVING_DOWN");
  Serial.println("[Motor] Waiting 500ms before start...");
}

void applyMotorStartIfReady() {
  if (!pendingMotorStart)
    return;
  if (millis() - motorStartRequestTime < 500)
    return;

  if (requestedMotorState == MOVING_UP) {
    digitalWrite(MOTOR_DIR_A_PIN, HIGH);
    digitalWrite(MOTOR_DIR_B_PIN, LOW);
    Serial.println("[Motor] STARTED - Direction: UP");
  } else if (requestedMotorState == MOVING_DOWN) {
    digitalWrite(MOTOR_DIR_A_PIN, LOW);
    digitalWrite(MOTOR_DIR_B_PIN, HIGH);
    Serial.println("[Motor] STARTED - Direction: DOWN");
  }

  motorState = requestedMotorState;
  pendingMotorStart = false;
}

void stopMotor() {
  if (motorState != STOPPED) {
    Serial.println("[Motor] STOPPED");
  }
  motorState = STOPPED;
  digitalWrite(MOTOR_DIR_A_PIN, LOW);
  digitalWrite(MOTOR_DIR_B_PIN, LOW);
}

// ======================== DISPLAY =========================
void showError(const char *message) {
  strncpy(errorMessage, message, sizeof(errorMessage) - 1);
  errorMessage[sizeof(errorMessage) - 1] = '\0'; // Ensure null termination
  errorStartTime = millis();
  Serial.print("[Display] Error shown: ");
  Serial.println(message);
}

// Helper functions for EEPROM
void saveFloatToEEPROM(int addr, float val) { EEPROM.put(addr, val); }

float loadFloatFromEEPROM(int addr) {
  float val;
  EEPROM.get(addr, val);
  return val;
}

void loadMemorySlots() {
  memSlots[0] = loadFloatFromEEPROM(EEPROM_M1_ADDR);
  memSlots[1] = loadFloatFromEEPROM(EEPROM_M2_ADDR);
  memSlots[2] = loadFloatFromEEPROM(EEPROM_M3_ADDR);

  Serial.println("[Memory] Loaded from EEPROM:");
  for (int i = 0; i < 3; i++) {
    Serial.print("  M");
    Serial.print(i + 1);
    Serial.print(": ");
    if (isnan(memSlots[i])) {
      Serial.println("EMPTY");
    } else {
      Serial.print(memSlots[i]);
      Serial.println(" cm");
    }
  }

  // If EEPROM is fresh (0xFF), values might be NAN or garbage.
  // Ideally check for valid range or separate flag, but NAN check usually works
  // if float format allows.
}

void updateDisplay(float dist, MotorState state) {
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
      // Force refresh of normal display next time
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

    if (isnan(dist))
      u8g2.print(F("ERR"));
    else {
      u8g2.print(dist, 1);
      u8g2.print(F(" cm"));
    }

    // Slot indicator
    u8g2.setCursor(80, 12);
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
      u8g2.print(F("DN"));
    else
      u8g2.print(F("STOP"));

    // Heartbeat pixel
    if (heartbeat)
      u8g2.drawPixel(127, 0);

  } while (u8g2.nextPage());
}

// ======================== SETUP =========================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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

  Serial.println("[System] Loading memory slots...");
  loadMemorySlots();

  // LED Initialization Sequence
  Serial.println("[System] LED initialization sequence started");
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  Serial.println("[System] LED initialization complete");

  Serial.println("[System] Ready");
}

// ======================== LOOP =========================
void loop() {
  unsigned long now = millis();

  // Distance reading
  float distRaw =
      (motorState == STOPPED) ? readUltrasonicMedian() : readUltrasonicFast();

  if (isnan(distRaw)) {
    Serial.println("[Sensor] WARNING: Invalid reading (NAN)");
    showError("Sensor Error");
    // Removed early return to allow updateDisplay to run
  } else {
    currentDistance = distRaw;
  }

  // Auto-STOP for limits
  if (currentDistance >= MAX_DISTANCE_CM && motorState == MOVING_UP) {
    Serial.print("[Limit] MAX reached (");
    Serial.print(MAX_DISTANCE_CM);
    Serial.println(" cm) - stopping motor");
    stopMotor();
  }

  if (currentDistance <= MIN_DISTANCE_CM && motorState == MOVING_DOWN) {
    Serial.print("[Limit] MIN reached (");
    Serial.print(MIN_DISTANCE_CM);
    Serial.println(" cm) - stopping motor");
    stopMotor();
  }

  // Display refresh
  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    updateDisplay(currentDistance, motorState);
    lastDisplayUpdate = now;
  }

  // ---------------- MEMORY BUTTON LOGIC ----------------
  bool memPressed = !digitalRead(BTN_MEMORY_PIN);

  if (memPressed && memButtonDownTime == 0) {
    Serial.println("[Button] MEMORY button pressed");
    memButtonDownTime = now;
    memButtonWasHeld = false;
  }

  if (!memPressed && memButtonDownTime != 0) {
    unsigned long pressDuration = now - memButtonDownTime;

    if (!memButtonWasHeld) {
      // Short press → cycle
      selectedSlot++;
      if (selectedSlot > 2)
        selectedSlot = -1;

      Serial.print("[Memory] Slot cycled to: ");
      if (selectedSlot == -1) {
        Serial.println("OFF");
      } else {
        Serial.print("M");
        Serial.println(selectedSlot + 1);
      }
    }

    memButtonDownTime = 0;
  }

  if (memPressed && now - memButtonDownTime >= MEMORY_LONG_PRESS &&
      !memButtonWasHeld) {
    memButtonWasHeld = true;
    Serial.println("[Button] MEMORY long press detected");

    if (selectedSlot == -1) {
      Serial.println("[Memory] ERROR: No slot selected");
      showError("Select M1-M3");
    } else {
      if (isnan(memSlots[selectedSlot])) {
        memSlots[selectedSlot] = currentDistance;
        saveFloatToEEPROM(EEPROM_M1_ADDR + selectedSlot * 10,
                          memSlots[selectedSlot]);
        Serial.print("[Memory] SAVED M");
        Serial.print(selectedSlot + 1);
        Serial.print(" = ");
        Serial.print(currentDistance);
        Serial.println(" cm");
        showError("Saved");
      } else {
        memSlots[selectedSlot] = NAN;
        saveFloatToEEPROM(EEPROM_M1_ADDR + selectedSlot * 10,
                          memSlots[selectedSlot]);
        Serial.print("[Memory] ERASED M");
        Serial.println(selectedSlot + 1);
        showError("Erased");
      }
    }
  }

  // ---------------- AUTO-DETECT MEMORY ----------------
  bool nearAnyMemory = false;
  static bool wasNearMemory = false;

  for (int i = 0; i < 3; i++) {
    if (!isnan(memSlots[i])) {
      if (abs(currentDistance - memSlots[i]) <= MEMORY_TOLERANCE_CM) {
        nearAnyMemory = true;
        if (!wasNearMemory) {
          Serial.print("[Memory] Near M");
          Serial.print(i + 1);
          Serial.print(" position (");
          Serial.print(memSlots[i]);
          Serial.println(" cm)");
        }
      }
    }
  }

  if (nearAnyMemory) {
    if (motorState != STOPPED && !ignoreMemoryStop) {
      Serial.println("[Memory] Auto-stop at memory position");
      stopMotor();
    }
  } else {
    if (wasNearMemory) {
      Serial.println("[Memory] Left memory zone");
    }
    ignoreMemoryStop = false; // Re-arm detection once we leave the zone
  }

  wasNearMemory = nearAnyMemory;

  static bool lastYellowState = false;
  bool newYellowState = nearAnyMemory;
  if (newYellowState != lastYellowState) {
    Serial.print("[LED] YELLOW: ");
    Serial.println(newYellowState ? "ON" : "OFF");
    lastYellowState = newYellowState;
  }
  digitalWrite(LED_YELLOW_PIN, nearAnyMemory ? HIGH : LOW);

  // ---------------- BUTTON INTERRUPTS ----------------
  if (upPressedISR) {
    upPressedISR = false;
    Serial.println("[Button] UP button action");
    if (motorState == STOPPED && currentDistance < MAX_DISTANCE_CM) {
      Serial.println("[Button] UP - Starting motor");
      ignoreMemoryStop = true; // Allow moving away from current spot
      beginMotorMove(MOVING_UP);
    } else {
      if (motorState != STOPPED) {
        Serial.println("[Button] UP - Stopping motor (toggle)");
      } else {
        Serial.println("[Button] UP - Already at max limit");
      }
      stopMotor();
    }
  }

  if (downPressedISR) {
    downPressedISR = false;
    Serial.println("[Button] DOWN button action");
    if (motorState == STOPPED && currentDistance > MIN_DISTANCE_CM) {
      Serial.println("[Button] DOWN - Starting motor");
      ignoreMemoryStop = true; // Allow moving away from current spot
      beginMotorMove(MOVING_DOWN);
    } else {
      if (motorState != STOPPED) {
        Serial.println("[Button] DOWN - Stopping motor (toggle)");
      } else {
        Serial.println("[Button] DOWN - Already at min limit");
      }
      stopMotor();
    }
  }

  // Non-blocking motor start
  applyMotorStartIfReady();

  // LED blinking while moving
  if (motorState != STOPPED) {
    if (now - lastBlinkTime >= LED_BLINK_INTERVAL) {
      greenLedState = !greenLedState;
      digitalWrite(LED_GREEN_PIN, greenLedState);
      lastBlinkTime = now;
    }
  } else {
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}
