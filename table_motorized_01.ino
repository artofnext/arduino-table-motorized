
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// ----------------- PIN DEFINITIONS -----------------
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

const int MOTOR_DIR_A_PIN = 4;
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
  if (now - lastInterrupt < 30) return;

  if (!digitalRead(BTN_UP_PIN)) upPressedISR = true;
  if (!digitalRead(BTN_DOWN_PIN)) downPressedISR = true;

  lastInterrupt = now;
}

// ======================== ULTRASONIC =========================
float readUltrasonicFast() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration <= 0) return NAN;

  return duration * 0.0343 / 2.0;
}

float readUltrasonicMedian() {
  float arr[5];
  for (int i = 0; i < 5; i++) {
   FloatFromEEPROM(EEPROM_M2_ADDR);
  memSlots[2] = loadFloatFromEEPROM(EEPROM_M3_ADDR);
}

// ======================== MOTOR CONTROL =========================
void beginMotorMove(MotorState s) {
  requestedMotorState = s;
  motorStartRequestTime = millis();
  pendingMotorStart = true;
}

void applyMotorStartIfReady() {
  if (!pendingMotorStart) return;
  if (millis() - motorStartRequestTime < 500) return;

  if (requestedMotorState == MOVING_UP) {
    digitalWrite(MOTOR_DIR_A_PIN, HIGH);
    digitalWrite(MOTOR_DIR_B_PIN, LOW);
  } else if (requestedMotorState == MOVING_DOWN) {
    digitalWrite(MOTOR_DIR_A_PIN, LOW);
    digitalWrite(MOTOR_DIR_B_PIN, HIGH);
  }

  motorState = requestedMotorState;
  pendingMotorStart = false;
}

void stopMotor() {
  motorState = STOPPED;
  digitalWrite(MOTOR_DIR_A_PIN, LOW);
  digitalWrite(MOTOR_DIR_B_PIN, LOW);
}

// ======================== DISPLAY =========================
void showError(const char* message) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.drawStr(0, 16, message);
  } while (u8g2.nextPage());
  delay(1500);
}

void updateDisplay(float dist, MotorState state) {
  if (abs(dist - previousDisplayedDistance) < 0.1 &&
      state == lastDisplayedState) return;

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
    if (selectedSlot == -1) u8g2.print(F("[--]"));
    else u8g2.print(selectedSlot == 0 ? "[M1]" :
                    selectedSlot == 1 ? "[M2]" :
                                        "[M3]");

    // Movement
    u8g2.setCursor(0, 28);
    if (state == MOVING_UP) u8g2.print(F("UP"));
    else if (state == MOVING_DOWN) u8g2.print(F("DN"));
    else u8g2.print(F("STOP"));

    // Heartbeat pixel
    if (heartbeat) u8g2.drawPixel(127, 0);

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

  Serial.begin(115200);
  setupPCINT();
  u8g2.begin();

  loadMemorySlots();

  Serial.println("[System] Ready");
}

// ======================== LOOP =========================
void loop() {
  unsigned long now = millis();

  // Distance reading
  float distRaw = (motorState == STOPPED)
                    ? readUltrasonicMedian()
                    : readUltrasonicFast();

  if (isnan(distRaw)) {
    showError("Sensor Error");
    return;
  }

  currentDistance = distRaw;

  // Auto-STOP for limits
  if (currentDistance >= MAX_DISTANCE_CM && motorState == MOVING_UP)
    stopMotor();

  if (currentDistance <= MIN_DISTANCE_CM && motorState == MOVING_DOWN)
    stopMotor();

  // Display refresh
  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    updateDisplay(currentDistance, motorState);
    lastDisplayUpdate = now;
  }

  // ---------------- MEMORY BUTTON LOGIC ----------------
  bool memPressed = !digitalRead(BTN_MEMORY_PIN);

  if (memPressed && memButtonDownTime == 0) {
    memButtonDownTime = now;
    memButtonWasHeld = false;
  }

  if (!memPressed && memButtonDownTime != 0) {
    unsigned long pressDuration = now - memButtonDownTime;

    if (!memButtonWasHeld) {
      // Short press → cycle
      selectedSlot++;
      if (selectedSlot > 2) selectedSlot = -1;

      Serial.print("Selected slot: ");
      Serial.println(selectedSlot);
    }

    memButtonDownTime = 0;
  }

  if (memPressed && now - memButtonDownTime >= MEMORY_LONG_PRESS && !memButtonWasHeld) {
    memButtonWasHeld = true;

    if (selectedSlot == -1) {
      showError("Select M1-M3");
    } else {
      if (isnan(memSlots[selectedSlot])) {
        memSlots[selectedSlot] = currentDistance;
        saveFloatToEEPROM(EEPROM_M1_ADDR + selectedSlot * 10, memSlots[selectedSlot]);
        showError("Saved");
      } else {
        memSlots[selectedSlot] = NAN;
        saveFloatToEEPROM(EEPROM_M1_ADDR + selectedSlot * 10, memSlots[selectedSlot]);
        showError("Erased");
      }
    }
  }

  // ---------------- AUTO-DETECT MEMORY ----------------
  bool nearMemory = false;

  for (int i = 0; i < 3; i++) {
    if (!isnan(memSlots[i])) {
      if (abs(currentDistance - memSlots[i]) <= MEMORY_TOLERANCE_CM) {
        nearMemory = true;
        if (motorState != STOPPED) stopMotor();
      }
    }
  }

  digitalWrite(LED_YELLOW_PIN, nearMemory ? HIGH : LOW);

  // ---------------- BUTTON INTERRUPTS ----------------
  if (upPressedISR) {
    upPressedISR = false;
    if (motorState == STOPPED && currentDistance < MAX_DISTANCE_CM)
      beginMotorMove(MOVING_UP);
    else
      stopMotor();
  }

  if (downPressedISR) {
    downPressedISR = false;
    if (motorState == STOPPED && currentDistance > MIN_DISTANCE_CM)
      beginMotorMove(MOVING_DOWN);
    else
      stopMotor();
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
