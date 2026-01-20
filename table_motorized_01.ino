
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

// --- PIN DEFINITIONS ---
// HC-SR04
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

// L298N
const int MOTOR_DIR_A_PIN = 4;
const int MOTOR_DIR_B_PIN = 6;
const int MOTOR_PWM_PIN = 5;  // future PWM use

// Buttons (PCINT interrupts)
const int BTN_UP_PIN = 11;
const int BTN_DOWN_PIN = 12;
const int BTN_MEMORY_PIN = 10;

// LEDs
const int LED_RED_PIN = 9;
const int LED_GREEN_PIN = 8;
// Yellow LED reserved

// OLED (Hardware I2C on A4/A5)
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

// --- CONTROL PARAMETERS ---
const int EEPROM_DISTANCE_ADDR = 0;
const float MIN_DISTANCE_CM = 67.0;
const float MAX_DISTANCE_CM = 111.0;
const unsigned long DISPLAY_REFRESH_INTERVAL = 300;
const unsigned long LED_BLINK_INTERVAL = 200;
const unsigned long RESET_HOLD_TIME = 3000;

// --- STATE ---
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

// --- INTERRUPT FLAGS ---
volatile bool upPressedISR = false;
volatile bool downPressedISR = false;

// --- NON-BLOCKING MOTOR START ---
bool pendingMotorStart = false;
MotorState requestedMotorState = STOPPED;
unsigned long motorStartRequestTime = 0;

// ============================================================================
// INTERRUPT SETUP
// ============================================================================
void setupPCINT() {
  // Enable PCINT0 group (pins D8–D13)
  PCICR |= (1 << PCIE0);

  // Enable interrupts for pins 10, 11, 12
  PCMSK0 |= (1 << PCINT2);  // pin 10
  PCMSK0 |= (1 << PCINT3);  // pin 11
  PCMSK0 |= (1 << PCINT4);  // pin 12
}

ISR(PCINT0_vect) {
  static unsigned long lastInterrupt = 0;
  unsigned long now = millis();
  if (now - lastInterrupt < 30) return; // debounce

  if (!digitalRead(BTN_UP_PIN))   upPressedISR = true;
  if (!digitalRead(BTN_DOWN_PIN)) downPressedISR = true;

  lastInterrupt = now;
}

// ============================================================================
// ULTRASONIC SENSOR
// ============================================================================

// Fast read (used while motor is moving)
float readUltrasonicFast() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (duration <= 0) return currentDistance;

  return duration * 0.0343 / 2.0;
}

// Median filtered (used when stopped)
float readUltrasonicMedian() {
  const int samples = 5;
  float arr[samples];

  for (int i = 0; i < samples; i++) {
    arr[i] = readUltrasonicFast();
    delay(8);
  }

  // Insertion sort
  for (int i = 1; i < samples; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }

  return arr[samples/2];
}

// ============================================================================
// EEPROM
// ============================================================================
void saveDistanceToEEPROM(float distance) {
  if (abs(distance - previousDistance) < 0.5) return;
 ) {
  requestedMotorState = state;
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

void stopAndSaveState(bool save) {
  if (motorState == STOPPED) return;

  motorState = STOPPED;
  digitalWrite(MOTOR_DIR_A_PIN, LOW);
  digitalWrite(MOTOR_DIR_B_PIN, LOW);

  if (save) {
    currentDistance = readUltrasonicMedian();
    saveDistanceToEEPROM(currentDistance);
  }
}

// ============================================================================
// DISPLAY
// ============================================================================
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
    u8g2.print(F("Table height:"));

    u8g2.setCursor(0, 28);
    u8g2.print(dist, 1);
    u8g2.print(F(" cm"));

    u8g2.setCursor(95, 28);
    if (state == MOVING_UP) u8g2.print(F("UP"));
    else if (state == MOVING_DOWN) u8g2.print(F("DN"));
    else u8g2.print(F("STP"));

    if (heartbeat) u8g2.drawPixel(127, 0);

  } while (u8g2.nextPage());
}

// ============================================================================
// SETUP
// ============================================================================
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

  Serial.begin(115200);
  setupPCINT();

  u8g2.begin();
  u8g2.setContrast(255);

  currentDistance = readUltrasonicMedian();

  float stored = loadDistanceFromEEPROM();
  if (isnan(stored)) {
    saveDistanceToEEPROM(currentDistance);
  } else {
    previousDistance = stored;
  }

  Serial.println(F("[System] Ready."));
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  // Update distance
  if (motorState == STOPPED)
    currentDistance = readUltrasonicMedian();
  else
    currentDistance = readUltrasonicFast();

  // Display
  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    updateDisplay(currentDistance, motorState);
    lastDisplayUpdate = now;
  }

  // Dual-button EEPROM reset
  bool upPressed = !digitalRead(BTN_UP_PIN);
  bool downPressed = !digitalRead(BTN_DOWN_PIN);

  if (upPressed && downPressed) {
    if (!dualButtonActive) {
      dualButtonActive = true;
      dualButtonStartTime = now;
      stopAndSaveState(false);
    } else if (now - dualButtonStartTime > RESET_HOLD_TIME) {
      currentDistance = readUltrasonicMedian();
      saveDistanceToEEPROM(currentDistance);
      Serial.println(F("[System] EEPROM Calibrated."));
      dualButtonActive = false;
      delay(400);
    }
    return;
  } else {
    dualButtonActive = false;
  }

  // Interrupt-driven buttons
  if (upPressedISR) {
    upPressedISR = false;
    if (motorState == STOPPED && currentDistance < MAX_DISTANCE_CM)
      beginMotorMove(MOVING_UP);
    else
      stopAndSaveState(true);
  }

  if (downPressedISR) {
    downPressedISR = false;
    if (motorState == STOPPED && currentDistance > MIN_DISTANCE_CM)
      beginMotorMove(MOVING_DOWN);
    else
      stopAndSaveState(true);
  }

  // Motor delayed start
  applyMotorStartIfReady();

  // Limit protection
  if (motorState == MOVING_UP && currentDistance >= MAX_DISTANCE_CM)
    stopAndSaveState(true);

  if (motorState == MOVING_DOWN && currentDistance <= MIN_DISTANCE_CM)
    stopAndSaveState(true);

  // LED blinking when moving
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
