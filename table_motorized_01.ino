#include <EEPROM.h>
#include <U8g2lib.h>

// --- PIN DEFINITIONS ---

// HC-SR04 Ultrasonic Sensor
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

// L298N Motor Driver
const int MOTOR_PWM_PIN = 5;
const int MOTOR_DIR_A_PIN = 4;
const int MOTOR_DIR_B_PIN = 6;

// Control Buttons
const int BTN_UP_PIN = 11;
const int BTN_DOWN_PIN = 12;
const int BTN_MEMORY_PIN = 10;

// Status LEDs
const int LED_RED_PIN = 9;
const int LED_GREEN_PIN = 8;
const int LED_YELLOW_PIN = 7;

// OLED Display (32 * 128) - Software I2C on A0/A1
const int OLED_SDA_PIN = A0;
const int OLED_SCK_PIN = A1;

// Using Page Buffer (_1_) for RAM efficiency (Important for Uno/Nano)
U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, OLED_SCK_PIN, OLED_SDA_PIN,
                                            U8X8_PIN_NONE);

// --- CONTROL PARAMETERS ---
const int EEPROM_DISTANCE_ADDR = 0;
const float DISTANCE_TOLERANCE_CM = 2.0;
const float MIN_DISTANCE_CM = 67.0;
const float MAX_DISTANCE_CM = 111.0;
const long DEBOUNCE_DELAY = 50;
const long LED_BLINK_INTERVAL = 200;
const unsigned long RESET_HOLD_TIME = 3000;
const unsigned long DISPLAY_REFRESH_INTERVAL = 500;

// --- STATE MANAGEMENT ---
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN };
MotorState motorState = STOPPED;

float currentDistance = 0.0;
float previousDistance = 0.0;
unsigned long lastButtonTime = 0;
unsigned long lastBlinkTime = 0;
bool greenLedState = false;
unsigned long lastDisplayUpdate = 0;
unsigned long dualButtonStartTime = 0;
bool dualButtonActive = false;

// --- DISPLAY FUNCTIONS ---

void updateDisplay(float distance, MotorState state) {
  static bool heartbeat = false;
  heartbeat = !heartbeat;

  uint8_t p = 0;
  u8g2.firstPage();
  do {
    p++;
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.setCursor(0, 12);
    u8g2.print(F("Table height:"));

    u8g2.setCursor(0, 28);
    if (isnan(distance) || distance < 1.0) {
      u8g2.print(F("---"));
    } else {
      u8g2.print(distance, 1);
      u8g2.print(F(" cm"));
    }

    u8g2.setCursor(95, 28);
    if (state == MOVING_UP)
      u8g2.print(F("UP"));
    else if (state == MOVING_DOWN)
      u8g2.print(F("DN"));
    else
      u8g2.print(F("STP"));

    // Heartbeat dot to show loop is alive
    if (heartbeat)
      u8g2.drawPixel(127, 0);

  } while (u8g2.nextPage());

  // Serial debug with page count
  Serial.print(F("[Display] Refreshing... Pages: "));
  Serial.print(p);
  Serial.print(F(", Dist: "));
  Serial.print(distance);
  Serial.println(F(" cm"));
}

// --- HELPER FUNCTIONS ---

float readUltrasonicFiltered() {
  const int samples = 5;
  float readings[samples];
  int vCount = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000UL); // ~4m timeout
    if (duration > 50) { // filter out immediate noise
      readings[vCount++] = duration * 0.0343 / 2.0;
    }
    delay(25);
  }

  if (vCount == 0)
    return currentDistance;

  // Sort
  for (int i = 0; i < vCount - 1; i++) {
    for (int j = 0; j < vCount - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        float tmp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = tmp;
      }
    }
  }
  return readings[vCount / 2];
}

void saveDistanceToEEPROM(float distance) {
  EEPROM.put(EEPROM_DISTANCE_ADDR, distance);
  Serial.print(F("[EEPROM] Saved: "));
  Serial.println(distance);
}

float loadDistanceFromEEPROM() {
  float stored;
  EEPROM.get(EEPROM_DISTANCE_ADDR, stored);
  if (isnan(stored) || stored < 10.0 || stored > 200.0)
    return NAN;
  return stored;
}

void setMotorState(MotorState state) {
  if (motorState == state)
    return;
  switch (state) {
  case MOVING_UP:
    delay(500);
    digitalWrite(MOTOR_DIR_A_PIN, HIGH);
    digitalWrite(MOTOR_DIR_B_PIN, LOW);
    break;
  case MOVING_DOWN:
    delay(500);
    digitalWrite(MOTOR_DIR_A_PIN, LOW);
    digitalWrite(MOTOR_DIR_B_PIN, HIGH);
    break;
  default:
    digitalWrite(MOTOR_DIR_A_PIN, LOW);
    digitalWrite(MOTOR_DIR_B_PIN, LOW);
    break;
  }
  motorState = state;
}

void stopAndSaveState(bool save) {
  if (motorState == STOPPED)
    return;
  setMotorState(STOPPED);
  if (save) {
    currentDistance = readUltrasonicFiltered();
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
  }
}

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

  // Internal Pullups for Software I2C
  pinMode(OLED_SDA_PIN, INPUT_PULLUP);
  pinMode(OLED_SCK_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(100);

  u8g2.begin();
  u8g2.setContrast(255);

  // Test Screen
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x13_tr);
    u8g2.drawStr(0, 15, "Initializing...");
  } while (u8g2.nextPage());

  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);

  currentDistance = readUltrasonicFiltered();
  float stored = loadDistanceFromEEPROM();
  if (isnan(stored)) {
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
  } else {
    previousDistance = stored;
  }

  Serial.println(F("[System] Ready."));
}

void loop() {
  bool upPressed = !digitalRead(BTN_UP_PIN);
  bool downPressed = !digitalRead(BTN_DOWN_PIN);
  unsigned long now = millis();

  // Periodic Update
  if (now - lastDisplayUpdate >= DISPLAY_REFRESH_INTERVAL) {
    currentDistance = readUltrasonicFiltered();
    updateDisplay(currentDistance, motorState);
    lastDisplayUpdate = now;
  }

  // EEPROM Reset
  if (upPressed && downPressed) {
    if (!dualButtonActive) {
      dualButtonActive = true;
      dualButtonStartTime = now;
      stopAndSaveState(false);
    } else if (now - dualButtonStartTime >= RESET_HOLD_TIME) {
      currentDistance = readUltrasonicFiltered();
      saveDistanceToEEPROM(currentDistance);
      previousDistance = currentDistance;
      Serial.println(F("[System] EEPROM Calibrated."));
      dualButtonActive = false;
      delay(500);
    }
    return;
  } else {
    dualButtonActive = false;
  }

  // Handle Buttons
  if ((upPressed || downPressed) && (now - lastButtonTime > DEBOUNCE_DELAY)) {
    lastButtonTime = now;
    if (motorState != STOPPED) {
      stopAndSaveState(true);
    } else {
      if (upPressed && currentDistance < MAX_DISTANCE_CM) {
        setMotorState(MOVING_UP);
      } else if (downPressed && currentDistance > MIN_DISTANCE_CM) {
        setMotorState(MOVING_DOWN);
      }
    }
  }

  // Monitor Motion
  if (motorState != STOPPED) {
    // blink green led
    if (now - lastBlinkTime > LED_BLINK_INTERVAL) {
      greenLedState = !greenLedState;
      digitalWrite(LED_GREEN_PIN, greenLedState);
      lastBlinkTime = now;
    }

    if (motorState == MOVING_UP && currentDistance >= MAX_DISTANCE_CM)
      stopAndSaveState(true);
    if (motorState == MOVING_DOWN && currentDistance <= MIN_DISTANCE_CM)
      stopAndSaveState(true);
  } else {
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}