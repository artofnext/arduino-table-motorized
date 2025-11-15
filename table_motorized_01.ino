#include <EEPROM.h>

// --- PIN DEFINITIONS ---

// HC-SR04 Ultrasonic Sensor
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;

// L298N Motor Driver (Parallel Mode)
const int MOTOR_PWM_PIN = 5; // PWM Pin
const int MOTOR_DIR_A_PIN = 4;
const int MOTOR_DIR_B_PIN = 6;

// Control Buttons (Active LOW with internal PULLUP)
const int BTN_UP_PIN = 11;
const int BTN_DOWN_PIN = 12;

// Status LEDs
const int LED_GREEN_PIN = 8; // Blinks when motor is moving
const int LED_RED_PIN = 9;   // Turns on when security error occurs

// --- EEPROM AND CONTROL PARAMETERS ---
const int EEPROM_DISTANCE_ADDR = 0;
const float DISTANCE_TOLERANCE_CM = 2.0; 

// Distance Limits (in centimeters)
const float MIN_DISTANCE_CM = 50.0;
const float MAX_DISTANCE_CM = 150.0;

// Motor Speed (0-255)
const int MOTOR_SPEED = 220;

// Button Debounce time (in milliseconds)
const long DEBOUNCE_DELAY = 50;

// Green LED Blink Timing
const long LED_BLINK_INTERVAL = 200; // Milliseconds for one state (200ms ON, 200ms OFF)

// Dual button press duration for EEPROM reset
const unsigned long RESET_HOLD_TIME = 3000; // 3 seconds

// --- STATE MANAGEMENT ---
enum MotorState {
  STOPPED,
  MOVING_UP,
  MOVING_DOWN
};

MotorState motorState = STOPPED;

// --- RUNTIME VARIABLES ---
float currentDistance = 0.0;
float previousDistance = 0.0;

unsigned long lastButtonTime = 0;
unsigned long lastBlinkTime = 0;
bool greenLedState = false;
unsigned long dualButtonStartTime = 0;
bool dualButtonActive = false;

// --- HELPER FUNCTIONS ---

float readUltrasonicFiltered() {
  const int samples = 5;
  float readings[samples];
  
  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30 ms
    readings[i] = duration * 0.0343 / 2.0; // cm
    delay(50);
  }

  // Sort readings
  for (int i = 0; i < samples - 1; i++) {
    for (int j = 0; j < samples - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        float tmp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = tmp;
      }
    }
  }

  float median = readings[samples / 2];
  Serial.print(F("[Ultrasonic] Median distance: "));
  Serial.print(median);
  Serial.println(F(" cm"));
  return median;
}

void saveDistanceToEEPROM(float distance) {
  EEPROM.put(EEPROM_DISTANCE_ADDR, distance);
  Serial.print(F("[EEPROM] Saved distance: "));
  Serial.print(distance);
  Serial.println(F(" cm"));
}

float loadDistanceFromEEPROM() {
  float stored;
  EEPROM.get(EEPROM_DISTANCE_ADDR, stored);
  if (isnan(stored) || stored < 10.0 || stored > 200.0) {
    Serial.println(F("[EEPROM] Invalid or empty data detected."));
    return NAN;
  }
  Serial.print(F("[EEPROM] Loaded distance: "));
  Serial.print(stored);
  Serial.println(F(" cm"));
  return stored;
}

void setMotorState(MotorState state) {
  if (motorState == state) return; // Avoid redundant state changes

  switch (state) {
    case MOVING_UP:
      digitalWrite(MOTOR_DIR_A_PIN, HIGH);
      digitalWrite(MOTOR_DIR_B_PIN, LOW);
      analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
      Serial.println(F("[MOTOR] Moving UP"));
      break;
    case MOVING_DOWN:
      digitalWrite(MOTOR_DIR_A_PIN, LOW);
      digitalWrite(MOTOR_DIR_B_PIN, HIGH);
      analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED);
      Serial.println(F("[MOTOR] Moving DOWN"));
      break;
    default:
      digitalWrite(MOTOR_DIR_A_PIN, LOW);
      digitalWrite(MOTOR_DIR_B_PIN, LOW);
      analogWrite(MOTOR_PWM_PIN, 0);
      Serial.println(F("[MOTOR] STOPPED"));
      break;
  }
  motorState = state;
}

void blinkGreenLed() {
  unsigned long now = millis();
  if (now - lastBlinkTime >= LED_BLINK_INTERVAL) {
    greenLedState = !greenLedState;
    digitalWrite(LED_GREEN_PIN, greenLedState);
    lastBlinkTime = now;
  }
}

void signalRedLed(unsigned long durationMs = 1000, const __FlashStringHelper* reason = nullptr) {
  if (reason) {
    Serial.print(F("[ALERT] "));
    Serial.println(reason);
  }
  digitalWrite(LED_RED_PIN, HIGH);
  delay(durationMs);
  digitalWrite(LED_RED_PIN, LOW);
}

// --- SETUP ---

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_A_PIN, OUTPUT);
  pinMode(MOTOR_DIR_B_PIN, OUTPUT);
  
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("\n=== Adjustable Table Controller Initialized ==="));
  Serial.println(F("[System] Power-on safety delay (2s)..."));

  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);

  currentDistance = readUltrasonicFiltered();
  float stored = loadDistanceFromEEPROM();
  if (isnan(stored)) {
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
    Serial.println(F("[EEPROM] Initialized with current distance."));
  } else {
    previousDistance = stored;
  }

  setMotorState(STOPPED);
  Serial.println(F("[System] Ready for input."));
}

// --- MAIN LOOP ---

void loop() {
  bool upPressed = !digitalRead(BTN_UP_PIN);
  bool downPressed = !digitalRead(BTN_DOWN_PIN);

  unsigned long now = millis();

  // --- Dual button EEPROM reset ---
  if (upPressed && downPressed) {
    if (!dualButtonActive) {
      dualButtonActive = true;
      dualButtonStartTime = now;
      Serial.println(F("[Reset] Both buttons pressed, waiting 3 seconds..."));
    } else if (now - dualButtonStartTime >= RESET_HOLD_TIME) {
      setMotorState(STOPPED);
      currentDistance = readUltrasonicFiltered();
      saveDistanceToEEPROM(currentDistance);
      previousDistance = currentDistance;
      Serial.println(F("[Reset] EEPROM value reset to current measurement."));
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_RED_PIN, HIGH);
        delay(200);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_RED_PIN, LOW);
        delay(200);
      }
      dualButtonActive = false;
      delay(500);
    }
    return;
  } else {
    dualButtonActive = false;
  }

  // --- Motor stop when buttons released ---
  if (motorState != STOPPED && !upPressed && !downPressed) {
    setMotorState(STOPPED);
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(200);
    currentDistance = readUltrasonicFiltered();
    saveDistanceToEEPROM(currentDistance);
    digitalWrite(LED_GREEN_PIN, LOW);
    previousDistance = currentDistance;
    Serial.println(F("[Motor] Movement stopped. Distance saved."));
    return;
  }

  // --- UP movement ---
  if (upPressed) {
    currentDistance = readUltrasonicFiltered();
    if (fabs(currentDistance - previousDistance) > DISTANCE_TOLERANCE_CM) {
      signalRedLed(1000, F("Untrusted ultrasonic reading (UP)."));
      return;
    }
    if (currentDistance >= MAX_DISTANCE_CM) {
      signalRedLed(1000, F("MAX limit reached. Cannot move UP."));
      return;
    }
    setMotorState(MOVING_UP);
    while (!digitalRead(BTN_UP_PIN)) {
      blinkGreenLed();
      currentDistance = readUltrasonicFiltered();
      if (currentDistance >= MAX_DISTANCE_CM) {
        setMotorState(STOPPED);
        signalRedLed(1000, F("MAX limit reached during UP motion."));
        break;
      }
    }
    setMotorState(STOPPED);
    digitalWrite(LED_GREEN_PIN, HIGH);
    currentDistance = readUltrasonicFiltered();
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
    digitalWrite(LED_GREEN_PIN, LOW);
    Serial.println(F("[UP] Movement complete."));
  } 
  // --- DOWN movement ---
  else if (downPressed) {
    currentDistance = readUltrasonicFiltered();
    if (fabs(currentDistance - previousDistance) > DISTANCE_TOLERANCE_CM) {
      signalRedLed(1000, F("Untrusted ultrasonic reading (DOWN)."));
      return;
    }
    if (currentDistance <= MIN_DISTANCE_CM) {
      signalRedLed(1000, F("MIN limit reached. Cannot move DOWN."));
      return;
    }
    setMotorState(MOVING_DOWN);
    while (!digitalRead(BTN_DOWN_PIN)) {
      blinkGreenLed();
      currentDistance = readUltrasonicFiltered();
      if (currentDistance <= MIN_DISTANCE_CM) {
        setMotorState(STOPPED);
        signalRedLed(1000, F("MIN limit reached during DOWN motion."));
        break;
      }
    }
    setMotorState(STOPPED);
    digitalWrite(LED_GREEN_PIN, HIGH);
    currentDistance = readUltrasonicFiltered();
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
    digitalWrite(LED_GREEN_PIN, LOW);
    Serial.println(F("[DOWN] Movement complete."));
  }
}
