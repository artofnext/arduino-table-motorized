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
const int BTN_MEMORY_PIN = 10;


// Status LEDs
const int LED_RED_PIN = 9;   // Turns on when security error occurs
const int LED_GREEN_PIN = 8; // Blinks when motor is moving
const int LED_YELLOW_PIN = 7;  // Memory indicator

// --- EEPROM AND CONTROL PARAMETERS ---
const int EEPROM_DISTANCE_ADDR = 0;
const float DISTANCE_TOLERANCE_CM = 2.0; 

// Distance Limits (in centimeters)
const float MIN_DISTANCE_CM = 67.0;
const float MAX_DISTANCE_CM = 111.0;

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
      delay(1000); // waiting a bit
      digitalWrite(MOTOR_DIR_A_PIN, HIGH);
      digitalWrite(MOTOR_DIR_B_PIN, LOW);
      Serial.println(F("[MOTOR] Moving UP"));
      break;
    case MOVING_DOWN:
      delay(1000); // waiting a bit
      digitalWrite(MOTOR_DIR_A_PIN, LOW);
      digitalWrite(MOTOR_DIR_B_PIN, HIGH);
      Serial.println(F("[MOTOR] Moving DOWN"));
      break;
    default:
      digitalWrite(MOTOR_DIR_A_PIN, LOW);
      digitalWrite(MOTOR_DIR_B_PIN, LOW);
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

// --- NEW HELPER FUNCTION for Motor Stop & State Save ---
void stopAndSaveState(bool saveToEEPROM) {
  if (motorState == STOPPED) return;
  
  setMotorState(STOPPED);
  digitalWrite(LED_GREEN_PIN, LOW); // Ensure LED is off immediately

  if (saveToEEPROM) {
    // Quick blink to confirm stop/save
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(100);
    digitalWrite(LED_GREEN_PIN, LOW);
    
    currentDistance = readUltrasonicFiltered();
    saveDistanceToEEPROM(currentDistance);
    previousDistance = currentDistance;
    Serial.println(F("[Motor] Movement stopped. Distance saved."));
  } else {
    Serial.println(F("[Motor] Movement stopped."));
  }
}

// --- SETUP ---

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_A_PIN, OUTPUT);
  pinMode(MOTOR_DIR_B_PIN, OUTPUT);
  
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);  
  pinMode(BTN_MEMORY_PIN, INPUT_PULLUP);
  
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("\n=== Adjustable Table Controller Initialized (Toggle Mode) ==="));
  Serial.println(F("[System] Power-on safety delay (2s)..."));

  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  // for (int i = 0; i < 20; i++) {
  //   digitalWrite(LED_GREEN_PIN, HIGH);
  //   delay(400);
  //   digitalWrite(LED_GREEN_PIN, LOW);
  //   delay(400);
  // }

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
      stopAndSaveState(false); // Stop motor immediately on dual press
    } else if (now - dualButtonStartTime >= RESET_HOLD_TIME) {
      stopAndSaveState(false);
      
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

  // --- Button Press Handling (Toggle/Stop Logic) ---
  // A single button press is either a start or a stop command.
  if ((upPressed || downPressed) && (now - lastButtonTime > DEBOUNCE_DELAY)) {
    lastButtonTime = now; // Update debounce timer for the next press

    // 1. If motor is currently MOVING, any button press is a STOP command
    if (motorState != STOPPED) {
        stopAndSaveState(true);
        return; 
    }

    // 2. If motor is STOPPED, check which button was pressed to START
    if (upPressed && !downPressed) {
      // UP START attempt
      currentDistance = readUltrasonicFiltered();
      if (fabs(currentDistance - previousDistance) > DISTANCE_TOLERANCE_CM) {
        signalRedLed(1000, F("Untrusted ultrasonic reading (UP start). Safety lock."));
        return;
      }
      if (currentDistance >= MAX_DISTANCE_CM) {
        signalRedLed(1000, F("MAX limit reached. Cannot start move UP."));
        return;
      }
      setMotorState(MOVING_UP);
    } 
    else if (downPressed && !upPressed) {
      // DOWN START attempt
      currentDistance = readUltrasonicFiltered();
      if (fabs(currentDistance - previousDistance) > DISTANCE_TOLERANCE_CM) {
        signalRedLed(1000, F("Untrusted ultrasonic reading (DOWN start). Safety lock."));
        return;
      }
      if (currentDistance <= MIN_DISTANCE_CM) {
        signalRedLed(1000, F("MIN limit reached. Cannot start move DOWN."));
        return;
      }
      setMotorState(MOVING_DOWN);
    }
  }

  // --- Continuous Movement Monitoring (The Non-Blocking Motion Loop) ---
  if (motorState != STOPPED) {
    blinkGreenLed();
    
    // Read and check distance frequently
    currentDistance = readUltrasonicFiltered();

    // Check for limits hit during motion
    if (motorState == MOVING_UP && currentDistance >= MAX_DISTANCE_CM) {
      signalRedLed(1000, F("MAX limit reached during UP motion. AUTO-STOP."));
      stopAndSaveState(true); 
      return;
    }
    
    if (motorState == MOVING_DOWN && currentDistance <= MIN_DISTANCE_CM) {
      signalRedLed(1000, F("MIN limit reached during DOWN motion. AUTO-STOP."));
      stopAndSaveState(true); 
      return;
    }
  } else {
    // If stopped, ensure green LED is permanently off
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}