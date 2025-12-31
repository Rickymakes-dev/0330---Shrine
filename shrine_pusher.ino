/*
 * CNC Shield Pusher Control System
 * X Axis: Horizontal (In/Out) - Limit at retracted (-X) - X port on CNC shield
 * Y Axis: Elevation (Up/Down) - Limit at bottom (-Y) - Y port on CNC shield
 * 
 * Physical wiring (directly matches CNC shield labels):
 * - Horizontal motor -> X port on CNC shield (X axis)
 * - Elevation motor -> Y port on CNC shield (Y axis)
 * 
 * Safety Order:
 * - Retraction: X (horizontal) retracts first, then Y (elevation) lowers
 * - Extension: Y (elevation) raises first, then X (horizontal) extends
 */

// Pin Definitions for CNC Shield V3
// X Axis (Horizontal) - X port on CNC shield
#define X_STEP_PIN         2
#define X_DIR_PIN          5
#define X_ENABLE_PIN       8

// Y Axis (Elevation) - Y port on CNC shield
#define Y_STEP_PIN         3
#define Y_DIR_PIN          6
#define Y_ENABLE_PIN       8

// Limit switches
#define X_LIMIT_PIN        9   // -X limit (horizontal retracted position)
#define Y_LIMIT_PIN        10  // -Y limit (elevation bottom position)

#define COOL_EN_PIN        A3  // Cycle trigger switch (CoolEn/SpnEn pin)
#define E_STOP_PIN         A0  // Emergency stop (Abort pin)

// Configuration
const bool TEST_MODE = false;        // Set to true to enable test mode
const bool AUTO_HOMING = true;     // Set to false to disable auto-homing on startup

// Movement parameters (adjust these for your setup)
const long STEPS_PER_MM = 80;          // Steps per millimeter (adjust for your motors)
const long X_MAX_TRAVEL_MM = 81;       // Maximum X (horizontal) travel in mm
const long Y_MAX_TRAVEL_MM = 43;       // Maximum Y (elevation) travel in mm

// Speed configuration (in mm/s)
const float HOMING_SPEED_MM_S = 10.0;  // Homing speed in mm/s (slower for safety)
const float NORMAL_SPEED_MM_S = 50.0;  // Normal movement speed in mm/s
const float TEST_SPEED_MM_S = 30.0;    // Test mode speed in mm/s

// Calculated delays (microseconds between steps)
// At 80 steps/mm: 5mm/s=2500us, 8mm/s=1562us, 10mm/s=1250us
// Note: If motors skip steps (inconsistent distance), increase these delays
const unsigned int HOMING_SPEED_DELAY = 2000;   // 6mm/s (very slow, safe)
const unsigned int NORMAL_SPEED_DELAY = 1500;   // 8mm/s (slow for accuracy)
const unsigned int TEST_SPEED_DELAY = 1500;     // 8mm/s (slow for accuracy)
const unsigned int START_SPEED_DELAY = 3000;    // Starting delay for acceleration ramp
const unsigned int ACCEL_STEPS = 200;           // Steps to accelerate/decelerate

const int DEBOUNCE_DELAY = 50;         // Debounce time in milliseconds
const unsigned long MOTOR_IDLE_TIMEOUT = 2000;  // Disable motors after 2 seconds idle

// State variables
enum SystemState {
  STATE_INIT,
  STATE_HOMING,
  STATE_READY,
  STATE_EXTENDING,
  STATE_RETRACTING,
  STATE_STOPPED,
  STATE_TEST_MODE
};

SystemState currentState = STATE_INIT;
bool isHomed = false;
bool coolEnLastState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long lastMotorActivity = 0;  // Track when motors last moved
bool motorsEnabled = false;

// Direction definitions (swap HIGH/LOW if motors move wrong direction)
#define DIR_X_IN HIGH     // X (horizontal): retract toward limit
#define DIR_X_OUT LOW     // X (horizontal): extend away from limit
#define DIR_Y_DOWN HIGH   // Y (elevation): down toward limit
#define DIR_Y_UP LOW      // Y (elevation): up away from limit

// Enable motors (call before any movement)
void enableMotors() {
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(X_ENABLE_PIN, LOW);
  motorsEnabled = true;
  lastMotorActivity = millis();
  delay(10);  // Short delay for driver to wake up
}

// Disable motors (reduces heat when idle)
void disableMotors() {
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(X_ENABLE_PIN, HIGH);
  motorsEnabled = false;
  Serial.println(F("Motors disabled (idle)"));
}

// Check if motors should be disabled due to idle timeout
void checkMotorIdle() {
  if (motorsEnabled && (millis() - lastMotorActivity > MOTOR_IDLE_TIMEOUT)) {
    disableMotors();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("CNC Pusher Control"));
  Serial.println(F("X=Horizontal, Y=Elevation"));
  
  // Configure step and direction pins
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  
  // Configure limit switches (with internal pullup)
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  
  // Configure control switches
  pinMode(COOL_EN_PIN, INPUT_PULLUP);
  pinMode(E_STOP_PIN, INPUT_PULLUP);
  
  // Start with motors disabled (will enable when needed)
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(X_ENABLE_PIN, HIGH);
  motorsEnabled = false;
  
  delay(100);
  
  // Check if in test mode
  if (TEST_MODE) {
    currentState = STATE_TEST_MODE;
    testMode();
    return;
  }
  
  // Perform auto-homing if enabled
  if (AUTO_HOMING) {
    Serial.println(F("Auto-homing..."));
    currentState = STATE_HOMING;
    performHoming();
  } else {
    Serial.println(F("Ready. Waiting for trigger..."));
    currentState = STATE_READY;
    isHomed = false;
  }
}

void loop() {
  // Check E-Stop first
  if (digitalRead(E_STOP_PIN) == LOW) {
    emergencyStop();
    return;
  }
  
  // Check if motors should be disabled due to idle
  checkMotorIdle();
  
  // Debug: Print button state every second
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 1000) {
    lastDebugPrint = millis();
    Serial.print(F("Btn:"));
    Serial.print(digitalRead(COOL_EN_PIN) == LOW ? "PRESSED" : "released");
    Serial.print(F(" State:"));
    Serial.println(currentState);
  }
  
  // Handle states
  switch (currentState) {
    case STATE_READY:
      // Wait for coolEn switch press (button goes LOW when pressed)
      if (digitalRead(COOL_EN_PIN) == LOW) {
        delay(50);  // Simple debounce
        if (digitalRead(COOL_EN_PIN) == LOW) {  // Still pressed
          Serial.println(F("Button pressed!"));
          // Wait for release
          while (digitalRead(COOL_EN_PIN) == LOW) delay(10);
          
          if (!isHomed) {
            Serial.println(F("Not homed. Homing..."));
            currentState = STATE_HOMING;
            performHoming();
          } else {
            Serial.println(F("Extending..."));
            currentState = STATE_EXTENDING;
            extendSequence();
          }
        }
      }
      break;
      
    case STATE_EXTENDING:
      // After extension, wait for next press to retract
      currentState = STATE_READY;
      break;
      
    case STATE_STOPPED:
      // Do nothing, system is stopped
      break;
      
    default:
      break;
  }
  
  delay(10);
}

bool checkSwitchPress(int pin) {
  bool currentReading = digitalRead(pin);
  
  if (currentReading != coolEnLastState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentReading == LOW && coolEnLastState == HIGH) {
      coolEnLastState = currentReading;
      return true;
    }
  }
  
  coolEnLastState = currentReading;
  return false;
}

void performHoming() {
  Serial.println(F("=== HOMING ==="));
  enableMotors();  // Enable motors before movement
  
  // Step 1: Home X (horizontal) first - retract
  Serial.println(F("Homing X (horizontal)..."));
  if (!homeAxis(X_STEP_PIN, X_DIR_PIN, X_LIMIT_PIN, DIR_X_IN, 'X')) {
    Serial.println(F("ERR: X home fail"));
    emergencyStop();
    return;
  }
  
  // Step 2: Home Y (elevation) - lower
  Serial.println(F("Homing Y (elevation)..."));
  if (!homeAxis(Y_STEP_PIN, Y_DIR_PIN, Y_LIMIT_PIN, DIR_Y_DOWN, 'Y')) {
    Serial.println(F("ERR: Y home fail"));
    emergencyStop();
    return;
  }
  
  isHomed = true;
  currentState = STATE_READY;
  Serial.println(F("=== HOMED ==="));
}

bool homeAxis(int stepPin, int dirPin, int limitPin, int direction, char axisName) {
  // If already at limit, consider it homed (no movement needed)
  if (digitalRead(limitPin) == LOW) {
    Serial.print(axisName);
    Serial.println(F(" already at limit - homed"));
    return true;
  }
  
  // Not at limit, need to move toward it
  digitalWrite(dirPin, direction);
  delay(50);
  
  long stepCount = 0;
  unsigned int currentDelay;
  
  while (digitalRead(limitPin) == HIGH) {
    if (checkEStop()) return false;
    // Accelerate at start of homing move
    if (stepCount < ACCEL_STEPS) {
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - HOMING_SPEED_DELAY) * stepCount / ACCEL_STEPS);
    } else {
      currentDelay = HOMING_SPEED_DELAY;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
    stepCount++;
  }
  
  Serial.print(axisName);
  Serial.println(F(" homed"));
  return true;
}

void extendSequence() {
  Serial.println(F("=== EXTEND ==="));
  
  // Step 1: Y (elevation) up first
  Serial.println(F("Y up..."));
  moveAxis(Y_STEP_PIN, Y_DIR_PIN, DIR_Y_UP, Y_MAX_TRAVEL_MM * STEPS_PER_MM);
  
  // Step 2: X (horizontal) out
  Serial.println(F("X out..."));
  moveAxis(X_STEP_PIN, X_DIR_PIN, DIR_X_OUT, X_MAX_TRAVEL_MM * STEPS_PER_MM);
  
  Serial.println(F("=== EXTENDED ==="));
  
  waitForButtonPress();
  retractSequence();
}

void retractSequence() {
  Serial.println(F("=== RETRACT ==="));
  
  // Step 1: X (horizontal) in first
  Serial.println(F("X in..."));
  if (!moveAxisToLimit(X_STEP_PIN, X_DIR_PIN, X_LIMIT_PIN, DIR_X_IN, 'X')) {
    emergencyStop();
    return;
  }
  
  // Step 2: Y (elevation) down
  Serial.println(F("Y down..."));
  if (!moveAxisToLimit(Y_STEP_PIN, Y_DIR_PIN, Y_LIMIT_PIN, DIR_Y_DOWN, 'Y')) {
    emergencyStop();
    return;
  }
  
  Serial.println(F("=== RETRACTED ==="));
  currentState = STATE_READY;
}

void moveAxis(int stepPin, int dirPin, int direction, long steps) {
  enableMotors();  // Ensure motors are enabled
  digitalWrite(dirPin, direction);
  delay(50);
  
  Serial.print(F("Moving ")); Serial.print(steps); Serial.println(F(" steps"));
  
  unsigned int currentDelay;
  
  for (long i = 0; i < steps; i++) {
    if (checkEStop()) return;
    
    // Acceleration ramp
    if (i < ACCEL_STEPS) {
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - NORMAL_SPEED_DELAY) * i / ACCEL_STEPS);
    } else if (i > steps - ACCEL_STEPS) {
      long stepsFromEnd = steps - i;
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - NORMAL_SPEED_DELAY) * stepsFromEnd / ACCEL_STEPS);
    } else {
      currentDelay = NORMAL_SPEED_DELAY;
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
    if (i % 800 == 0) Serial.print('.');
  }
  lastMotorActivity = millis();  // Update activity time
  Serial.println(F(" Done"));
}

bool moveAxisToLimit(int stepPin, int dirPin, int limitPin, int direction, char axisName) {
  enableMotors();  // Ensure motors are enabled
  digitalWrite(dirPin, direction);
  delay(50);
  
  long stepCount = 0;
  long maxSteps = (axisName == 'Y' ? Y_MAX_TRAVEL_MM : X_MAX_TRAVEL_MM) * STEPS_PER_MM * 1.2;
  
  Serial.print(F("Moving to limit, max ")); Serial.print(maxSteps); Serial.println(F(" steps"));
  
  unsigned int currentDelay;
  
  while (digitalRead(limitPin) == HIGH && stepCount < maxSteps) {
    if (checkEStop()) return false;
    
    // Accelerate at start
    if (stepCount < ACCEL_STEPS) {
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - NORMAL_SPEED_DELAY) * stepCount / ACCEL_STEPS);
    } else {
      currentDelay = NORMAL_SPEED_DELAY;
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
    stepCount++;
    if (stepCount % 800 == 0) Serial.print('.');
  }
  Serial.println();
  
  if (stepCount >= maxSteps) {
    Serial.print(axisName);
    Serial.println(F(" no limit!"));
    return false;
  }
  lastMotorActivity = millis();  // Update activity time
  Serial.print(axisName); Serial.print(F(" limit reached after ")); Serial.print(stepCount); Serial.println(F(" steps"));
  return true;
}

void waitForButtonPress() {
  Serial.println(F("Press btn to retract..."));
  // Wait for button release first (if pressed)
  while (digitalRead(COOL_EN_PIN) == LOW) delay(10);
  delay(100);  // Debounce
  
  // Now wait for press
  while (true) {
    if (checkEStop()) return;
    if (digitalRead(COOL_EN_PIN) == LOW) {
      delay(50);  // Debounce
      if (digitalRead(COOL_EN_PIN) == LOW) {
        Serial.println(F("Button pressed - retracting"));
        // Wait for release
        while (digitalRead(COOL_EN_PIN) == LOW) delay(10);
        return;
      }
    }
    delay(10);
  }
}

bool checkEStop() {
  if (digitalRead(E_STOP_PIN) == LOW) {
    emergencyStop();
    return true;
  }
  return false;
}

void emergencyStop() {
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(X_ENABLE_PIN, HIGH);
  currentState = STATE_STOPPED;
  isHomed = false;
  Serial.println(F("\n!!! E-STOP !!!"));
  while (true) delay(1000);
}

void stepMotor(int stepPin, long steps, long stepDelay) {
  for (long i = 0; i < steps; i++) {
    if (checkEStop()) return;
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);  // Longer HIGH pulse for reliability
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    if (i % 400 == 0) Serial.print('.');
  }
}

// Step function with acceleration ramp for reliable movement
void doSteps(int stepPin, long numSteps, unsigned int targetDelay) {
  enableMotors();  // Ensure motors are enabled
  Serial.print(F("Stepping ")); Serial.print(numSteps); Serial.println(F(" steps"));
  
  unsigned int currentDelay;
  
  for (long i = 0; i < numSteps; i++) {
    // Calculate delay with acceleration/deceleration ramp
    if (i < ACCEL_STEPS) {
      // Accelerating: start slow, speed up
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - targetDelay) * i / ACCEL_STEPS);
    } else if (i > numSteps - ACCEL_STEPS) {
      // Decelerating: slow down at end
      long stepsFromEnd = numSteps - i;
      currentDelay = START_SPEED_DELAY - ((START_SPEED_DELAY - targetDelay) * stepsFromEnd / ACCEL_STEPS);
    } else {
      // Cruising at target speed
      currentDelay = targetDelay;
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
    
    if (i % 400 == 0) Serial.print('.');
  }
  lastMotorActivity = millis();  // Update activity time
  Serial.println(F(" Done"));
}

void testMode() {
  Serial.println(F("\n=== TEST MODE ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  1-4: Move 10mm (1=Xout 2=Xin 3=Yup 4=Ydn)"));
  Serial.println(F("  5-8: Move 50mm (5=Xout 6=Xin 7=Yup 8=Ydn)"));
  Serial.println(F("  m: Measure travel (move until limit)"));
  Serial.println(F("  h: Home all, s: Status, w: Watch inputs"));
  Serial.print(F("Steps/mm:")); Serial.print(STEPS_PER_MM);
  Serial.print(F(" Delay:")); Serial.println(TEST_SPEED_DELAY);
  
  while (true) {
    if (checkEStop()) return;
    checkMotorIdle();  // Disable motors if idle
    
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      // Clear any remaining characters
      while (Serial.available()) Serial.read();
      
      Serial.print(F("\nCmd: ")); Serial.println(cmd);
      
      switch (cmd) {
        case '1': // X out 10mm
          Serial.println(F("X out 10mm"));
          digitalWrite(X_DIR_PIN, DIR_X_OUT);
          delay(50);
          doSteps(X_STEP_PIN, 10L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '2': // X in 10mm
          Serial.println(F("X in 10mm"));
          digitalWrite(X_DIR_PIN, DIR_X_IN);
          delay(50);
          doSteps(X_STEP_PIN, 10L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '3': // Y up 10mm
          Serial.println(F("Y up 10mm"));
          digitalWrite(Y_DIR_PIN, DIR_Y_UP);
          delay(50);
          doSteps(Y_STEP_PIN, 10L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '4': // Y down 10mm
          Serial.println(F("Y down 10mm"));
          digitalWrite(Y_DIR_PIN, DIR_Y_DOWN);
          delay(50);
          doSteps(Y_STEP_PIN, 10L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '5': // X out 50mm
          Serial.println(F("X out 50mm"));
          digitalWrite(X_DIR_PIN, DIR_X_OUT);
          delay(50);
          doSteps(X_STEP_PIN, 50L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '6': // X in 50mm
          Serial.println(F("X in 50mm"));
          digitalWrite(X_DIR_PIN, DIR_X_IN);
          delay(50);
          doSteps(X_STEP_PIN, 50L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '7': // Y up 50mm
          Serial.println(F("Y up 50mm"));
          digitalWrite(Y_DIR_PIN, DIR_Y_UP);
          delay(50);
          doSteps(Y_STEP_PIN, 50L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case '8': // Y down 50mm
          Serial.println(F("Y down 50mm"));
          digitalWrite(Y_DIR_PIN, DIR_Y_DOWN);
          delay(50);
          doSteps(Y_STEP_PIN, 50L * STEPS_PER_MM, TEST_SPEED_DELAY);
          break;
          
        case 'm': // Measure travel distance
        case 'M':
          Serial.println(F("\n=== MEASURE TRAVEL ==="));
          Serial.println(F("First, manually position axes away from limits"));
          Serial.println(F("Then press: x=measure X, y=measure Y"));
          while (!Serial.available()) delay(10);
          {
            char axis = Serial.read();
            while (Serial.available()) Serial.read();
            
            if (axis == 'x' || axis == 'X') {
              Serial.println(F("Measuring X travel to limit..."));
              digitalWrite(X_DIR_PIN, DIR_X_IN);
              delay(50);
              long steps = 0;
              while (digitalRead(X_LIMIT_PIN) == HIGH && steps < 100000L) {
                digitalWrite(X_STEP_PIN, HIGH);
                delayMicroseconds(50);  // Longer HIGH pulse
                digitalWrite(X_STEP_PIN, LOW);
                delayMicroseconds(TEST_SPEED_DELAY);
                steps++;
                if (steps % 400 == 0) Serial.print('.');
              }
              Serial.println();
              Serial.print(F("X Steps: ")); Serial.println(steps);
              Serial.print(F("X Travel: ")); Serial.print(steps / STEPS_PER_MM); Serial.println(F(" mm"));
            }
            else if (axis == 'y' || axis == 'Y') {
              Serial.println(F("Measuring Y travel to limit..."));
              digitalWrite(Y_DIR_PIN, DIR_Y_DOWN);
              delay(50);
              long steps = 0;
              while (digitalRead(Y_LIMIT_PIN) == HIGH && steps < 100000L) {
                digitalWrite(Y_STEP_PIN, HIGH);
                delayMicroseconds(50);  // Longer HIGH pulse
                digitalWrite(Y_STEP_PIN, LOW);
                delayMicroseconds(TEST_SPEED_DELAY);
                steps++;
                if (steps % 400 == 0) Serial.print('.');
              }
              Serial.println();
              Serial.print(F("Y Steps: ")); Serial.println(steps);
              Serial.print(F("Y Travel: ")); Serial.print(steps / STEPS_PER_MM); Serial.println(F(" mm"));
            }
          }
          break;
          
        case 'h': // Home
        case 'H':
          Serial.println(F("Homing..."));
          performHoming();
          break;
          
        case 's': // Status
        case 'S':
          Serial.print(F("X lim:")); Serial.print(digitalRead(X_LIMIT_PIN) == LOW ? "HIT " : "- ");
          Serial.print(F("Y lim:")); Serial.print(digitalRead(Y_LIMIT_PIN) == LOW ? "HIT " : "- ");
          Serial.print(F("Btn:")); Serial.print(digitalRead(COOL_EN_PIN) == LOW ? "ON " : "- ");
          Serial.print(F("ES:")); Serial.println(digitalRead(E_STOP_PIN) == LOW ? "ON" : "-");
          break;
          
        case 'w': // Watch inputs
        case 'W':
          Serial.println(F("Watching (press any key to stop)..."));
          while (!Serial.available()) {
            Serial.print(F("X:")); Serial.print(digitalRead(X_LIMIT_PIN) == LOW ? "T " : "- ");
            Serial.print(F("Y:")); Serial.print(digitalRead(Y_LIMIT_PIN) == LOW ? "T " : "- ");
            Serial.print(F("Btn:")); Serial.print(digitalRead(COOL_EN_PIN) == LOW ? "T " : "- ");
            Serial.print(F("ES:")); Serial.println(digitalRead(E_STOP_PIN) == LOW ? "T" : "-");
            delay(200);
          }
          while (Serial.available()) Serial.read();
          Serial.println(F("Watch stopped"));
          break;
          
        case 'e': // Extend
        case 'E':
          Serial.println(F("Extend sequence..."));
          extendSequence();
          break;
          
        case 'r': // Retract
        case 'R':
          Serial.println(F("Retract sequence..."));
          retractSequence();
          break;
          
        case '?': // Help
          Serial.println(F("\n=== COMMANDS ==="));
          Serial.println(F("1=Xout10 2=Xin10 3=Yup10 4=Ydn10"));
          Serial.println(F("5=Xout50 6=Xin50 7=Yup50 8=Ydn50"));
          Serial.println(F("m=measure h=home s=status w=watch"));
          Serial.println(F("e=extend r=retract"));
          break;
          
        default:
          if (cmd >= 32) { // Printable character
            Serial.println(F("Unknown. Press ? for help"));
          }
          break;
      }
    }
    delay(10);
  }
}
