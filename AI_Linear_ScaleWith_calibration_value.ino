/*
 * Arduino Linear Scale Controller with Calibration & Manual Override
 *
 * This sketch reads a quadrature linear scale with 0.005mm resolution using interrupts,
 * monitors active-low upper and lower limit switches, and controls a motor
 * based on serial commands or manual push buttons. It reports its position in mm while moving.
 *
 * Hardware Connections:
 * - Linear Scale Channel A -> Arduino D3 (Interrupt 0)
 * - Linear Scale Channel B -> Arduino D2 (Interrupt 1)
 * - Manual Up Button     -> Arduino D4 (Active Low)
 * - Manual Down Button   -> Arduino D5 (Active Low)
 * - Motor Control (Lower)  -> Arduino D9 (Active Low)
 * - Motor Control (Raise)  -> Arduino D10 (Active Low)
 * - Lower Limit Switch   -> Arduino D12 (Active Low)
 * - Upper Limit Switch   -> Arduino D11 (Active Low)
 *
 * Serial Commands Format:
 * - "U<distance>" : Move Up by <distance> mm (e.g., "U150")
 * - "D<distance>" : Move Down by <distance> mm (e.g., "D75.5")
 * - "S"             : Stop the motor immediately
 * - "Z"             : Zero the current position counter
 * - "C"             : Start calibration cycle (moves up to top limit switch)
 * - "P<value>"      : Set the current position to <value> mm (e.g., "P200.0")
 */

// --- Pin Definitions ---
const int ENCODER_A_PIN = 3;  // Interrupt Pin 1
const int ENCODER_B_PIN = 2;  // Interrupt Pin 0
const int MANUAL_UP_PIN = 4;
const int MANUAL_DOWN_PIN = 5;
const int MOTOR_DOWN_PIN = 9;
const int MOTOR_UP_PIN = 10;
const int LOWER_LIMIT_PIN = 12;
const int UPPER_LIMIT_PIN = 11;

// --- Constants ---
const float RESOLUTION_MM = 0.02; // The resolution of the linear scale in mm
const long POSITION_REPORT_INTERVAL = 150; // Milliseconds between position reports while moving

// --- Global Variables ---
// Volatile variables are used because they are modified in ISRs
volatile long currentPositionCounts = 0; // Current position in encoder counts
volatile long targetPositionCounts = 0;  // Target position in encoder counts for auto moves
volatile byte encoderA_lastState = 0;

// Motor state enumeration for automatic commands
enum MotorState { STOPPED, MOVING_UP, MOVING_DOWN, CALIBRATING };
MotorState motorState = STOPPED;

// Limit switch flags
bool lowerLimitActive = false;
bool upperLimitActive = false;

// For position reporting timer
unsigned long lastPositionReportTime = 0;

// Flag to indicate if manual control is active
bool manualOverride = false;

// --- Setup Function: Runs once on startup ---
void setup() {
  // Initialize Serial Communication at 9600 baud
  Serial.begin(115200);
  Serial.println("Linear Scale Controller Initialized");

  // --- Pin Modes ---
  // Motor control pins are outputs, initialized to HIGH (inactive)
  pinMode(MOTOR_UP_PIN, OUTPUT);
  pinMode(MOTOR_DOWN_PIN, OUTPUT);
  digitalWrite(MOTOR_UP_PIN, HIGH);
  digitalWrite(MOTOR_DOWN_PIN, HIGH);

  // Limit switch and manual control pins are inputs with internal pull-up resistors
  pinMode(LOWER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(UPPER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(MANUAL_UP_PIN, INPUT_PULLUP);
  pinMode(MANUAL_DOWN_PIN, INPUT_PULLUP);


  // Encoder pins are inputs
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  
  // Store the initial state of encoder A for quadrature decoding
  encoderA_lastState = digitalRead(ENCODER_A_PIN);

  // --- Interrupts ---
  // Attach interrupts to encoder pins. The ISR 'handleEncoder' is called on any change.
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), handleEncoder, CHANGE);
  
  Serial.println("Ready to receive commands or manual input.");
}

// --- Main Loop: Runs continuously ---
void loop() {
  // Always check hardware status first
  checkLimitSwitches();
  handleManualControls(); // Prioritize manual buttons

  // If manual override is active, skip all automatic/serial logic
  if (manualOverride) {
    reportPosition(); // Report position during manual moves
    delay(10);
    return;
  }
  
  // --- Automatic / Serial Control Section ---
  // This section only runs when manual buttons are not being pressed.
  if (Serial.available() > 0) {
    handleSerialCommand();
  }
  controlMotor(); // Manages automatic movement and calibration
  reportPosition(); // Report position during automatic moves

  delay(10); 
}

// --- Function to handle manual jog buttons ---
void handleManualControls() {
    bool manualUpPressed = (digitalRead(MANUAL_UP_PIN) == LOW);
    bool manualDownPressed = (digitalRead(MANUAL_DOWN_PIN) == LOW);

    if (manualUpPressed) {
        // If a manual button is pressed, cancel any auto-move
        if (motorState != STOPPED) {
            Serial.println("Manual override: Auto-move cancelled.");
            stopMotor(); // This sets motorState to STOPPED
        }
        moveUp(); // Directly call motor function
        manualOverride = true; // Set override flag
    } else if (manualDownPressed) {
        // If a manual button is pressed, cancel any auto-move
        if (motorState != STOPPED) {
            Serial.println("Manual override: Auto-move cancelled.");
            stopMotor();
        }
        moveDown();
        manualOverride = true;
    } else {
        // No manual buttons are pressed.
        // If we WERE just pressing one, we need to stop the motor now.
        if (manualOverride) {
            stopMotor();
            manualOverride = false; // Clear override flag
            Serial.println("Manual control released.");
        }
    }
}

// --- Interrupt Service Routine for Encoders ---
void handleEncoder() {
  // Read the current state of the encoder pins
  byte stateA = digitalRead(ENCODER_A_PIN);
  byte stateB = digitalRead(ENCODER_B_PIN);

  // Determine direction based on quadrature logic
  if ((encoderA_lastState == LOW) && (stateA == HIGH)) {
    if (stateB == LOW) {
      currentPositionCounts--; // Moving down
    } else {
      currentPositionCounts++; // Moving up
    }
  }
  // Update the last state of Encoder A for the next interrupt
  encoderA_lastState = stateA;
}

// --- Function to Handle Serial Commands ---
void handleSerialCommand() {
  char command = toupper(Serial.read());

  if (command == 'U' || command == 'D') {
    float distanceMm = Serial.parseFloat();
    long distanceCounts = (long)(distanceMm / RESOLUTION_MM);

    if (command == 'U') {
      targetPositionCounts = currentPositionCounts + distanceCounts;
      motorState = MOVING_UP;
      Serial.print("Command: Move UP by ");
      Serial.print(distanceMm);
      Serial.println("mm");
    } else { // command == 'D'
      targetPositionCounts = currentPositionCounts - distanceCounts;
      motorState = MOVING_DOWN;
      Serial.print("Command: Move DOWN by ");
      Serial.print(distanceMm);
      Serial.println("mm");
    }
  } else if (command == 'S') {
    motorState = STOPPED;
    targetPositionCounts = currentPositionCounts;
    Serial.println("Command: STOP");
  } else if (command == 'Z') {
    currentPositionCounts = 0;
    targetPositionCounts = 0;
    Serial.println("Command: Position ZEROED");
  } else if (command == 'C') {
    motorState = CALIBRATING;
    Serial.println("Command: Calibrating...");
  } else if (command == 'P') {
    float newPositionMm = Serial.parseFloat();
    currentPositionCounts = (long)(newPositionMm / RESOLUTION_MM);
    targetPositionCounts = currentPositionCounts; // Ensure we don't move
    Serial.print("Command: Position set to ");
    Serial.print(newPositionMm, 3);
    Serial.println(" mm");
  }

  // Clear any remaining characters in the serial buffer
  while(Serial.available() > 0) { Serial.read(); }
}

// --- Function to Check Limit Switches ---
void checkLimitSwitches() {
  lowerLimitActive = (digitalRead(LOWER_LIMIT_PIN) == LOW);
  upperLimitActive = (digitalRead(UPPER_LIMIT_PIN) == LOW);
}

// --- Function to Control the Motor for Automatic Commands and Calibration ---
void controlMotor() {
  if (motorState == MOVING_UP) {
    if (currentPositionCounts >= targetPositionCounts || upperLimitActive) {
      stopMotor();
      if (!upperLimitActive) Serial.println("Target reached.");
    } else {
      moveUp();
    }
  } else if (motorState == MOVING_DOWN) {
    if (currentPositionCounts <= targetPositionCounts || lowerLimitActive) {
      stopMotor();
      if (!lowerLimitActive) Serial.println("Target reached.");
    } else {
      moveDown();
    }
  } else if (motorState == CALIBRATING) {
    if (upperLimitActive) {
      stopMotor();
      Serial.println("Done");
    } else {
      moveUp();
    }
  } else { // motorState == STOPPED
    stopMotor();
  }
}

// --- Function to send position data over serial while moving ---
void reportPosition() {
  // Report if motor is active (either pin is LOW), regardless of mode
  bool motorIsActive = (digitalRead(MOTOR_UP_PIN) == LOW) || (digitalRead(MOTOR_DOWN_PIN) == LOW);
  if (motorIsActive && (millis() - lastPositionReportTime > POSITION_REPORT_INTERVAL)) {
    float positionMm = (float)currentPositionCounts * RESOLUTION_MM;
    Serial.println(positionMm, 3); // Print position in mm with 3 decimal places
    lastPositionReportTime = millis();
  }
}

// --- Motor Action Functions ---
void moveUp() {
  if (!upperLimitActive) {
    digitalWrite(MOTOR_DOWN_PIN, HIGH);
    digitalWrite(MOTOR_UP_PIN, LOW);
  } else {
    stopMotor();
    Serial.println("WARNING: Upper limit active. Cannot move up.");
  }
}

void moveDown() {
  if (!lowerLimitActive) {
    digitalWrite(MOTOR_UP_PIN, HIGH);
    digitalWrite(MOTOR_DOWN_PIN, LOW);
  } else {
    stopMotor();
    Serial.println("WARNING: Lower limit active. Cannot move down.");
  }
}

void stopMotor() {
  if (motorState != STOPPED) {
     motorState = STOPPED;
  }
  digitalWrite(MOTOR_UP_PIN, HIGH);
  digitalWrite(MOTOR_DOWN_PIN, HIGH);
}

