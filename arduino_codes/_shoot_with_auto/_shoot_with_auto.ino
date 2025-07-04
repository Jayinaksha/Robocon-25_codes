/* Enhanced Shoot Controller for Basketball Robot
 * Integrates with existing PS4 controller and adds autonomous shooting
 * ESP32 with servo and motor control for basketball shooting
 * INCLUDES Button 8 logic for shooting permission
 */

#include <Servo.h>

bool Override = true;
bool AutonomousMode = false;
bool Shoot_mode1 = false;
bool Shoot_mode2 = false;
bool Shoot_mode3 = false;
bool Stop_mode = false;
bool AutoShootInProgress = false;
bool ShootPermissionGranted = false;  // NEW: Track shooting permission

// Shooting speeds (PWM values)
const int shoot_speed1 = 100;
const int shoot_speed2 = 120;
const int shoot_speed3 = 150;

// Servo angles
const int initial_angle = 90;
const int shoot_angle = 0;
const int shoot_delay = 1000;

const int MAX_AXES = 8;
float axes[MAX_AXES] = {0.0};

const int MAX_BUTTONS = 15;
int buttons[MAX_BUTTONS] = {0};

// Servo setup
Servo shootServo;
const int spin = 5; // Servo signal pin
int currentAngle = initial_angle;

// Motor pins - direction
int mr = 22; // Right motor
int ml = 26; // Left motor
int ma = 13; // Shoot angle motor

// Motor pins - pwm
int Mr = 9;  // Right
int Ml = 11; // Left
int Ma = 14; // Shoot angle

// Speed control
const int angle_change_speed = 150;
const int rot_speed = 100;
const int incr_dcr_const = 10;
int change_speed = 0;

// Timing
const int toggle_delay = 250;
unsigned long auto_shoot_start_time = 0;
int auto_shoot_stage = 0; // 0: idle, 1: spinning up, 2: shooting, 3: completing

// Serial parsing
String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  
  // Servo initialization
  shootServo.attach(spin);
  shootServo.write(initial_angle);
  Serial.println("Servo initialized at 90 degrees");
  
  // Initialize motor pins
  pinMode(mr, OUTPUT);
  pinMode(ml, OUTPUT);
  pinMode(ma, OUTPUT);
  
  // Initialize motors to stop
  stopShootMotors();
  
  Serial.println("=== BASKETBALL ROBOT SHOOT CONTROLLER ===");
  Serial.println("Enhanced Shoot Controller started");
  Serial.println("Button 8: Grant Shooting Permission");
  Serial.println("Button 9: Override Toggle");
  Serial.println("Supports PS4 controller and autonomous shooting");
  Serial.println("Waiting for data...");
}

void loop() {
  // Read serial data
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processSerialInput();
      inputBuffer = "";
    } else if (c >= 32 && c <= 126) {
      inputBuffer += c;
    }
  }
  
  // Handle autonomous shooting
  if (AutonomousMode && AutoShootInProgress) {
    processAutonomousShoot();
  }
  
  // Check button states (including Button 8 for shooting permission)
  check_all_buttons();
  
  delay(2);
}

void processSerialInput() {
  if (inputBuffer.length() == 0) return;
  
  // Check if it's a ROS command
  if (inputBuffer.startsWith("SHOOT:") || 
      inputBuffer.startsWith("AUTO_MODE:") || 
      inputBuffer.startsWith("STOP_SHOOT")) {
    
    processRosCommand(inputBuffer);
  } else {
    // Process PS4 controller data
    processPS4Data(inputBuffer);
  }
}

void processRosCommand(String command) {
  Serial.print("ROS Shoot Command: ");
  Serial.println(command);
  
  if (command.startsWith("SHOOT:")) {
    // Parse: SHOOT:mode1, SHOOT:mode2, SHOOT:mode3
    String mode = command.substring(6);
    
    if (mode == "mode1") {
      executeAutonomousShoot(1);
    } else if (mode == "mode2") {
      executeAutonomousShoot(2);
    } else if (mode == "mode3") {
      executeAutonomousShoot(3);
    } else {
      Serial.println("Unknown shoot mode");
    }
  }
  else if (command.startsWith("AUTO_MODE:")) {
    // Parse: AUTO_MODE:ON or AUTO_MODE:OFF
    bool autoMode = command.substring(10) == "ON";
    AutonomousMode = autoMode;
    Serial.print("Autonomous mode: ");
    Serial.println(autoMode ? "ON" : "OFF");
    
    if (!autoMode) {
      // Reset shooting permission when exiting autonomous mode
      ShootPermissionGranted = false;
      Serial.println("Shooting permission reset");
    }
  }
  else if (command == "STOP_SHOOT") {
    Serial.println("Stop shoot command received");
    stopAutonomousShoot();
  }
}

void processPS4Data(String data) {
  // Process PS4 controller data
  int delimiterPos = data.indexOf(';');
  if (delimiterPos <= 0) {
    Serial.println("PS4 packet missing delimiter, skipping.");
    return;
  }
  
  String axesStr = data.substring(0, delimiterPos);
  String buttonsStr = data.substring(delimiterPos + 1);
  
  parseAxes(axesStr, axes, MAX_AXES);
  parseButtons(buttonsStr, buttons, MAX_BUTTONS);
  
  // Only process PS4 shooting if not in autonomous mode
  if (!AutonomousMode && !AutoShootInProgress) {
    Shoot_control();
  }
}

void executeAutonomousShoot(int mode) {
  if (Override) {
    Serial.println("Cannot shoot - Override is ON");
    return;
  }
  
  if (!ShootPermissionGranted) {
    Serial.println("🎯 SHOOTING PERMISSION REQUIRED!");
    Serial.println("🎯 Press Button 8 to grant shooting permission");
    return;
  }
  
  Serial.print("Starting autonomous shooting with mode ");
  Serial.println(mode);
  
  AutoShootInProgress = true;
  auto_shoot_start_time = millis();
  auto_shoot_stage = 1;
  
  // Set shooting mode
  Shoot_mode1 = (mode == 1);
  Shoot_mode2 = (mode == 2);
  Shoot_mode3 = (mode == 3);
  Stop_mode = false;
  
  Serial.print("Autonomous shooting started with speed: ");
  if (mode == 1) Serial.println(shoot_speed1 + change_speed);
  else if (mode == 2) Serial.println(shoot_speed2 + change_speed);
  else if (mode == 3) Serial.println(shoot_speed3 + change_speed);
}

void processAutonomousShoot() {
  unsigned long elapsed = millis() - auto_shoot_start_time;
  
  switch (auto_shoot_stage) {
    case 1: // Spinning up motors
      if (elapsed > 2000) { // 2 seconds to spin up
        Serial.println("Motors spun up - executing shoot");
        shoot();
        auto_shoot_stage = 2;
        auto_shoot_start_time = millis();
      }
      break;
      
    case 2: // Shooting in progress
      if (elapsed > 3000) { // 3 seconds for shooting
        Serial.println("Shooting completed");
        auto_shoot_stage = 3;
        auto_shoot_start_time = millis();
      }
      break;
      
    case 3: // Completing and stopping
      stopAutonomousShoot();
      Serial.println("STATUS:SHOOT_COMPLETE");
      
      // Reset shooting permission after shooting
      ShootPermissionGranted = false;
      Serial.println("Shooting permission reset - ready for next shot");
      break;
  }
}

void stopAutonomousShoot() {
  AutoShootInProgress = false;
  auto_shoot_stage = 0;
  Shoot_mode1 = false;
  Shoot_mode2 = false;
  Shoot_mode3 = false;
  Stop_mode = true;
  
  stopShootMotors();
  Rotate_angle(initial_angle);
  
  Serial.println("Autonomous shooting stopped");
}

void stopShootMotors() {
  analogWrite(Mr, 0);
  analogWrite(Ml, 0);
  analogWrite(Ma, 0);
}

void Shoot_control() {
  const int speed1 = shoot_speed1 + change_speed;
  const int speed2 = shoot_speed2 + change_speed;
  const int speed3 = shoot_speed3 + change_speed;
  
  if (Override == 1) {
    stopShootMotors();
    Rotate_angle(initial_angle);
    return;
  }
  
  if (Stop_mode == 1) {
    stopShootMotors();
    Rotate_angle(initial_angle);
  }
  else if (Shoot_mode1 == 1) {
    motor_rpm(speed1);
    Serial.print("Shoot mode 1 - Speed: ");
    Serial.println(speed1);
  }
  else if (Shoot_mode2 == 1) {
    motor_rpm(speed2);
    Serial.print("Shoot mode 2 - Speed: ");
    Serial.println(speed2);
  }
  else if (Shoot_mode3 == 1) {
    motor_rpm(speed3);
    Serial.print("Shoot mode 3 - Speed: ");
    Serial.println(speed3);
  }
  else if (buttons[10] == 1) {
    delay(toggle_delay);
    shoot();
    Serial.println("Manual shooting initiated");
  }
  else {
    stopShootMotors();
    Rotate_angle(initial_angle);
  }
  
  // Angle control
  if (buttons[14] == 1 && !Shoot_mode1 && !Shoot_mode2 && !Shoot_mode3 && !Override) {
    digitalWrite(ma, 0);
    analogWrite(Ma, angle_change_speed);
    Serial.println("Angle increasing");
  }
  else if (buttons[13] == 1 && !Shoot_mode1 && !Shoot_mode2 && !Shoot_mode3 && !Override) {
    digitalWrite(ma, 1);
    analogWrite(Ma, angle_change_speed);
    Serial.println("Angle decreasing");
  }
  else {
    analogWrite(Ma, 0);
  }
}

// *** ENHANCED: Button handling including Button 8 for shooting permission ***
void check_all_buttons() {
  // Static variables for button state tracking
  static int last_override_button = 0, last_shoot1_button = 0, last_shoot2_button = 0, last_shoot3_button = 0;
  static int last_stop_button = 0, last_speedup_button = 0, last_slowdown_button = 0;
  static int last_shoot_permission_button = 0;  // NEW: Button 8 for shooting permission
  
  static unsigned long last_override_time = 0, last_shoot1_time = 0, last_shoot2_time = 0, last_shoot3_time = 0;
  static unsigned long last_stop_time = 0, last_speedup_time = 0, last_slowdown_time = 0;
  static unsigned long last_shoot_permission_time = 0;  // NEW: Timing for Button 8
  
  const unsigned long debounce_ms = 200;
  unsigned long now = millis();
  
  // Override button (button 9)
  if (buttons[9] == 1 && last_override_button == 0 && now - last_override_time > debounce_ms) {
    Override = !Override;
    if (Override) {
      stopAutonomousShoot();
      ShootPermissionGranted = false;  // Reset permission when override is ON
    }
    Shoot_mode1 = false;
    Shoot_mode2 = false;
    Shoot_mode3 = false;
    Stop_mode = true;
    change_speed = 0;
    Serial.print("Override: ");
    Serial.println(Override ? "ON" : "OFF");
    last_override_time = now;
  }
  last_override_button = buttons[9];
  
  // *** NEW: Button 8 - Shooting Permission ***
  if (buttons[8] == 1 && last_shoot_permission_button == 0 && now - last_shoot_permission_time > debounce_ms) {
    if (!Override) {  // Only allow if Override is OFF
      ShootPermissionGranted = !ShootPermissionGranted;
      Serial.print("🎯 Button 8 pressed - Shooting Permission: ");
      Serial.println(ShootPermissionGranted ? "GRANTED" : "REVOKED");
      
      if (ShootPermissionGranted) {
        Serial.println("🏀 Ready to shoot when commanded!");
        Serial.println("STATUS:SHOOT_PERMISSION_GRANTED");
      } else {
        Serial.println("🛑 Shooting permission revoked");
        Serial.println("STATUS:SHOOT_PERMISSION_REVOKED");
      }
      
      last_shoot_permission_time = now;
    } else {
      Serial.println("❌ Cannot grant shooting permission - Override is ON");
    }
  }
  last_shoot_permission_button = buttons[8];
  
  // Shoot mode 1 (button 0)
  if (buttons[0] == 1 && last_shoot1_button == 0 && now - last_shoot1_time > debounce_ms) {
    Shoot_mode1 = true;
    Shoot_mode2 = false;
    Shoot_mode3 = false;
    Stop_mode = false;
    change_speed = 0;
    Serial.println("Shoot mode 1 activated - Speed: 100");
    last_shoot1_time = now;
  }
  last_shoot1_button = buttons[0];
  
  // Shoot mode 2 (button 1)
  if (buttons[1] == 1 && last_shoot2_button == 0 && now - last_shoot2_time > debounce_ms) {
    Shoot_mode2 = true;
    Shoot_mode1 = false;
    Shoot_mode3 = false;
    Stop_mode = false;
    change_speed = 0;
    Serial.println("Shoot mode 2 activated - Speed: 120");
    last_shoot2_time = now;
  }
  last_shoot2_button = buttons[1];
  
  // Shoot mode 3 (button 3)
  if (buttons[3] == 1 && last_shoot3_button == 0 && now - last_shoot3_time > debounce_ms) {
    Shoot_mode3 = true;
    Shoot_mode1 = false;
    Shoot_mode2 = false;
    Stop_mode = false;
    change_speed = 0;
    Serial.println("Shoot mode 3 activated - Speed: 150");
    last_shoot3_time = now;
  }
  last_shoot3_button = buttons[3];
  
  // Stop mode (button 2)
  if (buttons[2] == 1 && last_stop_button == 0 && now - last_stop_time > debounce_ms) {
    Stop_mode = true;
    Shoot_mode1 = false;
    Shoot_mode2 = false;
    Shoot_mode3 = false;
    change_speed = 0;
    Serial.println("Stop mode activated");
    last_stop_time = now;
  }
  last_stop_button = buttons[2];
  
  // Speed up (button 11)
  if (buttons[11] == 1 && last_speedup_button == 0 && now - last_speedup_time > debounce_ms) {
    change_speed += incr_dcr_const;
    Serial.print("Speed increased by: ");
    Serial.println(incr_dcr_const);
    last_speedup_time = now;
  }
  last_speedup_button = buttons[11];
  
  // Slow down (button 12)
  if (buttons[12] == 1 && last_slowdown_button == 0 && now - last_slowdown_time > debounce_ms) {
    change_speed -= incr_dcr_const;
    Serial.print("Speed decreased by: ");
    Serial.println(incr_dcr_const);
    last_slowdown_time = now;
  }
  last_slowdown_button = buttons[12];
}

void Rotate_angle(int targetAngle) {
  if (currentAngle == targetAngle) return;
  
  int step = (targetAngle > currentAngle) ? 5 : -5;
  while (currentAngle != targetAngle) {
    currentAngle += step;
    shootServo.write(currentAngle);
    delay(15);
  }
  Serial.print("Servo angle: ");
  Serial.println(currentAngle);
}

void motor_rpm(int speed) {
  digitalWrite(mr, 1);
  digitalWrite(ml, 0);
  analogWrite(Mr, speed);
  analogWrite(Ml, speed);
}

void shoot() {
  if (Shoot_mode1 || Shoot_mode2 || Shoot_mode3) {
    Serial.println("Shooting sequence started");
    Rotate_angle(shoot_angle);
    delay(shoot_delay);
    Rotate_angle(initial_angle);
    Serial.println("Shooting sequence completed");
    
    // Reset modes
    Stop_mode = true;
    Shoot_mode1 = false;
    Shoot_mode2 = false;
    Shoot_mode3 = false;
  }
}

// Helper functions
int parseAxes(String &axesStr, float *axes, int maxAxes) {
  int startPos = 0, axisIndex = 0;
  
  while (startPos >= 0 && axisIndex < maxAxes) {
    int commaPos = axesStr.indexOf(',', startPos);
    if (commaPos >= 0) {
      axes[axisIndex++] = axesStr.substring(startPos, commaPos).toFloat();
      startPos = commaPos + 1;
    } else {
      axes[axisIndex++] = axesStr.substring(startPos).toFloat();
      break;
    }
  }
  return axisIndex;
}

int parseButtons(String &buttonsStr, int *buttons, int maxButtons) {
  int startPos = 0, buttonIndex = 0;
  
  while (startPos >= 0 && buttonIndex < maxButtons) {
    int commaPos = buttonsStr.indexOf(',', startPos);
    if (commaPos >= 0) {
      buttons[buttonIndex++] = buttonsStr.substring(startPos, commaPos).toInt();
      startPos = commaPos + 1;
    } else {
      buttons[buttonIndex++] = buttonsStr.substring(startPos).toInt();
      break;
    }
  }
  return buttonIndex;
}