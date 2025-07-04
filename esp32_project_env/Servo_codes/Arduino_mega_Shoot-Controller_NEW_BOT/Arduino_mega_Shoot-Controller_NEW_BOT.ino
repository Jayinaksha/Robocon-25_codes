/*
 * ESP32 Joystick Data Receiver with Basic Motor Control
 * Uses axis[0] for left/right and axis[1] for forward/back
 * Format from ROS: "axis_values;button_values"
 */

#include <Servo.h>

bool Override = true;
bool Shoot_mode1= false;
bool Shoot_mode2= false;
bool Shoot_mode3= false;
bool Stop_mode= false;

const int initial_angle = 120;
const int shoot_angle = 0;
const int toggle_delay = 250;

const int MAX_AXES = 8;
float axes[MAX_AXES] = {0.0};

const int MAX_BUTTONS = 15;
int buttons[MAX_BUTTONS] = {0};

//Servo setup
Servo shootServo;

const int spin = 5; //Servo signal pin
int currentAngle = initial_angle;

// direction
int mr = 22; // right motor
int ml = 26; // left motor
int ma = 13; // Shoot angle motor

// pwm
int Mr = 9; // right
int Ml = 11; // left
int Ma = 14; // shoot angle 

// Angle change speed
const int angle_change_speed = 150;
//rotation speed
const int rot_speed = 100;
const int incr_dcr_const = 10;

int change_speed = 0;

//Serial parsing
String inputBuffer = "";

//function declaration or prototype
void shoot();
void motor_rpm(int speed);
void Rotate_angle(int targetAngle);
void check();
int parseButtons(String &buttonsStr, int *buttons, int maxButtons);
int parseAxes(String &axesStr, float *axes, int maxAxes);
void Shoot_control();

void setup() {
  Serial.begin(115200);

  // Servo initialization
  //shootServo.setPeriodHertz(50); //Standart 50Hz Servo
  shootServo.attach(spin); // set min/max pulse width
  shootServo.write(initial_angle);
  Serial.println("Servo initiated at 120 degrees.");

  pinMode(mr, OUTPUT);
  pinMode(ml, OUTPUT);
  pinMode(ma, OUTPUT);

  Serial.println("Motors initialised. Send the data.");

  Serial.println("ESP32 Shoot Controller started");
  Serial.println("Waiting for data from ROS...");
}

void loop() {
   // Read serial data robustly
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInputBuffer();
      inputBuffer = "";
    } else if (c >= 32 && c <= 126) { // Only store printable ASCII for safety
      inputBuffer += c;
    }
  }

  check(); // Handle override toggle
  delay(2); // Small delay to yield CPU; no blocking delays!
}

void processInputBuffer() {
  if (inputBuffer.length() == 0) return;

  //Serial.print("RAW: ");
  //Serial.println(inputBuffer);

  int delimiterPos = inputBuffer.indexOf(';');
  if (delimiterPos <= 0) {
    Serial.println("Packet missing delimiter, skipping.");
    return;
  }
  else{

    String axesStr = inputBuffer.substring(0, delimiterPos);
    String buttonsStr = inputBuffer.substring(delimiterPos + 1);

    parseAxes(axesStr, axes, MAX_AXES);
    parseButtons(buttonsStr, buttons, MAX_BUTTONS);

  // Print button 9 for debug:
  // Serial.print("Button9: ");
  // Serial.println(buttons[9]);

  // Execute robot control
  Shoot_control();
  }
}

void Shoot_control(){
  const int speed1 = 150 + change_speed;
  const int speed2 = 200 + change_speed;
  const int speed3 = 220 + change_speed;

  char speed[100];

  if (Override==1){
    analogWrite(Mr,0);
    analogWrite(Ml,0);
    analogWrite(Ma,0);
    Rotate_angle(initial_angle);
  }
  
  else if(Stop_mode==1){
    analogWrite(Mr,0);
    analogWrite(Ml,0);
    Rotate_angle(initial_angle);

  }

  else if(Shoot_mode1==1){
    motor_rpm(speed1);
    sprintf(speed, "Current speed: %d", speed1);
    Serial.println(speed);
  }

  else if(Shoot_mode2==1){
    motor_rpm(speed2);
    sprintf(speed, "Current speed: %d", speed2);
    Serial.println(speed);
  }

  else if(Shoot_mode3==1){
    motor_rpm(speed3);
    sprintf(speed, "Current speed: %d", speed3);
    Serial.println(speed);
  }

  else if (buttons[10]==1){
    delay(250);
    shoot();
    Serial.print("Shooting_initiated.");
  }
  else{
    analogWrite(Mr,0);
    analogWrite(Ml,0);
    Rotate_angle(initial_angle);
  }

  if (buttons[14]==1 && Shoot_mode1==0 && Shoot_mode2==0 && Shoot_mode3==0 && Override==0){
    digitalWrite(ma, 0);
    analogWrite(Ma,angle_change_speed);
    Serial.println("Angle increasing....");
  }

  else if (buttons[13]==1 && Shoot_mode1==0 && Shoot_mode2==0 && Shoot_mode3==0 && Override==0){
    digitalWrite(ma, 1);
    analogWrite(Ma,angle_change_speed);
    Serial.println("Angle decreasing....");
  }
  else {
    analogWrite(Ma, 0);
  }

  if (buttons[10]==1){
    delay(toggle_delay);
    shoot();
    Serial.print("Shooting_initiated.");
  }

}

// Helper function to parse axes
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

// Helper function to parse buttons
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

void check(){
  static int last_override_button = 0, last_shoot1_button = 0, last_shoot2_button = 0, last_shoot3_button = 0;
  static int last_stop_button = 0, last_speedup_button = 0, last_slowdown_button = 0;
  static unsigned long last_override_time = 0, last_shoot1_time = 0, last_shoot2_time = 0, last_shoot3_time = 0;
  static unsigned long last_stop_time = 0, last_speedup_time = 0, last_slowdown_time = 0;
  const unsigned long debounce_ms = 200;
  unsigned long now = millis();

  //Override
  if(buttons[9]==1 && last_override_button == 0 && now - last_override_time > debounce_ms){
    Override =! Override;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Shoot_mode3=0;
    Stop_mode=1;
    change_speed=0;
    Serial.print("Override:");
    Serial.println(Override);
    last_override_time = now;
  }
  last_override_button = buttons[9];

  //Shoot mode 1 (button 0)
  if(buttons[0]==1 && last_shoot1_button == 0 && now - last_shoot1_time > debounce_ms){
    Shoot_mode1 = 1;
    Shoot_mode2=0;
    Shoot_mode3=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 1 activating with speed: 150");
    last_shoot1_time = now;
  }
  last_shoot1_button = buttons[0];

  // SHOOT MODE 2 (button 1)
  if(buttons[1]==1 && last_shoot2_button == 0 && now - last_shoot2_time > debounce_ms){
    Shoot_mode2 = 1;
    Shoot_mode1=0;
    Shoot_mode3=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 2 activating with speed: 200");
    last_shoot2_time = now;
  }
  last_shoot2_button = buttons[1];

  // SHOOT MODE 3 (button 3)
  if(buttons[3]==1 && last_shoot3_button == 0 && now - last_shoot3_time > debounce_ms){
    Shoot_mode3 = 1;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 3 activating with speed: 220");
    last_shoot3_time = now;
  }
  last_shoot3_button = buttons[3];
  
  // STOP MODE (button 2)
  if(buttons[2]==1 && last_stop_button == 0 && now - last_stop_time > debounce_ms){
    Stop_mode = 1;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Shoot_mode3=0;
    change_speed=0;
    Serial.println("Stop mode  activated stopping Flywheel");
    last_stop_time = now;
  }
  last_stop_button = buttons[2];

  // SPEED UP (button 11)
  if(buttons[11]==1 && last_speedup_button == 0 && now - last_speedup_time > debounce_ms){
    change_speed = change_speed + incr_dcr_const;
    Serial.print("Speed increased: ");
    Serial.println(change_speed);
    last_speedup_time = now;
  }
  last_speedup_button = buttons[11];

  // SLOW DOWN (button 12)
  if(buttons[12]==1 && last_slowdown_button == 0 && now - last_slowdown_time > debounce_ms){
    change_speed = change_speed - incr_dcr_const;
    Serial.print("Speed decreased: ");
    Serial.println(change_speed);
    last_slowdown_time = now;
  }
  last_slowdown_button = buttons[12];
}

void Rotate_angle(int targetAngle){
  if(currentAngle == targetAngle) return;

  int step = (targetAngle > currentAngle) ? 5 : -5;
  while (currentAngle != targetAngle){
    currentAngle += step;
    shootServo.write(currentAngle);
    delay(5);
  }
  Serial.print("Servo reached:");
  Serial.print(currentAngle);
  Serial.println("degrees");
}
void motor_rpm(int speed){
  digitalWrite(mr,0);
  digitalWrite(ml,1);
  analogWrite(Mr,speed);
  analogWrite(Ml,speed);
}

void shoot(){
  if(Shoot_mode1==1 || Shoot_mode2==1 || Shoot_mode3 == 1){
    Rotate_angle(shoot_angle);
    Serial.println("Shooting");
    delay(1500);
    Rotate_angle(initial_angle);
    Serial.println("Comming back, stopping...");
    Stop_mode= 1;
    Shoot_mode1= 0;
    Shoot_mode2= 0;
    Shoot_mode3= 0;
  }
}