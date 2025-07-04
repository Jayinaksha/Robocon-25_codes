/*
 * ESP32 Joystick Data Receiver with Basic Motor Control
 * Uses axis[0] for left/right and axis[1] for forward/back
 * Format from ROS: "axis_values;button_values"
 */

#include <ESP32Servo.h>

bool Override = true;
bool Shoot_mode1= false;
bool Shoot_mode2= false;
bool Shoot_mode3= false;
bool Stop_mode= false;

const int initial_angle = 120;
const int shoot_angle = 0;

const int MAX_AXES = 8;
float axes[MAX_AXES] = {0.0};

const int MAX_BUTTONS = 15;
int buttons[MAX_BUTTONS] = {0};

//Servo setup
Servo shootServo;

const int spin = 5; //Servo signal pin
int currentAngle = initial_angle;

//LEDC PWM setup
const int freq = 5000; //PWM frequency
const int resolution = 8; //PWM resolution (0-255)

// direction
int mr = 23; // right motor
int ml = 22; // left motor
int ma = 13; // Shoot angle motor

// pwm
int Mr = 19; // right
int Ml = 21; // left
int Ma = 14; // shoot angle 

//PWM channels for motors
const int MR_pwmChannel = 0;
const int ML_pwmChannel = 1;
const int MA_pwmChannel = 2;

// Angle change speed
const int angle_change_speed = 150;
//rotation speed
const int rot_speed = 100;
const int incr_dcr_const = 10;

int change_speed = 0;

void setup() {
  Serial.begin(115200);

  // Servo initialization
  shootServo.setPeriodHertz(50); //Standart 50Hz Servo
  shootServo.attach(spin, 500, 2400); // set min/max pulse width
  shootServo.write(initial_angle);
  Serial.println("Servo initiated at 120 degrees.");

  pinMode(mr, OUTPUT);
  ledcSetup(MR_pwmChannel, freq, resolution);
  ledcAttachPin(Mr, MR_pwmChannel);

  pinMode(ml, OUTPUT);
  ledcSetup(ML_pwmChannel, freq, resolution);
  ledcAttachPin(Ml, ML_pwmChannel);

  pinMode(ma, OUTPUT);
  ledcSetup(MA_pwmChannel, freq, resolution);
  ledcAttachPin(Ma, MA_pwmChannel);

  Serial.println("Motors initialised. Send the data.");

  Serial.println("ESP32 Shoot Controller started");
  Serial.println("Waiting for data from ROS...");
}

void loop() {
  check();
  static String inputBuffer = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c != '\n') {
      inputBuffer += c;
    } else {
      int delimiterPos = inputBuffer.indexOf(';');

      if (delimiterPos > 0) {
        String axesStr = inputBuffer.substring(0, delimiterPos);
        String buttonsStr = inputBuffer.substring(delimiterPos + 1);

        parseAxes(axesStr, axes, MAX_AXES);
        parseButtons(buttonsStr, buttons, MAX_BUTTONS);

        Shoot_control();
      }

      inputBuffer = "";
    }
  }

  delay(10);
}

void Shoot_control(){
  const int speed1 = 150 + change_speed;
  const int speed2 = 200 + change_speed;
  const int speed3 = 220 + change_speed;

  char speed[100];

  if (Override==1){
    ledcWrite(MR_pwmChannel,0);
    ledcWrite(ML_pwmChannel,0);
    ledcWrite(MA_pwmChannel,0);
    Rotate_angle(initial_angle);
  }

  else if(Stop_mode==1){
    ledcWrite(MR_pwmChannel,0);
    ledcWrite(ML_pwmChannel,0);
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
    ledcWrite(MR_pwmChannel,0);
    ledcWrite(ML_pwmChannel,0);
    Rotate_angle(initial_angle);
  }

  if (buttons[14]==1 && Shoot_mode1==0 && Shoot_mode2==0 && Shoot_mode3==0 && Override==0){
    digitalWrite(ma, 0);
    ledcWrite(MA_pwmChannel,angle_change_speed);
    Serial.println("Angle increasing....");
  }

  else if (buttons[13]==1 && Shoot_mode1==0 && Shoot_mode2==0 && Shoot_mode3==0 && Override==0){
    digitalWrite(ma, 1);
    ledcWrite(MA_pwmChannel,angle_change_speed);
    Serial.println("Angle decreasing....");
  }
  else {
    ledcWrite(MA_pwmChannel, 0);
  }

  if (buttons[10]==1){
    delay(250);
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
  if(buttons[9]==1){
    delay(250);
    Override =! Override;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Shoot_mode3=0;
    Stop_mode=1;
    change_speed=0;
    Serial.print("Override:");
    Serial.println(Override);
  }
  else {
    Override = Override;
  }

  if(buttons[0]==1){
    delay(250);
    Shoot_mode1 = 1;
    Shoot_mode2=0;
    Shoot_mode3=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 1 activating with speed: 150");
  }
  else {
    Shoot_mode1 = Shoot_mode1;
  }

  if(buttons[1]==1){
    delay(250);
    Shoot_mode2 = 1;
    Shoot_mode1=0;
    Shoot_mode3=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 2 activating with speed: 200");
  }
  else {
    Shoot_mode2 = Shoot_mode2;
  }

  if(buttons[3]==1){
    delay(250);
    Shoot_mode3 = 1;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Stop_mode=0;
    change_speed=0;
    Serial.println("Shoot mode 3 activating with speed: 220");
  }
  else {
    Shoot_mode3 = Shoot_mode3;
  }

  if(buttons[2]==1){
    delay(250);
    Stop_mode = 1;
    Shoot_mode1=0;
    Shoot_mode2=0;
    Shoot_mode3=0;
    change_speed=0;
    Serial.println("Stop mode  activated stopping Flywheel");
  }
  else {
    Stop_mode = Stop_mode;
  }

  if(buttons[11]==1){
    delay(250);
    change_speed = change_speed + incr_dcr_const;
  }
  else {
    change_speed = change_speed;
  }

  if(buttons[12]==1){
    delay(250);
    change_speed = change_speed - incr_dcr_const;
  }
  else {
    change_speed = change_speed;
  }
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
  ledcWrite(MR_pwmChannel,speed);
  ledcWrite(ML_pwmChannel,speed);
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