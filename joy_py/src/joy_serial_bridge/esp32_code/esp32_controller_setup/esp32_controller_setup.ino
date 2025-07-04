/*
 * ESP32 Joystick Data Receiver with Basic Motor Control
 * Uses axis[0] for left/right and axis[1] for forward/back
 * Format from ROS: "axis_values;button_values"
 */
bool Override = true;

const int MAX_AXES = 8;
float axes[MAX_AXES] = {0.0};

const int MAX_BUTTONS = 15;
int buttons[MAX_BUTTONS] = {0};

// pwm
int M1 = 13; //  Front
int M2 = 27; // left
int M3 = 4;  // back
int M4 = 21; // right
// direction
int m1 = 12; // front
int m2 = 26; // left
int m3 = 2;  // back
int m4 = 19; // right

//rotation speed
const int rot_speed = 100;

void setup() {
  Serial.begin(115200);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);

  Serial.println("ESP32 Joy Receiver started");
  Serial.println("Waiting for data from ROS...");
}

void loop() {
  check_override();
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

        controlMotors(axes[0], axes[1],axes[4],axes[5]);
      }

      inputBuffer = "";
    }
  }

  delay(10);
}

void controlMotors(float xAxis, float yAxis, float rAnticlock, float rClock) {
  float x = xAxis * 255;
  float y = yAxis * 255;
  float rotAnticlock = map(-rAnticlock, -1, 1, 0, rot_speed);
  float rotclock = map(-rClock, -1, 1, 0, rot_speed);
  
    
  if(Override==1){
    analogWrite(M1, 0);
    analogWrite(M2, 0);
    analogWrite(M3, 0);
    analogWrite(M4, 0);
  }

  else if (x > 0 && y > 0) {
    //considering Left as positive and front as positive
    digitalWrite(m1, 0);
    digitalWrite(m2, 0);
    digitalWrite(m3, 0);
    digitalWrite(m4, 0);
    analogWrite(M1, x);
    analogWrite(M2, y);
    analogWrite(M3, x);
    analogWrite(M4, y);
    Serial.println("Moving in Secound quadrant");

  } 
   else if (x > 0 && y < 0) {
    digitalWrite(m1, 0);
    digitalWrite(m2, 1);
    digitalWrite(m3, 0);
    digitalWrite(m4, 1);
    analogWrite(M1,  x);
    analogWrite(M2, -y);
    analogWrite(M3,  x);
    analogWrite(M4, -y);
    Serial.println("Moving in Third quadrant");

  } else if(x < 0 && y > 0) {
    digitalWrite(m1, 1);
    digitalWrite(m2, 0);
    digitalWrite(m3, 1);
    digitalWrite(m4, 0);
    analogWrite(M1, -x);
    analogWrite(M2,  y);
    analogWrite(M3, -x);
    analogWrite(M4,  y);
    Serial.println("Moving in First quadrant");
  } 

 else if (x < 0 && y < 0) {
    digitalWrite(m1, 1);
    digitalWrite(m2, 1);
    digitalWrite(m3, 1);
    digitalWrite(m4, 1);
    analogWrite(M1, -x);
    analogWrite(M2, -y);
    analogWrite(M3, -x);
    analogWrite(M4, -y);
    Serial.println("Moving in Forth quadrant");
  }

  else if (x > 0 && y == 0) {
    digitalWrite(m1, 0);
    digitalWrite(m3, 0);
    analogWrite(M1, x);
    analogWrite(M2, 0);
    analogWrite(M3, x);
    analogWrite(M4, 0);
    Serial.println("Moving on -ve x-axis");
  }

  else if (x < 0 && y == 0) {
    digitalWrite(m1, 1);
    digitalWrite(m3, 1);
    analogWrite(M1, -x);
    analogWrite(M2,  0);
    analogWrite(M3, -x);
    analogWrite(M4,  0);
    Serial.println("Moving on +ve x-axis");
  }

  else if (x == 0 && y > 0) {
    digitalWrite(m2, 0);
    digitalWrite(m4, 0);
    analogWrite(M1, 0);
    analogWrite(M2, y);
    analogWrite(M3, 0);
    analogWrite(M4, y);
    Serial.println("Moving on +ve y-axis");
  }

  else if (x == 0 && y < 0) {
    digitalWrite(m2, 1);
    digitalWrite(m4, 1);
    analogWrite(M1,  0);
    analogWrite(M2, -y);
    analogWrite(M3,  0);
    analogWrite(M4, -y);
    Serial.println("Moving on -ve y-axis");
  }

  else if (rotAnticlock > 0){
    digitalWrite(m1, 0);
    digitalWrite(m2, 0);
    digitalWrite(m3, 1);
    digitalWrite(m4, 1);
    analogWrite(M1, rotAnticlock);
    analogWrite(M2, rotAnticlock);
    analogWrite(M3, rotAnticlock);
    analogWrite(M4, rotAnticlock);
    Serial.println("Rotating Anti clockwise");
  }

  else if (rotclock > 0){
    digitalWrite(m1, 1);
    digitalWrite(m2, 1);
    digitalWrite(m3, 0);
    digitalWrite(m4, 0);
    analogWrite(M1, rotclock);
    analogWrite(M2, rotclock);
    analogWrite(M3, rotclock);
    analogWrite(M4, rotclock);
    Serial.println("Rotating  Clockwise");
  }  

  else {
    // Stop
    analogWrite(M1, 0);
    analogWrite(M2, 0);
    analogWrite(M3, 0);
    analogWrite(M4, 0);}

}

void check_override(){
  if(buttons[9]==1){
    delay(250);
    Override =! Override;
    Serial.print("Override:");
    Serial.println(Override);
  }
  else {
    Override = Override;
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