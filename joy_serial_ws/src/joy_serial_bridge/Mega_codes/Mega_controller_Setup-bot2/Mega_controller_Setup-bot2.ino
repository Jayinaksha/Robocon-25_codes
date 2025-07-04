/* for bot 1
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
int M1 = 5;  // Front
int M2 = 7; // left
int M3 = 9; // back
int M4 = 11; // right
// direction
int m1 = 24;  // front
int m2 = 28; // left
int m3 = 32; // back
int m4 = 36; // right

//rotation speed
const int rot_speed = 100;
const int movement_speed = 180;

//Serial parsing
String inputBuffer = "";
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

  Serial.println("Arduino Joy Receiver started");
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

  check_override(); // Handle override toggle
  delay(2); // Small delay to yield CPU; no blocking delays!
}

// -- PROCESS FULL LINE --
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
  controlMotors(axes[0], axes[1], axes[4], axes[5]);
  }
}


void controlMotors(float xAxis, float yAxis, float rAnticlock, float rClock) {
  float x = xAxis * movement_speed;
  float y = yAxis * movement_speed;
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
  static int last_override_button = 0; 
  static bool toggled_on_this_press = false;
  static unsigned long last_toggle_time = 0;
  const unsigned long debounce_ms = 200;

  unsigned long now = millis();
  if (buttons[9] == 1 && last_override_button == 0 && !toggled_on_this_press) {
    if (now - last_toggle_time > debounce_ms) {
      Override = !Override;
      Serial.print("Buttons9:");
      Serial.println(buttons[9]);
      Serial.print("Override:");
      Serial.println(Override);
      last_toggle_time = now;
      toggled_on_this_press = true;
    }
  }
  if (buttons[9] == 0) {
    toggled_on_this_press = false; // allow next toggle only after release
  }
  last_override_button = buttons[9];
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