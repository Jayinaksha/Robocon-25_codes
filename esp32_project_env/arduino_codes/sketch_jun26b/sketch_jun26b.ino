#include <ESP32Servo.h>
#define m1_dig 11
#define m1_pwm 12
#define m2_dig 13
#define m2_pwm 14

const int speed = 180;

Servo myServo;
int dir = 0; // 0 = 0 degrees, 1 = 180 degrees
int angle = 0;

int servoPin = 23;  // Connect your servo signal wire to GPIO 23

void setup() {
  Serial.begin(115200);
  myServo.setPeriodHertz(50); // Standard 50Hz for servos
  myServo.attach(servoPin, 150, 2400); // min and max pulse width in µs
  Serial.println("Servo test start");
  myServo.write(0);
}

void loop() {
  // Read Serial input to change direction
  if (Serial.available()) {
    char input = Serial.read();
    if (input == 'a') {
      dir = 1;  // Set direction to 180
      Serial.println("Moving to 180°");
    } else if (input == 'b') {
      dir = 0;  // Set direction to 0
      Serial.println("Moving to 0°");
    }
  }

  if (Serial.available()){
    char input2 = Serial.read();
    if(input2 == 'r'){
      digitalWrite(m1_dig,1);
      digitalWrite(m2_dig,1);
      analogWrite(m1_pwm,speed);
      analogWrite(m2_pwm,speed);
    }
    else if (input2 == 's'){
      digitalWrite(m1_dig,1);
      digitalWrite(m2_dig,1);
      analogWrite(m1_pwm,0);
      analogWrite(m2_pwm,0);
    }
  }

  // Sweep from 0 to 180
  if (dir == 1 && angle != 180) {
    for (angle = angle; angle <= 180; angle += 5) {
      myServo.write(angle);
      delay(5);
    }
  }

  // Sweep from 180 to 0
  if (dir == 0 && angle != 0) {
    for (angle = angle; angle >= 0; angle -= 5) {
      myServo.write(angle);
      delay(5);
    }
  }

  delay(50); // Short delay to avoid overwhelming the servo
}