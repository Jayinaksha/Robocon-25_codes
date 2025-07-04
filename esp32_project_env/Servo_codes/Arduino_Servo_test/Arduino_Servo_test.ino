#include <Servo.h>

Servo myServo;

const int servoPin = 5;
int currentAngle = 0;

void setup() {
  Serial.begin(115200);

  //myServo.setPeriodHertz(50);  // Standard 50 Hz servo
  myServo.attach(servoPin);  // Set min/max pulse width for your servo

  myServo.write(currentAngle);
  Serial.println("Initialized at 120 degrees. Send 'a' or 'b'.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'a') {
      rotateToAngle(0);
    } else if (command == 'b') {
      rotateToAngle(100);
    }
  }
}

void rotateToAngle(int targetAngle) {
  if (currentAngle == targetAngle) return;

  int step = (targetAngle > currentAngle) ? 5 : -5;

  while (currentAngle != targetAngle) {
    currentAngle += step;
    myServo.write(currentAngle);
    delay(5);
  }

  Serial.print("Reached ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
}