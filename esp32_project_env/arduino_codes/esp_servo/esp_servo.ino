#include <ESP32Servo.h>
#include "BluetoothSerial.h"
#include <Wire.h>

// Check if Bluetooth is enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

// Device name
const String BT_DEVICE_NAME = "Main-test";

// MPU9250 I2C address
const int MPU9250_ADDRESS = 0x68;

// Motor pins
const int M1 = 17;
const int m1 = 18;
const int M2 = 19;
const int m2 = 25;
const int M3 = 33;
const int m3 = 32;

// Relay pin
const int RELAY_PIN = 27;

// Kalman filter variables
float pitch = 0.0;
float bias = 0.0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.01;
float Q_bias = 0.003;
float R_measure = 0.03;

float avg = 0;
int count = 0;
float setupangle = 0.0;
unsigned long lastTime = 0;

int threshold = 0;
int motorPWM = 150;  // default PWM
bool thresholdSet = false;
bool btConnected = false;

void motorstop() {
  analogWrite(M2, 0);
  analogWrite(M3, 0);
}

void stop() {
  analogWrite(M1, 0);
}

void clockwise() {
  digitalWrite(m1, LOW);
  analogWrite(M1, 200);
}

void anticlockwise() {
  digitalWrite(m1, HIGH);
  analogWrite(M1, 200);
}

void start() {
  digitalWrite(m2, 1);
  digitalWrite(m3, 0);
  analogWrite(M2, motorPWM);
  analogWrite(M3, motorPWM);
}

void setupMPU9250() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("MPU9250 not found at address 0x%02X. Error: %d\n", MPU9250_ADDRESS, error);
    Serial.println("Check your connections. Continuing anyway...");
  } else {
    Serial.println("MPU9250 found!");
  }
  
  writeMPU9250(MPU9250_ADDRESS, 0x6B, 0x00); // Wake up MPU
  writeMPU9250(MPU9250_ADDRESS, 0x1B, 0x00); // Gyro ±250°/s
  writeMPU9250(MPU9250_ADDRESS, 0x1C, 0x00); // Accel ±2g
  
  Serial.println("MPU9250 initialized successfully");
}

void calib() {
  Serial.println("Starting calibration...");
  count = 0;
  avg = 0;
  
  while (count <= 100) {
    readAndEstimatePitch();
    avg += pitch;
    count++;
    delay(5);
  }
  avg /= count;
  
  Serial.printf("Calibration complete. Baseline: %.2f°\n", avg);
}

void setup() {
  Serial.begin(115200);
  
  // Wait for serial to be ready
  delay(1000);
  Serial.println("\n\nSystem starting...");
  
  // Initialize Bluetooth with improved error handling
  Serial.println("Initializing Bluetooth...");
  
  if (!SerialBT.begin(BT_DEVICE_NAME)) {
    Serial.println("ERROR: Failed to initialize Bluetooth");
  } else {
    Serial.println("SUCCESS: Bluetooth device started, pair with '" + BT_DEVICE_NAME + "'");
  }
  
  // Alternative serial connection method (if Bluetooth fails)
  Serial.println("NOTE: If Bluetooth fails, you can control via Serial Monitor");
  Serial.println("Commands: J[angle] = threshold, B = relay off, F = relay on, K[value] = motor PWM");

  // Initialize I2C
  Serial.println("Initializing I2C...");
  Wire.begin(21, 22);  // SDA, SCL
  
  // Initialize pins
  pinMode(M1, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, 1); // Relay initially OFF
  
  Serial.println("Initializing MPU9250...");
  setupMPU9250();
  
  lastTime = millis();
  calib();  // Calibrate pitch baseline
  
  Serial.println("Setup complete, entering main loop");
}

void loop() {
  // Check Bluetooth connection periodically
  static unsigned long lastBtCheck = 0;
  if (millis() - lastBtCheck >= 5000) { // Every 5 seconds
    btConnected = SerialBT.hasClient();
    if (btConnected) {
      Serial.println("Bluetooth is connected");
    } else {
      Serial.println("Waiting for Bluetooth connection...");
    }
    lastBtCheck = millis();
  }
  
  // Process Bluetooth commands
  processCommands();
  
  // Read sensor and update pitch
  readAndEstimatePitch();
  float corrected_pitch = abs(pitch - avg + setupangle);

  // Debug print every 100 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    Serial.printf("Pitch (Kalman): %.2f°\n", corrected_pitch);
    lastPrint = millis();
  }

  // Motor control based on pitch
  if (thresholdSet) {
    if (corrected_pitch > threshold + 3) {
      clockwise();
    } else if (corrected_pitch < threshold - 3) {
      anticlockwise();
    } else {
      stop();
    }
  } else {
    stop();  // Default safe state
  }

  delay(10);  // ~100 Hz
}

// Unified command processing function to handle both Serial and Bluetooth commands
void processCommands() {
  // Process Bluetooth commands
  while (SerialBT.available() > 0) {
    String input = SerialBT.readStringUntil('\n');
    input.trim();
    Serial.print("BT Received: ");
    Serial.println(input);
    processCommand(input, true);
  }
  
  // Also process Serial commands (as a fallback)
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.print("Serial Received: ");
    Serial.println(input);
    processCommand(input, false);
  }
}

void processCommand(String input, bool fromBluetooth) {
  if (input.startsWith("J")) {
    String val = input.substring(1);
    threshold = val.toInt();
    thresholdSet = true;
    String response = "Threshold set to: " + String(threshold);
    sendResponse(response, fromBluetooth);
  } 
  else if (input == "B") {
    digitalWrite(RELAY_PIN, 0);
    sendResponse("Relay OFF", fromBluetooth);
  } 
  else if (input == "F") {
    digitalWrite(RELAY_PIN, 1);
    sendResponse("Relay ON", fromBluetooth);
  } 
  else if (input.startsWith("K")) {
    String val = input.substring(1);
    motorPWM = constrain(val.toInt(), 0, 255);
    start();
    String response = "Motor PWM set to: " + String(motorPWM);
    sendResponse(response, fromBluetooth);
  }
  else if (input == "STATUS") {
    String status = "System status:\n";
    status += "Threshold: " + String(threshold) + " (Set: " + (thresholdSet ? "Yes" : "No") + ")\n";
    status += "Motor PWM: " + String(motorPWM) + "\n";
    status += "Relay: " + String(digitalRead(RELAY_PIN) ? "ON" : "OFF") + "\n";
    sendResponse(status, fromBluetooth);
  }
}

void sendResponse(String message, bool toBluetooth) {
  // Always print to Serial for debugging
  Serial.println(message);
  
  // If command came from Bluetooth, send response back via Bluetooth
  if (toBluetooth) {
    SerialBT.println(message);
  }
}

void readAndEstimatePitch() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);
  if(Wire.endTransmission(false) != 0) {
    Serial.println("Error: Failed to communicate with MPU9250");
    return;
  }
  
  if(Wire.requestFrom(MPU9250_ADDRESS, 14, true) != 14) {
    Serial.println("Error: Could not get complete data from MPU9250");
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip gy
  Wire.read(); Wire.read(); // Skip gz

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gyro_x = gx / 131.0;

  float pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  kalmanUpdate(pitch_acc, gyro_x, dt);
}

void kalmanUpdate(float pitch_meas, float gyro_rate, float dt) {
  pitch += dt * (gyro_rate - bias);
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float y = pitch_meas - pitch;
  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  pitch += K0 * y;
  bias += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;
}

void writeMPU9250(byte address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.printf("Error writing to MPU9250: %d\n", error);
  }
}