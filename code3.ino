// Code 3: Testing VL53L0X LiDAR and GY-521 MPU6050.
// If the robot detects an obstacle infront of it by TOF200C-VL53L0X LiDAR, 
// Check for an open path to the left or right according to HW-201 IR sensors,
// Rotate 90 degrees (approximately) according to GY-521 MPU6050.

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

VL53L0X lidar;  // Initialize LiDAR
MPU6050 mpu;    // Initialize MPU6050

float yaw = 0; // Stores the yaw angle
unsigned long lastTime = 0;

// pins => motor 1
const int encoderPinC1 = 16;
const int encoderPinC2 = 23;
const int enablePin = 14;
const int input1Pin = 26;
const int input2Pin = 27;

// pins => motor 2
const int encoder2PinC1 = 17;
const int encoder2PinC2 = 4;
const int enable2Pin = 25;
const int input3Pin = 33;
const int input4Pin = 32;

// IR sensors
const int irLeftPin = 34;
const int irRightPin = 35;

// Thresholds
const int irThreshold = 300;
const int lidarThreshold = 200;
const int motorSpeed = 255;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!lidar.init()) {
    Serial.println("Failed to detect LiDAR sensor!");
    while (1);
  }
  lidar.setTimeout(500); // If the sensor fails to obtain a distance reading within 500 milliseconds, it will cancel the measurement rather than waiting indefinitely.
  lidar.startContinuous(); // LiDAR will continuously update distance readings 

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  calibrateMPU();

  pinMode(enablePin, OUTPUT);
  pinMode(input1Pin, OUTPUT);
  pinMode(input2Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  pinMode(input3Pin, OUTPUT);
  pinMode(input4Pin, OUTPUT);

  pinMode(irLeftPin, INPUT);
  pinMode(irRightPin, INPUT);

  digitalWrite(enablePin, HIGH);
  digitalWrite(enable2Pin, HIGH);
  moveForward();
}

// Ensure accurate deviation measurements by preventing accumulated errors and providing a stable starting point before each cycle.
void calibrateMPU() {
  yaw = 0; // Reset the yaw angle to zero before starting a new rotation measurement.
  lastTime = millis();
}

// Calculate the change in yaw (rotation angle) based on gyroscope data
float getYawChange() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz); // Reading gyroscope values ​​
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Compute time difference in seconds
  lastTime = currentTime;
  return (gz / 131.0) * dt; // Convert gyroscope Z-axis reading to degrees and return the yaw change
}

// Rotate the robot 90 degrees left or right 
void rotate90(bool left) {
  Serial.println(left ? "Turning LEFT..." : "Turning RIGHT...");
  float targetAngle = left ? -90 : 90; // Set target angle (left (-90°) or right (90°))
  yaw = 0;

  digitalWrite(input1Pin, left ? LOW : HIGH);
  digitalWrite(input2Pin, left ? HIGH : LOW);
  digitalWrite(input3Pin, left ? LOW : HIGH);
  digitalWrite(input4Pin, left ? HIGH : LOW);

  // Rotate until reaching the target angle
  while (abs(yaw) < abs(targetAngle)) {
    yaw += getYawChange();
    delay(10);
  }
  stopMotors(); // Once the target angle is reached, stop the motors
  moveForward(); // Now move forward
}

// Stop motors 
void stopMotors() {
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, LOW);
  Serial.println("Motors Stopped.");
}

// Move both motors together froward
void moveForward() {
  digitalWrite(input1Pin, HIGH);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, HIGH);
  Serial.println("Moving Forward...");
}

// Move bot motors together backward
void moveBackward() {
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, HIGH);
  digitalWrite(input3Pin, HIGH);
  digitalWrite(input4Pin, LOW);
  Serial.println("Moving Backward...");
}

void loop() {
  int irLeftValue = analogRead(irLeftPin); // Read left IR sensor's value
  int irRightValue = analogRead(irRightPin); // Read right IR sensor's value
  int lidarDistance = lidar.readRangeContinuousMillimeters(); // Read LiDAR distance detection 

  Serial.print("LiDAR Distance: "); Serial.println(lidarDistance);

  // Decide where to rotate according to the sensors readings
  if (lidarDistance < lidarThreshold) {
    stopMotors();
    delay(500);
    if (irLeftValue > irThreshold) {
      rotate90(true);  // Turn Left
    } else if (irRightValue > irThreshold) {
      rotate90(false); // Turn Right
    } else {
      // If no open path, move backward
      moveBackward();
      delay(500);
      stopMotors();
    }
  } else {
    moveForward();
  }
  delay(100);
}
