#include <MPU6050_tockn.h>
#include <Wire.h>

// --- PHYSICAL SPECS ---
const float GEAR_RATIO = 75.0;
const float WHEEL_DIAM_MM = 65.0;
const float ENCODER_PPR = 11.0;
const float LEFT_MOTOR_BIAS = 1; // Your left motor is stronger

// --- PIN DEFINITIONS (Your Swapped Controls) ---
const int dirRight = 13; const int pwmRight = 11;
const int dirLeft = 12;  const int pwmLeft = 3;
const int encLeftA = 2;

MPU6050 mpu6050(Wire);

volatile long leftCount = 0;
float ticksPerMeter;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Math for distance
  ticksPerMeter = (ENCODER_PPR * GEAR_RATIO) / ((WHEEL_DIAM_MM * 3.14159) / 1000.0);

  mpu6050.begin();
  Serial.println(F("Initializing IMU... Keep robot still."));
  mpu6050.calcGyroOffsets(true);

  pinMode(dirRight, OUTPUT); pinMode(pwmRight, OUTPUT);
  pinMode(dirLeft, OUTPUT);  pinMode(pwmLeft, OUTPUT);
  
  // Encoder Interrupt
  attachInterrupt(digitalPinToInterrupt(encLeftA), [](){ leftCount++; }, RISING);

  delay(3000); // 3 second safety delay

  // --- EXAMPLE MISSION ---
  driveForward(0.5, 160);
  delay(1000); 
  turnDegrees(90 -1 );  // This will now use Successive Approximation
  delay(500); 
  turnDegrees(90 -1);

  driveForward(0.5, 160);
  delay(1000);
  turnDegrees(90 -1);
  delay(500); 
  turnDegrees(90 -1);
}

void loop() {
  // Main logic is in setup for a one-time run
}

// --- DRIVE FUNCTIONS ---

void driveForward(float meters, int maxSpeed) {
  executeMove(meters, maxSpeed, true);
}

void driveBackwards(float meters, int maxSpeed) {
  executeMove(meters, maxSpeed, false);
}

void executeMove(float meters, int maxSpeed, bool forward) {
  long targetTicks = meters * ticksPerMeter;
  long startTicks = leftCount;
  
  mpu6050.update();
  float driveHeading = mpu6050.getAngleZ();

  while (abs(leftCount - startTicks) < targetTicks) {
    mpu6050.update();
    float error = driveHeading - mpu6050.getAngleZ();
    
    // Deceleration Logic
    long ticksRemaining = targetTicks - abs(leftCount - startTicks);
    int currentSpeed = maxSpeed;
    if (ticksRemaining < 1200) {
      currentSpeed = map(ticksRemaining, 0, 1200, 95, maxSpeed);
    }

    // Steering correction (Flipped if backing up)
    int correction = error * (forward ? 5 : -5);

    // Set Motor Directions
    if (forward) {
      digitalWrite(dirRight, HIGH); digitalWrite(dirLeft, LOW);
    } else {
      digitalWrite(dirRight, LOW); digitalWrite(dirLeft, HIGH);
    }
    
    int leftPWM = (currentSpeed + correction) * LEFT_MOTOR_BIAS;
    int rightPWM = (currentSpeed - correction);
    
    analogWrite(pwmLeft, constrain(leftPWM, 0, 255));
    analogWrite(pwmRight, constrain(rightPWM, 0, 255));
  }
  stopMotors();
}

// --- TURNING WITH SUCCESSIVE APPROXIMATION ---

void turnDegrees(float degreesToTurn) {
  mpu6050.update();
  float targetAngle = mpu6050.getAngleZ() + degreesToTurn;

  Serial.print(F("Turning to: ")); Serial.println(targetAngle);

  // 1. Initial "Fast" Turn
  pivotToAngle(targetAngle, 120); 

  // 2. Successive Approximation (The Nudge Loop)
  // It will check 4 times to see if it needs to nudge closer
  for (int i = 0; i < 4; i++) {
    delay(500); // Wait for robot to stop sliding
    mpu6050.update();
    float currentAngle = mpu6050.getAngleZ();
    float error = targetAngle - currentAngle;

    // If within 1 degree, we are happy
    if (abs(error) < 0.5) {
      Serial.println(F("Target Reached."));
      break; 
    }

    // If still off, do a tiny slow "nudge"
    Serial.print(F("Nudging. Error: ")); Serial.println(error);
    pivotToAngle(targetAngle, 100); 
  }
  stopMotors();
}

void pivotToAngle(float target, int speed) {
  while (true) {
    mpu6050.update();
    float now = mpu6050.getAngleZ();
    float error = target - now;

    // Stop if we hit the mark (with a tiny 0.5 buffer to prevent infinite loop)
    if (abs(error) < 0.8) break;

    bool turningRight = (error > 0);

    if (turningRight) {
      digitalWrite(dirLeft, LOW); digitalWrite(dirRight, LOW);
    } else {
      digitalWrite(dirLeft, HIGH); digitalWrite(dirRight, HIGH);
    }

    analogWrite(pwmLeft, (float)speed * LEFT_MOTOR_BIAS);
    analogWrite(pwmRight, speed);
  }
  stopMotors();
}

void stopMotors() {
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, 0);
}