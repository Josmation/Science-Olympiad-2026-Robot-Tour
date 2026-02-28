#include <MPU6050_tockn.h>
#include <Wire.h>

// Motor shield fixed pin assignments
// Channel A (left motor)
const int pwmLeft   = 3;
const int dirLeft   = 12;
const int brakeLeft = 9;
// Channel B (right motor)
const int pwmRight   = 11;
const int dirRight   = 13;
const int brakeRight = 8;

const int buttonPin = 0; // A0

// Encoder pins, yellow wire = a, green wire = b 
const int ENC_LEFT_A  = 2;
const int ENC_LEFT_B  = 4;
const int ENC_RIGHT_A = 7;
const int ENC_RIGHT_B = 5;

//motor and wheel consts
const float COUNTS_PER_REV   = 825.0;
const float WHEEL_DIAM_MM    = 65.0;
const float WHEEL_CIRCUM_M   = (WHEEL_DIAM_MM * 3.14159265) / 1000.0;
const float METERS_PER_COUNT = WHEEL_CIRCUM_M / COUNTS_PER_REV;

const float SENSOR_SCALE = 1.33;

// SIX SEVENNNNNN!!!!!!
const float DISTANCE_CALIBRATION = .609;

// Steering tuning
const float KP         = 9.0;
const float RIGHT_BIAS = 9.5;

// Ramping
const long RAMP_UP_COUNTS   = 150;
const long RAMP_DOWN_COUNTS = 200;
const int  MIN_SPEED        = 60;

MPU6050 mpu6050(Wire);

void setup() {
  //serial for printing to console
  Serial.begin(115200);
  Wire.begin();
  
  Wire.setClock(400000);
  mpu6050.begin();

  // makes the imu better for the code, unrestricts the momentum and makes it filter out noise
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();


  //sets the pinmodes (in or out) for all of the pins
  pinMode(dirRight,   OUTPUT); pinMode(pwmRight,   OUTPUT);
  
  pinMode(dirLeft,    OUTPUT); pinMode(pwmLeft,    OUTPUT);
  pinMode(brakeLeft,  OUTPUT); pinMode(brakeRight, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);

  // Brakes OFF on startup
  digitalWrite(brakeLeft,  LOW);
  
  digitalWrite(brakeRight, LOW);

  // Encoder pins - plain inputs, no interrupts
  pinMode(ENC_LEFT_A,  INPUT_PULLUP);
  
  pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
}

// =============================================================
void loop() {
  mpu6050.update();


  
//button push that starts the whole path
  if (digitalRead(buttonPin) == LOW) {
    delay(1000);
    mpu6050.calcGyroOffsets(true);

    //driveDistance(.25, 150);
    
    //driveDistance(1.112, 120);(1METER)
    //1m
            
    //driveDistance(1.1557, 120); 1METER adj for turn

   

    //driveDistance(.54, 120);half meter

    //driveDistance(.586, 120); half meter adj for turn

    //driveDistance(.586, 120);

    //driveDistance(.25, 90); 1/4 meter
    
    //driveDistance(.29, 90); 1/4 meter adj for turn


    //delay(500);

    //turnDegrees(76);

    //delay(500);

   
    
    //90degrees
    //turnDegrees(76);

    
    //delay(500);

    //driveDistance(1.112, 120);

    
  }
}


//trying to move half then turn 90 moves it away from 90
//so I am making a function that when called, moves the robot to adjust for how much the 90 will deviate it from the original coords
//example, driveDistance(1, 200); turnDegrees(92); doesn't actuall moves it 1m then turn 90 in the exact location
//adjusted examples, driveDistance(1,200) adjustForTurn();, turnDegrees(92);

//find the turn distance deviation first
void adjustForTurn(){


 driveDistance(-.02, 200);

  
}
// function to stop motors so we get a clean motor call next time
void stopMotors() {

  
  analogWrite(pwmLeft,  0);
  analogWrite(pwmRight, 0);
  // Ensure brakes are off so motors can spin freely next call
  digitalWrite(brakeLeft,  LOW);
  digitalWrite(brakeRight, LOW);

  
}

// creats the different speeds for each interval (LOW,HIGH,LOW)
int rampedSpeed(long avgCounts, long targetCounts, int topSpeed) {
  if (avgCounts < RAMP_UP_COUNTS) {
    float t = (float)avgCounts / RAMP_UP_COUNTS;
    
    return (int)(MIN_SPEED + t * (topSpeed - MIN_SPEED));
  }
  long countsRemaining = targetCounts - avgCounts;
  if (countsRemaining < RAMP_DOWN_COUNTS) {

    
    float t = (float)countsRemaining / RAMP_DOWN_COUNTS;
    return (int)(MIN_SPEED + t * (topSpeed - MIN_SPEED));
  }
  return topSpeed;
}

//drive distance function that makes the robot g X distance, had to switch from time based to tick based with encoders
//also auto corrects path to be straight
void driveDistance(float meters, int speed) {
  bool isForward = (meters > 0);

  int dirLeftVal  = isForward ? LOW  : HIGH;
  int dirRightVal = isForward ? HIGH : LOW;
  
  int steerMult   = isForward ? 1    : -1;

  long targetCounts = (long)(abs(meters) * DISTANCE_CALIBRATION / METERS_PER_COUNT);

  long leftCount  = 0;
  long rightCount = 0;

  bool prevLeftA  = digitalRead(ENC_LEFT_A);
  bool prevRightA = digitalRead(ENC_RIGHT_A);

  mpu6050.update();
  float targetAngle = mpu6050.getAngleZ();

  digitalWrite(brakeLeft,  LOW);
  digitalWrite(brakeRight, LOW);
  digitalWrite(dirLeft,    dirLeftVal);
  
  digitalWrite(dirRight,   dirRightVal);

  analogWrite(pwmLeft,  MIN_SPEED);
  analogWrite(pwmRight, MIN_SPEED);

  unsigned long startTime     = millis();
  unsigned long lastGyroTime  = millis();
  unsigned long lastPrintTime = millis();

  const unsigned long TIMEOUT_MS = 15000;
  
  const unsigned long GYRO_MS    = 20;
  const unsigned long PRINT_MS   = 200;

  float error = 0;

  while (true) {
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.println("TIMEOUT - stopping motors");
      break;
    }

    // Poll left encoder
    bool currLeftA = digitalRead(ENC_LEFT_A);
    if (currLeftA && !prevLeftA) {
      if (digitalRead(ENC_LEFT_B) == LOW) leftCount++;
      else                                 leftCount--;
    }
    prevLeftA = currLeftA;

    // Poll right encoder
    bool currRightA = digitalRead(ENC_RIGHT_A);
    if (currRightA && !prevRightA) {
      if (digitalRead(ENC_RIGHT_B) == HIGH) rightCount++;
      else                                   rightCount--;
    }
    prevRightA = currRightA;

    long avgCounts = (abs(leftCount) + abs(rightCount)) / 2;
    if (avgCounts >= targetCounts) break;

    if (millis() - lastGyroTime >= GYRO_MS) {
      lastGyroTime = millis();
      mpu6050.update();
      error = targetAngle - mpu6050.getAngleZ();

      int baseSpeed  = rampedSpeed(avgCounts, targetCounts, speed);
      int adjustment = (int)(error * KP * steerMult);

      analogWrite(pwmLeft,  constrain(baseSpeed + adjustment,                   0, 255));
      analogWrite(pwmRight, constrain(baseSpeed - adjustment + (int)RIGHT_BIAS, 0, 255));
    }

    if (millis() - lastPrintTime >= PRINT_MS) {
      lastPrintTime = millis();
      Serial.print("L: "); Serial.print(leftCount);
      
      Serial.print("  R: "); Serial.print(rightCount);
      Serial.print("  avg: "); Serial.print(avgCounts);
      Serial.print(" / "); Serial.println(targetCounts);
    }
  }

  // Active brake using shield brake pins
  digitalWrite(brakeLeft,  HIGH);
  digitalWrite(brakeRight, HIGH);
  delay(65);
  stopMotors(); // also releases brakes
}

//function that turns X degreees. Ill probably only use 90 degrees (also 90 to get 90 degrees I have to input 92.5, most likely due to friction and stopping threshold)
void turnDegrees(float degreesToTurn) {
  const float STOP_EARLY_THRESHOLD = 1.2;
  const int   START_PUNCH          = 145;
  const int   FINISH_CREEP         = 80;
  
  const float TRANSITION_DEG       = 25.0;

  stopMotors();
  
  digitalWrite(brakeLeft,  LOW);
  digitalWrite(brakeRight, LOW);

  mpu6050.update();
  float startAngle = mpu6050.getAngleZ();

  bool turningRight = (degreesToTurn > 0);
  if (turningRight) {
    digitalWrite(dirLeft, LOW);  digitalWrite(dirRight, LOW);
  } else {
    digitalWrite(dirLeft, HIGH); digitalWrite(dirRight, HIGH);
  }

  unsigned long startTime = millis();


  
  while (true) {
    mpu6050.update();
    float currentPhysical = (mpu6050.getAngleZ() - startAngle) * SENSOR_SCALE;
    float absError        = abs(degreesToTurn - currentPhysical);

    if (absError <= STOP_EARLY_THRESHOLD) break;

    int spd;
    if      (millis() - startTime < 150) spd = START_PUNCH;
    else if (absError < TRANSITION_DEG)  spd = FINISH_CREEP;
    else                                 spd = 110;

    analogWrite(pwmLeft,  spd);
    
    analogWrite(pwmRight, spd);
  }

  // Active brake
  digitalWrite(brakeLeft,  HIGH);
  digitalWrite(brakeRight, HIGH);
  delay(75);
  stopMotors(); // releases brakes
}
