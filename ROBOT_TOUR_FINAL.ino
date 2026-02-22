#include <MPU6050_tockn.h>
#include <Wire.h>


const int dirRight = 13; 
const int pwmRight = 11;
const int dirLeft = 12; 
const int pwmLeft = 3;


const int buttonPin = A0; // A0 is safe and far away from motor logic



//adjust for calibration
const float DISTANCE_CALIBRATION = 0.715819;
const float SENSOR_SCALE = 1.33; 





//wheel and motor specs
const float GEAR_RATIO = 75.0;      
const float WHEEL_DIAM_MM = 65.0;

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  mpu6050.begin();

  // filters vibrations and speed that the IMU can report at
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();



  pinMode(dirRight, OUTPUT); pinMode(pwmRight, OUTPUT);
  pinMode(dirLeft, OUTPUT);  pinMode(pwmLeft, OUTPUT);
  
  //sets up button pin
  pinMode(buttonPin, INPUT_PULLUP); 


  mpu6050.calcGyroOffsets(true);


  
 
  
  



  driveDistance(1.0, 150);
  delay(500);

  //driveDistance funct seems to tilt at like 1 Degree so I added 1 to the 90 degree turn
  turnDegrees(91);
  delay(500);
  



}

void loop() {
  mpu6050.update();




}


//deadlocks motors
void stopMotors(){
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, 0);
}



void driveDistance(float meters, int speed) {

  //meters calculated seem wrong so I did 1 meter / meter traveled to get the calibration correction
  float calibratedMeters = meters * DISTANCE_CALIBRATION;

  
  bool isForward = (meters > 0);
  
 
  int dirLeftVal  = isForward ? LOW  : HIGH; 
  int dirRightVal = isForward ? HIGH : LOW;
  int steerMultiplier = isForward ? 1 : -1;

  //KP is the correction amount, 9 is a good from what I've found
  const float KP = 9;      
  const int RIGHT_BIAS = 9.5; 

  //calcs for the meter conversion
  float circumferenceM = (WHEEL_DIAM_MM * 3.14159) / 1000.0;
  float metersPerSecond = ((speed / 255.0) * 135.0 / 60.0) * circumferenceM;
  unsigned long duration_ms = (abs(calibratedMeters) / metersPerSecond) * 1000;



  mpu6050.update();
  float targetAngle = mpu6050.getAngleZ(); 
  unsigned long startTime = millis();

  digitalWrite(dirLeft, dirLeftVal); 
  digitalWrite(dirRight, dirRightVal); 


  //keeps robot straight
  while (millis() - startTime < duration_ms) {
    mpu6050.update();
    float currentAngle = mpu6050.getAngleZ();
    float error = targetAngle - currentAngle;
    int adjustment = error * KP * steerMultiplier; 

    analogWrite(pwmLeft,  constrain(speed + adjustment, 0, 255));
    analogWrite(pwmRight, constrain(speed - adjustment + RIGHT_BIAS, 0, 255));
  }

  // active brake to minimize slippage
  digitalWrite(dirLeft, !dirLeftVal); 
  digitalWrite(dirRight, !dirRightVal);
  analogWrite(pwmLeft, 160);
  analogWrite(pwmRight, 160);
  delay(65);
  stopMotors();
}

void turnDegrees(float degreesToTurn) {

  const float STOP_EARLY_THRESHOLD = 1.2; 
  const int   START_PUNCH = 145; 
  const int   FINISH_CREEP = 80;   
  const float TRANSITION_DEG = 25.0; 
  
  stopMotors();
  mpu6050.update();
  float startAngle = mpu6050.getAngleZ();

  //bool that detecks if it has to turn right or left
  bool turningRight = (degreesToTurn > 0);
  if (turningRight) {
    digitalWrite(dirLeft, LOW); digitalWrite(dirRight, LOW); 
  } else {
    digitalWrite(dirLeft, HIGH); digitalWrite(dirRight, HIGH); 
  }

  //makes sure that the time is very accurate (in milli seconds)
  unsigned long startTime = millis();

  while(true) {
    mpu6050.update();
    float currentPhysical = (mpu6050.getAngleZ() - startAngle) * SENSOR_SCALE;
    float absError = abs(degreesToTurn - currentPhysical);

    if (absError <= STOP_EARLY_THRESHOLD) break;

  //adjusts the speed based off the point in the turn
    int speed;
    if (millis() - startTime < 150) speed = START_PUNCH;
    else if (absError < TRANSITION_DEG) speed = FINISH_CREEP;
    else speed = 110;

    analogWrite(pwmLeft, speed);
    analogWrite(pwmRight, speed);
  }

  // other active brake also to prevent slippage
  digitalWrite(dirLeft, !digitalRead(dirLeft)); 
  digitalWrite(dirRight, !digitalRead(dirRight));
  analogWrite(pwmLeft, 160); 
  analogWrite(pwmRight, 160);
  delay(75); 
  stopMotors();
}