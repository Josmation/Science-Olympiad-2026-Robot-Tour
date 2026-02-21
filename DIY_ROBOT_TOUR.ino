#include <MPU6050_tockn.h>
#include <Wire.h>

const int dirRight = 13; const int pwmRight = 11;
const int dirLeft = 12;  const int pwmLeft = 3;

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  mpu6050.begin();

  // --- THE UNIVERSAL STABILIZER ---
  // Register 0x1A: Set DLPF to 10Hz (0x05). 
  // This is the "Heavy" filter to kill wood-floor chatter.
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05); 
  Wire.endTransmission();

  // Register 0x1B: Set Gyro Range to 500 deg/s (0x08).
  // This prevents "clipping" during mousepad stutters.
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  pinMode(dirRight, OUTPUT); pinMode(pwmRight, OUTPUT);
  pinMode(dirLeft, OUTPUT);  pinMode(pwmLeft, OUTPUT);

  Serial.println(F("Place robot on Wood floor and leave still..."));
  mpu6050.calcGyroOffsets(true);
  delay(2000);
  
  turnDegrees(90.0);
}

void loop() { mpu6050.update(); }

void stopMotors() {
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, 0);
}

void turnDegrees(float degreesToTurn) {
  const float SENSOR_SCALE = 1.33; 
  
  // SCIENTIFIC TWEAK: The "Anticipation" value.
  // If wood floor overshoots by 2 degrees, set this to 2.0.
  const float STOP_EARLY_THRESHOLD = 1.8; 

  const int   START_PUNCH = 145; 
  const int   FINISH_CREEP = 80;    // Dropped to 80 to further reduce wood-floor momentum
  const float TRANSITION_DEG = 25.0; 

  stopMotors();
  mpu6050.update();
  float startAngle = mpu6050.getAngleZ();
  float currentPhysical = 0;

  bool turningRight = (degreesToTurn > 0);
  if (turningRight) {
    digitalWrite(dirLeft, LOW); digitalWrite(dirRight, LOW); 
  } else {
    digitalWrite(dirLeft, HIGH); digitalWrite(dirRight, HIGH); 
  }

  unsigned long startTime = millis();

  while(true) {
    mpu6050.update();
    currentPhysical = (mpu6050.getAngleZ() - startAngle) * SENSOR_SCALE;
    
    float absError = abs(degreesToTurn - currentPhysical);

    // Instead of 0.4, we break early to account for the physical slide
    if (absError <= STOP_EARLY_THRESHOLD) break;

    int speed;
    if (millis() - startTime < 150) {
      speed = START_PUNCH; 
    } else if (absError < TRANSITION_DEG) {
      speed = FINISH_CREEP;
    } else {
      speed = 110; 
    }

    analogWrite(pwmLeft, speed);
    analogWrite(pwmRight, speed);
  }

  // --- REFINED ACTIVE BRAKE ---
  // Flip directions and give a slightly stronger pulse to "bite" the wood
  digitalWrite(dirLeft, !digitalRead(dirLeft)); 
  digitalWrite(dirRight, !digitalRead(dirRight));
  analogWrite(pwmLeft, 160); 
  analogWrite(pwmRight, 160);
  delay(65); // Increased from 50ms to 65ms to kill that wood-floor drift
  stopMotors();
}