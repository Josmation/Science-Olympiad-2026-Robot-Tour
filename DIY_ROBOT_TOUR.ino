
#include <MPU6050_tockn.h>
#include <Wire.h>

//motor specs
const float GEAR_RATIO = 75.0;      
const float WHEEL_DIAM_MM = 65.0;   
const float ENCODER_PPR = 11.0;


//motors pins and encoder pins
const int dirRight = 13; const int pwmRight = 11;
const int dirLeft = 12;  const int pwmLeft = 3;
const int encLeftA = 2;
const int encLeftB = 4; 


const int buttonPin = 13;

float ticksPerMeter;

MPU6050 mpu6050(Wire);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);  
  Wire.begin();
  mpu6050.begin();


  //mpu6050.setGyroOffsets(1.45, 1.23, -1.32);


  mpu6050.calcGyroOffsets(true);
  


  ticksPerMeter = ((ENCODER_PPR * GEAR_RATIO) / ((WHEEL_DIAM_MM * 3.14159) / 1000.0))*.71582;
  //calculates the ticks of a motor per meter (its math is wrong and is multiplied by the meter/distance it thinks one meter is to get the real amount)

}

void loop() {
  // put your main code here, to run repeatedly:


  
 


}
