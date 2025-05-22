#include "SparkFunLSM6DSO.h"
#include "Wire.h"
//#include "SPI.h"

LSM6DSO myIMU; //Default constructor is I2C, addr 0x6B
float angle;
unsigned long prevTime=millis();
void setup() {


  Serial.begin(115200);
  delay(500); 

  Wire.begin();
  delay(10);
  if( myIMU.begin() )
    Serial.println("Ready.");
  else { 
    Serial.println("Could not connect to IMU.");
    Serial.println("Freezing");
  }

  if( myIMU.initialize(BASIC_SETTINGS) )
    Serial.println("Loaded Settings.");
  delay(100);
  angle=atan2(myIMU.readFloatAccelX(), myIMU.readFloatAccelY()) * 180 / PI;
}



void loop()
{
  unsigned long dt=millis()-prevTime;
  prevTime=millis();
  

  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  
  float gx = myIMU.readFloatGyroX();
  float gy = myIMU.readFloatGyroY();
  float gz = myIMU.readFloatGyroZ();

  float a1=atan2(ax, sqrt(sq(ay)+sq(az))) * 180 / PI;
  float a2=-(gy*.67-.775*gz)*dt/1000;
  angle=a1*.1+(angle+a2)*.9 ;
  // Print in CSV format: ax, ay, az, gx, gy, gz


  Serial.print(a1);Serial.print(",");
  Serial.print(a2);Serial.print(",");
  Serial.print(angle+a2);Serial.print(",");
  Serial.print(angle);Serial.print(",");


  Serial.println();


}
