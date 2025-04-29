#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ODriveUART.h>


LSM6DSO myIMU; //default construction
HardwareSerial& odrive_serial = Serial1;
const unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)
ODriveUART odrive(odrive_serial);
float angle; //angle of the bike
float prevAngle; //previous angle of the bike
unsigned long prevTime=millis(); //previous time

//weight gyroscope and accelerometer 
const float kGA=.9;

//area of error, from the IMU for PID
float area;
const float kP=.01; //proportional gain
const float kI=.0000001; //integral gain
const float kD=.01; //derivative gain
void setup(){
    //configure odrive
    odrive_serial.begin(baudrate);
    Serial.begin(115200); 
    Serial.println("Waiting for ODrive...");
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(10);
    }

    Serial.println("found ODrive");
    Serial.print("DC voltage: ");
    Serial.println(odrive.getParameterAsFloat("vbus_voltage"));
    
    Serial.println("Enabling closed loop control...");
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }
    Serial.println("ODrive running!");

    //configure imu
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
    
    //initalize angle depends on congiuration of the actual IMU could be subject to change depending on how we attach the pdrive
    //for now this is a simple way to determine the angle of the bike
    //read the sensor to get the angle
    angle = atan2(myIMU.readFloatAccelX(), myIMU.readFloatAccelY()) * 180 / PI; //convert to degrees
    prevAngle=float(angle);
    prevTime=millis(); //set the previous time to the current time
}

//for now we are using floating points but note thhat later we will change this to integers for faster speeds
void loop(){
    unsigned long dt=millis()-prevTime;
    prevTime=millis();
    float a1=atan2(myIMU.readFloatAccelX(), myIMU.readFloatAccelY()) * 180 / PI;
    float a2=angle-myIMU.readFloatGyroZ()*dt/1000;
    angle=a1*(1-kGA)+a2*kGA; //calculate the angle of the bike

    //PID implementation
    area+=angle*dt;
    float torque=kI*area+kP*angle+kD*(angle-prevAngle)/dt; //calculate the velocity of the bike
    prevAngle=angle; //set the previous angle to the current angle
    odrive.setTorque(torque); //set torque
    Serial.print(angle); Serial.print(',');
    Serial.println(torque);

}