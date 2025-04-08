#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ODriveUART.h>


LSM6DSO myIMU; //default construction
HardwareSerial& odrive_serial = Serial1;
unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)
ODriveUART odrive(odrive_serial);
float angle; //angle of the bike

//area of error, from the IMU for PID
float area;

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
    angle = atan2(myIMU.readFloatAccelY(), myIMU.readFloatAccelX()) * 180 / PI; //convert to degrees
    unsigned long prevTime=millis(); //set the previous time to the current time
}

//for now we are using floating points but note thhat later we will change this to integers for faster speeds
void loop(){
    unsigned long dt= millis()-prevTime; //calculate the time difference
    prevTime=millis(); //set the previous time to the current time
    float velocity;
    //get angle from the IMU
    a1=atan2(myIMU.readFloatAccelY(), myIMU.readFloatAccelX()) * 180 / PI;
    a2=myIMU.readFloatGyroZ()*dt
     

    odrive.setVelocity(velocity); //set velocity

}