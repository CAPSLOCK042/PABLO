#include "SparkFunLSM6DSO.h"
#include "Wire.h"
#include <ODriveUART.h>


LSM6DSO myIMU; //default construction
HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial2 = Serial2;
const unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)
ODriveUART odrive1(odrive_serial1);
ODriveUART odrive2(odrive_serial2);
float angle; //angle of the bike
float prevAngle; //previous angle of the bike
unsigned long prevTime=millis(); //previous time
float ang=0;
//weight gyroscope and accelerometer 
const float kGA=.9;

//area of error, from the IMU for PID
float area;
float kP=.01; //proportional gain
float kI=0; //integral gain
float kD=.01; //derivative gain
String inputString = "";     // stores the input
bool stringComplete = false; // whether the string is complete
void setup(){
    //configure odrive
    odrive_serial1.begin(baudrate);
    odrive_serial2.begin(baudrate);

    Serial.begin(115200); 
    inputString.reserve(50);
    Serial.println("Waiting for ODrive...");
    while (odrive1.getState() == AXIS_STATE_UNDEFINED && odrive2.getState() == AXIS_STATE_UNDEFINED) {
        delay(10);
    }

    Serial.println("found ODrive");
    Serial.print("DC voltage: ");
    Serial.println(odrive1.getParameterAsFloat("vbus_voltage"));
    Serial.println("found ODrive");
    Serial.print("DC voltage: ");
    Serial.println(odrive2.getParameterAsFloat("vbus_voltage"));
    
    Serial.println("Enabling closed loop control...");
    while (odrive1.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL && odrive2.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive1.clearErrors();
        odrive1.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        odrive2.clearErrors();
        odrive2.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
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
    delay(1000);
    odrive1.setVelocity(-5); //set torque
    odrive2.setVelocity(5);
}
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else {
            inputString += inChar;
        }
    }
}
//for now we are using floating points but note thhat later we will change this to integers for faster speeds
void loop(){
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
    angle=a1*.1+(angle+a2)*.9+ang;


    //PID implementation
    area+=angle*dt;
    float torque=kI*area+kP*angle+kD*(angle-prevAngle)/dt; //calculate the velocity of the bike

    prevAngle=angle; //set the previous angle to the current angle
    float vel1 = odrive1.getVelocity();
    float vel2 = odrive2.getVelocity();


    odrive1.setVelocity(torque*dt-vel1,4.7); //set torque
    odrive2.setVelocity((torque*dt-vel2)/1.1,4.7); //set torque
    Serial.print(angle); Serial.print(",");
    Serial.print(torque*dt-vel1);Serial.print(",");Serial.print((torque*dt-vel2)/2);Serial.print(",Kp=");
    Serial.print(kP);Serial.print(",Ki=");
    Serial.print(kI);Serial.print(",Kd=");
    Serial.println(kD);
    // Handle Serial input for changing Kp, Ki, Kd
    
    if (stringComplete) {
      inputString.toLowerCase();
        if (inputString.startsWith("kp=")) {
            kP = inputString.substring(3).toFloat();
            Serial.print("Updated Kp: "); Serial.println(kP);
        } else if (inputString.startsWith("ki=")) {
            kI = inputString.substring(3).toFloat();
            Serial.print("Updated Ki: "); Serial.println(kI);
        } else if (inputString.startsWith("kd=")) {
            kD = inputString.substring(3).toFloat();
            Serial.print("Updated Kd: "); Serial.println(kD);
        } else if (inputString.startsWith("and=")) {
            ang = inputString.substring(3).toFloat();
            Serial.print("Updated Kd: "); Serial.println(kD);
        } else {
            Serial.println("Unknown command. Use Kp=, Ki=, or Kd=");
        }
        inputString = "";
        stringComplete = false;
    }

}