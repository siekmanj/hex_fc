#include <Arduino.h>
#include "IMU.h"
#include "Wire.h"

//Configure the MPU_6050
IMU::IMU(){
  Serial.println("Starting transmission with MPU");
  delay(200);
  Wire.beginTransmission(0x68);       //Start communicating with the MPU-6050
  Wire.write(0x6B);                   //Select register 0x6B
  Wire.write(0x00);                   //Send reset command
  Wire.endTransmission();             //End the transmission 
  delay(200);
  
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);       //Start communicating with the MPU-6050
  Wire.write(0x1C);                   //Send accelerometer self-test command
  Wire.write(0x10);             
  Wire.endTransmission();             //End the transmission
  delay(200);
  
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);       //Start communicating with the MPU-6050
  Wire.write(0x1B);                   //Send gyroscope self-test command
  Wire.write(0x08);             
  Wire.endTransmission();             //End transmission
  delay(200);

  last_t = micros();

  gyroYdrift = 0;
  gyroXdrift = 0;
  gyroZdrift = 0;
  pitchEstimate = 0;
  rollEstimate = 0;
  convert_to_degrees = 0;
  
}

void IMU::calibrate(int accuracy){
  double xdrift = 0;
  double ydrift = 0;
  double zdrift = 0;
  for(int i = 0; i < accuracy; i++){
    IMU::update();
    ydrift += gyroY;
    xdrift += gyroX;
    zdrift += gyroZ;
  }
  gyroYdrift = ydrift/accuracy;
  gyroXdrift = xdrift/accuracy;
  gyroZdrift = zdrift/accuracy;
}

double IMU::getPitch(){
  return pitchEstimate;
}
double IMU::getRoll(){
  return rollEstimate;
}
double IMU::getYaw(){
  return gyroZ;
}

void IMU::centerGyro(){
  pitchEstimate = accBiasPitch;
  rollEstimate = accBiasRoll;
}

void IMU::update(){

  Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
  Wire.write(0x3B);             //Send the address of ACCEL_XOUT (first register we want to read)
  Wire.endTransmission();       //End transmission
  Wire.requestFrom(0x68,14);    //Request 14 bytes
  
  while(Wire.available() < 14); //Wait until all the bytes are received
  
  accelX = Wire.read() << 8 | Wire.read();                                  
  accelY = Wire.read( )<< 8 | Wire.read();                                  
  accelZ = Wire.read() << 8 | Wire.read();                                  
  Wire.read(); Wire.read(); //Discard temp                              
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();                   
  gyroZ = Wire.read() << 8 | Wire.read();
  
  gyroY -= gyroYdrift;
  gyroX -= gyroXdrift;
  gyroZ -= gyroZdrift;
  
  pitchEstimate += gyroY * convert_to_degrees;
  rollEstimate += gyroX * convert_to_degrees;
  rollEstimate += pitchEstimate * sin(gyroZ * (convert_to_degrees * (3.142/180)));
  pitchEstimate -= rollEstimate * sin(gyroZ * (convert_to_degrees * (3.142/180))); 
  double aTotal = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));            
  accBiasRoll = asin((double)accelY/aTotal)* 57.2958; 
  accBiasPitch = asin((double)accelX/aTotal)* -57.2958; 


  pitchEstimate = pitchEstimate * 0.9992 + accBiasPitch * 0.0008;
  rollEstimate = rollEstimate * 0.9992 + accBiasRoll * 0.0008;
  double deltaT = micros() - last_t;
  convert_to_degrees = 1/(((1000000/deltaT)*65.5));
  last_t = micros();

  //pitchEstimate = pitchEstimate * 0.7 + gyroPitchEstimate * 0.3;
  //rollEstimate = rollEstimate * 0.7 + gyroRollEstimate * 0.3; 
                                 
}

