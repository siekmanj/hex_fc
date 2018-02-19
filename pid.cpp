#include "pid.h"

pid::pid(double kp, double ki, double kd){
    KP = kp;
    KI = ki;
    KD = kd;
    lastError = 0;
    integral = 0;
    lastPosition = 0;
}
double pid::calculateOutput(unsigned long currentTime, double position, double setpoint){
    int deltaT = currentTime - lastTime;
    
    double error = setpoint - (position * 16.667); // multiply position (in degrees) by a constant to get approximately microsecond-sized units
    integral += error * deltaT; // integrate the error 
    double derivative = (position - lastPosition)/(float)deltaT; // take the derivative of the error
    
    lastPosition = position;
    lastTime = currentTime;
    return error*KP + integral*KI + derivative*KD;
}

/*
 * void calculate_pid(){

  int dt = micros() - timeOfLastPID;
  timeOfLastPID = micros();

  errorTemp = setPointPitch - (pitchEstimate * 16.667); //This is the proportional term, which is fairly basic.
  
  integralP += errorTemp * dt; //The integral is just the area under the curve, so all we have to do is keep adding the errors together.
  derivativeP = (errorTemp - prevErrorP) / dt; //calculate the rate of change based on this loop's error and last loop's error.
  prevErrorP = errorTemp;
  outputPitch = KPp*errorTemp + KIp*integralP + KDp*derivativeP;
  
  if(outputPitch > 400) outputPitch = 400; //make sure the output stays within a certain range.
  if(outputPitch < -400) outputPitch = -400;

  
  errorTemp = setPointRoll - (rollEstimate * 16.667);
  integralR += errorTemp * dt;
  derivativeR = (errorTemp - prevErrorR) / dt;
  prevErrorR = errorTemp;
  outputRoll = KPr*errorTemp + KIr*integralR + KDr*derivativeR;
  if(outputRoll > 400) outputRoll = 400;
  if(outputRoll < -400) outputRoll = -400;
  
  
  if(throttle > 1020){
    errorTemp = (setPointYaw) - gZ * convert_to_degrees * 16.667;
  }else{
    errorTemp = 0;
    prevErrorY = 0;
    outputYaw = 0;
    outputPitch = 0;
    outputRoll = 0;
    integralP = 0;
    integralR = 0;
    integralY = 0;
    prevErrorP = 0;
    prevErrorY = 0;
    prevErrorR = 0;

    outputPitch = 0;
    outputRoll = 0;
    outputYaw = 0;
  }
  integralY += errorTemp * dt;
  derivativeR = (errorTemp - prevErrorY) / dt;
  prevErrorY = errorTemp;
  outputYaw = KPy*errorTemp + KIy*integralY + KDy*derivativeY;
  if(outputYaw > 400) outputYaw = 400;
  if(outputYaw < -400) outputYaw = -400;
  
}

 */
