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
    integral = limit(integral, -20.0, 20.0);
    double derivative = (position - lastPosition)/(float)deltaT; // take the derivative of the error
    
    lastPosition = position;
    lastTime = currentTime;

    
    return limit(error*KP + integral*KI + derivative*KD, lowerbound, upperbound);
}
void pid::limitOutput(double lowerbound, double upperbound){
  lim = true;
  this->upperbound = upperbound;
  this->lowerbound = lowerbound;
}
double pid::limit(double num, double lowerbound, double upperbound){
  if(lim){
    if(num < lowerbound){
      return lowerbound;
    }
    if(num > upperbound){
      return upperbound;
    }
  }
  return num;
}

