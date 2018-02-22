#include "pid.h"
#include "IMU.h"
#include "RC.h"

const float MAX_PID_OUT = 400;

int timeout = 0;

unsigned long zero_timer;

pid *pitchPID;
pid *rollPID;
pid *yawPID;

IMU *mpu;

RC *rc;

void setup() {
  Serial.begin(250000);
  log("\n\n");
  log("init HEX_FCv2.0");
  
  DDRD |= B11111100; //Set ports 2, 3, 4, 5, 6, and 7 to output
  DDRC |= B00000001; //Set port A0 to output

  float KPp = .5235;                    
  float KIp = 0.00000000;               
  float KDp = 1;                     
  
  float KPr = .5235;                       
  float KIr = 0.00000000;                
  float KDr = 1;                   
  
  float KPy = 0.01;                       
  float KIy = .00;                      
  float KDy = 1;

  log("Initializing PID controllers.");

  pitchPID = new pid(KPp, KIp, KDp);
  rollPID = new pid(KPr, KIr, KDr);
  yawPID = new pid(KPy, KIy, KDy);
  
  log("Initializing IMU.");
  
  mpu = new IMU();
  mpu->calibrate(2000);

  log("Waiting for connection to RC controller.");
  
  rc = new RC();
  while(!rc->connected()){//Wait until we get a connection to the RC controller
    Serial.print(".");
    delay(500); 
  }
  Serial.println();

  log("Connected. Awaiting arm command from user.");

  WAIT_FOR_INPUT();
  
  log("Arming ESCs...");

  ARM_ESCS();

  log("WARNING: ESCS ARE ARMED!");

  mpu->centerGyro();
  
  log("Here we go, boys and girls...");
  digitalWrite(A0, HIGH);
  zero_timer = micros();
}

void loop(){

    //Get values from IMU
    mpu->update();
    float pitchEstimate = mpu->getPitch();
    float rollEstimate = mpu->getRoll();
    float yawEstimate = mpu->getYaw();

    //Get values from RC receiver
    float setpointPitch = rc->getPitchSetpoint();
    float setpointRoll = rc->getRollSetpoint();
    float setpointYaw = rc->getYawSetpoint();
    int throttle = rc->getThrottle();

    //Calculate PID outputs
    unsigned long current_time = micros();
    float outputPitch = limit(pitchPID->calculateOutput(current_time, pitchEstimate, setpointPitch), -MAX_PID_OUT, MAX_PID_OUT);
    float outputRoll = limit(rollPID->calculateOutput(current_time, rollEstimate, setpointRoll), -MAX_PID_OUT, MAX_PID_OUT);
    float outputYaw = limit(yawPID->calculateOutput(current_time, yawEstimate, setpointYaw), -MAX_PID_OUT, MAX_PID_OUT); 

    //Wait until 5ms have passed since the last loop to maintain a stable iteration time
    while(zero_timer + 5000 > micros()){}

    //If we lose connection with the remote control this loop, increment a timeout timer.
    if(!rc->connected()) timeout++;
    else timeout = 0;
    
    zero_timer = micros();
    if(timeout < 10){
      PORTD |= B11111100; //Set ports 2-7 to high to start PWM pulse
       
      unsigned long esc1 = zero_timer + throttle + outputPitch - outputRoll + outputYaw; // Front left (2)
      unsigned long esc2 = zero_timer + throttle               - outputRoll - outputYaw; // Left (3)
      unsigned long esc3 = zero_timer + throttle - outputPitch - outputRoll + outputYaw; // Back Left (4)
      unsigned long esc4 = zero_timer + throttle - outputPitch + outputRoll - outputYaw; // Back Right (5)
      unsigned long esc5 = zero_timer + throttle               + outputRoll + outputYaw; // Right (6)
      unsigned long esc6 = zero_timer + throttle + outputPitch + outputRoll - outputYaw; // Front Right (7)
      
      while(PORTD >= 4){
        unsigned long esc_loop_timer = micros();
        if(esc1 <= esc_loop_timer) PORTD &= B11111011; //If the current time is greater than the time we want esc1 to be high for, turn port 2 off.
        if(esc2 <= esc_loop_timer) PORTD &= B11110111; //If the current time is greater than the time we want esc2 to be high for, turn port 3 off.
        if(esc3 <= esc_loop_timer) PORTD &= B11101111; //If the current time is greater than the time we want esc3 to be high for, turn port 4 off.
        if(esc4 <= esc_loop_timer) PORTD &= B11011111; //If the current time is greater than the time we want esc4 to be high for, turn port 5 off.
        if(esc5 <= esc_loop_timer) PORTD &= B10111111; //If the current time is greater than the time we want esc5 to be high for, turn port 6 off.
        if(esc6 <= esc_loop_timer) PORTD &= B01111111; //If the current time is greater than the time we want esc6 to be high for, turn port 7 off.
        if(micros() - zero_timer > 2000) PORTD &= B00000011; //If for some ungodly reason it's been more than 2ms and any ports are still on, turn everything off.
      }
    }else{ //RC receiver has not been getting signal for too long, so we need to abort immediately.
      EMERGENCY_ABORT();
    }    
}

void WAIT_FOR_INPUT(){
  while(rc->getCh3() < 800 || rc->getCh3() > 1040 || rc->getCh4() > 1040){
     delay(50);
     log("Waiting for arm command...");
     digitalWrite(A0, !digitalRead(A0));
     delay(500);
  }
}

float limit(float num, float lowerbound, float upperbound){
  if(num < lowerbound){
    return lowerbound;
  }
  if(num > upperbound){
    return upperbound;
  }
  return num;
}
void EMERGENCY_ABORT(){
  log("WARNING: SIGNAL LOST");
  while(true){
    digitalWrite(A0, !digitalRead(A0)); //Flash LED at analog pin 0
    delay(100);
    PORTD |= B11111100; //Write a steady 1000us pulse to motors to get them to stop spinning.
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
}
void ARM_ESCS(){
/*  for(int i = 1000; i < 2000; i+=10){
    PORTD |= B11111100;
    delayMicroseconds(i);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
  for(int i = 2000; i > 1000; i-=10){
    PORTD |= B11111100;
    delayMicroseconds(i);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
*/ 
  for(int i = 0; i < 25; i++){
    PORTD |= B11111100;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
}
void log(char* msg){
  Serial.print(millis()/1000.0, 4);
  Serial.print(": ");
  Serial.println(msg);
}
ISR(PCINT0_vect){
  rc->interruptHandler();
}



