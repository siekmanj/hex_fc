#include "pid.h"
#include "IMU.h"
#include "RC.h"

#define MAX_PID_OUT 400

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
  
  DDRD |= B11111100;               // Set ports 2, 3, 4, 5, 6, and 7 to output
  DDRC |= B00000001;

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
    unsigned long start_t = micros();
    mpu->update();
    
    float pitchEstimate = mpu->getPitch();
    float rollEstimate = mpu->getRoll();
    float yawEstimate = mpu->getYaw();
    
    float setpointPitch = rc->getPitchSetpoint();
    float setpointRoll = rc->getRollSetpoint();
    float setpointYaw = rc->getYawSetpoint();
    int throttle = rc->getThrottle();
    
    unsigned long current_time = micros();
    
    float outputPitch = constrain(pitchPID->calculateOutput(current_time, pitchEstimate, setpointPitch), -MAX_PID_OUT, MAX_PID_OUT);
    float outputRoll = constrain(rollPID->calculateOutput(current_time, rollEstimate, setpointRoll), -MAX_PID_OUT, MAX_PID_OUT);
    float outputYaw = constrain(yawPID->calculateOutput(current_time, yawEstimate, setpointYaw), -MAX_PID_OUT, MAX_PID_OUT); 

    while(zero_timer + 5000 > micros()){}
    
    if(!rc->connected()) timeout++;
    else timeout = 0;
    
    if(timeout < 10){
      zero_timer = micros();
      PORTD |= B11111100;
       
      unsigned long esc1 = zero_timer + throttle + outputPitch - outputRoll + outputYaw; // Front left
      unsigned long esc2 = zero_timer + throttle               - outputRoll - outputYaw; // Left
      unsigned long esc3 = zero_timer + throttle - outputPitch - outputRoll + outputYaw; // Back Left
      unsigned long esc4 = zero_timer + throttle - outputPitch + outputRoll - outputYaw; // Back Right
      unsigned long esc5 = zero_timer + throttle               + outputRoll + outputYaw; // Right
      unsigned long esc6 = zero_timer + throttle + outputPitch + outputRoll - outputYaw; // Front Right
      
      while(PORTD >= 4){
        unsigned long esc_loop_timer = micros();
        if(esc1 <= esc_loop_timer) PORTD &= B11111011; 
        if(esc2 <= esc_loop_timer) PORTD &= B11110111; 
        if(esc3 <= esc_loop_timer) PORTD &= B11101111; 
        if(esc4 <= esc_loop_timer) PORTD &= B11011111; 
        if(esc5 <= esc_loop_timer) PORTD &= B10111111; 
        if(esc6 <= esc_loop_timer) PORTD &= B01111111; 
        if(micros() - esc_loop_timer > 2000) PORTD &= B00000011;
      }
    }else{ //RC receiver not getting signal
      digitalWrite(A0, !digitalRead(A0)); //Flash LED at analog pin 0
      delay(100);
      PORTD |= B11111100; //Write a steady 1000us pulse to motors to get them to stop spinning.
      delayMicroseconds(1000);
      PORTD &= B00000000;
      delayMicroseconds(3000);
    }    
}

void WAIT_FOR_INPUT(){
  while(rc->getCh3() < 800 || rc->getCh3() > 1040 || rc->getCh4() > 1040){
     delay(50);
     log("Waiting for receiver...");
     digitalWrite(A0, !digitalRead(A0));
     delay(500);
  }
}

void ARM_ESCS(){
  for(int i = 0; i < 25; i++){
    PORTD |= B11111100;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
  log("ARMED");
}
void log(char* msg){
  Serial.print(millis()/1000.0, 4);
  Serial.print(": ");
  Serial.println(msg);
}
ISR(PCINT0_vect){
  rc->interruptHandler();
}



