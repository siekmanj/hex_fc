#include "pid.h"
#include "IMU.h"
#include "RC.h"

const float MAX_PID_OUT = 400;

unsigned long zero_timer;
int timeout = 0;

pid *pitchPID;
pid *rollPID;
pid *yawPID;

IMU *mpu;

RC *rc;

void setup() {
  Serial.begin(9600);
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
  mpu->calibrate(3000);

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
    unsigned long start_t = micros();
    //Get values from IMU
    unsigned long current_t = micros();
    mpu->update();
    float pitchEstimate = mpu->getPitch();
    float rollEstimate = mpu->getRoll();
    float yawEstimate = mpu->getYaw();

    //Get values from RC receiver
    float setpointPitch = rc->getPitchSetpoint();
    float setpointRoll = rc->getRollSetpoint();
    float setpointYaw = rc->getYawSetpoint();
    int throttle = rc->getThrottle();
    //If we lose connection with the remote control this loop, increment this timeout timer.
    if(!rc->connected()) timeout++;
    else timeout = 0;
    Serial.println(micros()-current_t);
    if(throttle > 1040 && timeout < 10){
      zero_timer = micros(); //Reset the zero timer
      PORTD |= B11111100; //Set ports 2-7 to high to start PWM pulse
      //Calculate PID outputs and constrain them between -400 and 400 in case we get ridiculous values
      float outputPitch = pitchPID->calculateOutput(zero_timer, pitchEstimate, setpointPitch);
      float outputRoll = rollPID->calculateOutput(zero_timer, rollEstimate, setpointRoll);
      float outputYaw = yawPID->calculateOutput(zero_timer, yawEstimate, setpointYaw); 
       
      unsigned long esc1 = zero_timer + throttle;// + outputPitch - outputRoll + outputYaw; // Front left (2)
      unsigned long esc2 = zero_timer + throttle;//               - outputRoll - outputYaw; // Left (3)
      unsigned long esc3 = zero_timer + throttle;// - outputPitch - outputRoll + outputYaw; // Back Left (4)
      unsigned long esc4 = zero_timer + throttle;// - outputPitch + outputRoll - outputYaw; // Back Right (5)
      unsigned long esc5 = zero_timer + throttle;//               + outputRoll + outputYaw; // Right (6)
      unsigned long esc6 = zero_timer + throttle;// + outputPitch + outputRoll - outputYaw; // Front Right (7)
      
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
      //Wait until 5ms have passed since the last loop to maintain a stable iteration time
    }else{ 
      if(timeout > 10) EMERGENCY_ABORT(); //RC receiver has not been getting signal for too long, so we need to abort immediately.
      if(throttle < 1040) MOTOR_SHUTDOWN(); //Throttle is below threshold, so we need to make sure motors don't start spinning unexpectedly.
    }    
    while(start_t + 4000 > micros()){}
}

void WAIT_FOR_INPUT(){
  while(rc->getCh3() < 800 || rc->getCh3() > 1040 || rc->getCh4() > 1040){
     delay(50);
     log("Waiting for arm command...");
     digitalWrite(A0, !digitalRead(A0));
     delay(500);
  }
}
void EMERGENCY_ABORT(){
  log("WARNING: SIGNAL LOST");
  while(true){
    digitalWrite(A0, !digitalRead(A0)); //Flash LED at analog pin 0
    delay(100);
    PORTD |= B11111100; //Write a steady 900us pulse to motors to get them to stop spinning.
    delayMicroseconds(900);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
}
void MOTOR_SHUTDOWN(){
    PORTD |= B11111100; //Write a steady 900us pulse to motors to stop them from spinning.
    delayMicroseconds(900);
    PORTD &= B00000000;
}
void ARM_ESCS(){
  for(int i = 0; i < 25; i++){
    PORTD |= B11111100;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
}
//This is a function that logs output with a timestamp to whatever is connected to the serial pins.
void log(char* msg){
  Serial.print(millis()/1000.0, 4);
  Serial.print(": ");
  Serial.println(msg);
}
//This is an interrupt subroutine for pins 8-13. We want to call our interrupt handler here to update RC values.
ISR(PCINT0_vect){
  rc->interruptHandler();
}



