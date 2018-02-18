#include <Wire.h>
#include "pid.h"
#include "IMU.h"

float KPp, KIp, KDp, KPr, KIr, KDr, KPy, KIy, KDy, KPt, KIt, KDt;
unsigned long esc1, esc2, esc3, esc4, esc5, esc6;
float setPointPitch, setPointRoll, setPointYaw, throttle;

unsigned long esc_loop_timer;
unsigned long zero_timer;

pid *pitchPID;
pid *rollPID;
pid *yawPID;

IMU *mpu;

void setup() {
  Serial.begin(250000);
  Wire.begin();
  Serial.println("INIT HEX_FCv2.0");
  
  DDRD |= B11111100;               // Set ports 2, 3, 4, 5, 6, and 7 to output
  DDRC |= B00100000;
  PCICR |= (1 << PCIE0);           // set PCIE0 to enable PCMSK0 scan
  //PCICR |= (1 << PCIE1);           // set PCIE1 to enable PCMSK1 scan
  PCMSK0 |= (1 << PCINT0);         // set PCINT0 (digital input 8) to trigger an interrupt
  PCMSK0 |= (1 << PCINT1);         // set PCINT1 (digital input 9) to trigger an interrupt
  PCMSK0 |= (1 << PCINT2);         // set PCINT2 (digital input 10) to trigger an interrupt
  PCMSK0 |= (1 << PCINT3);         // set PCINT3 (digital input 11) to trigger an interrupt
  PCMSK0 |= (1 << PCINT4);         // set PCINT4 (digital input 12) to trigger an interrupt
  PCMSK0 |= (1 << PCINT5);
  //PCMSK1 |= (1 << PCINT11);                          //set PCINT11 (analog input 3) to trigger an interrupt

  KPp = .5235;                    
  KIp = 0.00000000;               
  KDp = 1;                     
  
  KPr = .5235;                       
  KIr = 0.00000000;                
  KDr = 1;                   
  
  KPy = 0.01;                       
  KIy = .00;                      
  KDy = 1;

  pitchPID = new pid(KPp, KIp, KDp);
  rollPID = new pid(KPr, KIr, KDr);
  yawPID = new pid(KPy, KIy, KDy);
  
  Serial.println("INITIALIZING IMU");
  
  mpu = new IMU();
  mpu->calibrate(200);

  Serial.println("AWAITING INPUT");

  //WAIT_FOR_INPUT();
  
  Serial.println("ARMING ESCs!");

  ARM_ESCS();

  mpu->centerGyro();
  
  Serial.println("Here we go, boys and girls...");
  digitalWrite(A0, HIGH);
  zero_timer = micros();
}

void loop(){

    mpu->update();
    
    float pitchEstimate = mpu->getPitch();
    float rollEstimate = mpu->getRoll();
    float yawEstimate = mpu->getYaw();
    
    if(receiver_ch4 > 1508) setPointYaw = (1508 - receiver_ch4)/16;
    else if(receiver_ch4 < 1492) setPointYaw = (1492 - receiver_ch4)/16;
    
    throttle = receiver_ch3;
    
    if(receiver_ch2 > 1508) setPointPitch = (1508 - receiver_ch2)/1.6;
    else if(receiver_ch2 < 1492) setPointPitch = (1492 - receiver_ch2)/1.6;
    
    if(receiver_ch1 > 1508) setPointRoll = (receiver_ch1 - 1508)/-1.6;
    else if(receiver_ch1 < 1492) setPointRoll = (receiver_ch1 - 1492)/-1.6;

    //calculate_pid();
    unsigned long current_time = micros();
    float outputPitch = pitchPID->calculateOutput(current_time, pitchEstimate, setPointPitch);
    float outputRoll = rollPID->calculateOutput(current_time, rollEstimate, setPointRoll);
    float outputYaw = yawPID->calculateOutput(current_time, yawEstimate, setPointYaw); 
    //Serial.print(pitchEstimate); Serial.print(", "); Serial.print(rollEstimate); Serial.print(", "); Serial.println(outputYaw); 
    //Serial.print(" | ");
    while(zero_timer + 5000 > micros()){}
    zero_timer = micros();
    
    esc1 = zero_timer + throttle + outputPitch - outputRoll + outputYaw; // Front left
    esc2 = zero_timer + throttle               - outputRoll - outputYaw; // Left
    esc3 = zero_timer + throttle - outputPitch - outputRoll + outputYaw; // Back Left
    esc4 = zero_timer + throttle - outputPitch + outputRoll - outputYaw; // Back Right
    esc5 = zero_timer + throttle               + outputRoll + outputYaw; // Right
    esc6 = zero_timer + throttle + outputPitch + outputRoll - outputYaw; // Front Right
    
    PORTD |= B11111100;

    while(PORTD >= 4){
      esc_loop_timer = micros();
      if(esc1 <= esc_loop_timer) PORTD &= B11111011; 
      if(esc2 <= esc_loop_timer) PORTD &= B11110111; 
      if(esc3 <= esc_loop_timer) PORTD &= B11101111; 
      if(esc4 <= esc_loop_timer) PORTD &= B11011111; 
      if(esc5 <= esc_loop_timer) PORTD &= B10111111; 
      if(esc6 <= esc_loop_timer) PORTD &= B01111111; 
      
      if(2000 < micros() - esc_loop_timer) PORTD &= B00000011;
    }

//    Serial.println(micros()-start_t);

}

void WAIT_FOR_INPUT(){
  while(receiver_ch3 < 800 || receiver_ch3 > 1040 || receiver_ch4 > 1040){
     delay(50);
     Serial.println("Waiting for receiver...");
     digitalWrite(A0, !digitalRead(A0));
  }
}

void ARM_ESCS(){
  //Calibrate ESCs
  /*for(int i = 2000; i > 1000; i-=4){
    unsigned long loop_start_t = micros();
    PORTD |= B11111100;
    delayMicroseconds(i);
    PORTD &= B00000000;
    while(loop_start_t + 4000 > micros());
  }*/
  for(int i = 0; i < 25; i++){
    PORTD |= B11111100;
    delayMicroseconds(1000);
    PORTD &= B00000000;
    delayMicroseconds(3000);
  }
  Serial.println("ARMED");
}

