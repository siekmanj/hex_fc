#include "RC.h"
#include <Arduino.h>

const float SENSITIVITY = 0.625;
const float YAW_SENSITIVITY = 0.0625;

RC::RC(){
  PCICR |= (1 << PCIE0);           // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);         // set PCINT0 (digital input 8) to trigger an interrupt
  PCMSK0 |= (1 << PCINT1);         // set PCINT1 (digital input 9) to trigger an interrupt
  PCMSK0 |= (1 << PCINT2);         // set PCINT2 (digital input 10) to trigger an interrupt
  PCMSK0 |= (1 << PCINT3);         // set PCINT3 (digital input 11) to trigger an interrupt
  PCMSK0 |= (1 << PCINT4);         // set PCINT4 (digital input 12) to trigger an interrupt
  PCMSK0 |= (1 << PCINT5);
  
  //PCICR |= (1 << PCIE1);           // set PCIE1 to enable PCMSK1 scan
  //PCMSK1 |= (1 << PCINT11);        // set PCINT11 (analog input 3) to trigger an interrupt
  
  last_channel_1 = 0;
  last_channel_2 = 0;
  last_channel_3 = 0;
  last_channel_4 = 0;
  last_channel_5 = 0;
  last_channel_6 = 0;
  timeOfLastRead = 0;
}

bool RC::connected(){
  if(timeOfLastRead == 0) return false; //If we have not yet received radio contact, we are not connected.
  if(micros()-timeOfLastRead > 1000000) return false; //If we go one full second without radio contact, we are in danger of losing connection.
  return true;
}
double RC::getPitchSetpoint(){
  if(receiver_ch2 > 1508) return (1508 - receiver_ch2) * SENSITIVITY;
  else if(receiver_ch2 < 1492) return (1492 - receiver_ch2) * SENSITIVITY;
}
double RC::getRollSetpoint(){
  if(receiver_ch1 > 1508) return (receiver_ch1 - 1508) * SENSITIVITY;
  else if(receiver_ch1 < 1492) return (receiver_ch1 - 1492) * SENSITIVITY;
}
double RC::getYawSetpoint(){
  if(receiver_ch4 > 1508) return (1508 - receiver_ch4) * YAW_SENSITIVITY;
  else if(receiver_ch4 < 1492) return (1492 - receiver_ch4) * YAW_SENSITIVITY;
}
int RC::getThrottle(){
  return receiver_ch3;
}
int RC::getCh1(){
  return receiver_ch1;
}
int RC::getCh2(){
  return receiver_ch2;
}
int RC::getCh3(){
  return receiver_ch3;
}
int RC::getCh4(){
  return receiver_ch4;
}
int RC::getCh5(){
  return receiver_ch5;
}
int RC::getCh6(){
  return receiver_ch6;
}
void RC::interruptHandler(){ //It's really difficult to put an ISR inside a class, so this needs to be called inside an ISR in the main sketch.
  timeOfLastRead = micros();
  //Channel 1=========================================
  if(PINB & B00000001){ //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = timeOfLastRead;                                  //Set timer_1 to timeOfLastRead
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_ch1 = timeOfLastRead - timer_1;         //Channel 1 is timeOfLastRead - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = timeOfLastRead;                                  //Set timer_2 to timeOfLastRead
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_ch2 = timeOfLastRead - timer_2;         //Channel 2 is timeOfLastRead - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = timeOfLastRead;                                  //Set timer_3 to timeOfLastRead
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_ch3 = timeOfLastRead - timer_3;         //Channel 3 is timeOfLastRead - timer_3
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = timeOfLastRead;                                  //Set timer_4 to timeOfLastRead
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_ch4 = timeOfLastRead - timer_4;         //Channel 4 is timeOfLastRead - timer_4
  }
  //Channel 5=========================================
  if(PINB & B00010000 ){                                       //Is input 12 high?
    if(last_channel_5 == 0){                                   //Input 12 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = timeOfLastRead;                                  //Set timer_5 to timeOfLastRead
    }
  }
  else if(last_channel_5 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_ch5 = timeOfLastRead - timer_5;         //Channel 4 is timeOfLastRead - timer_4
  }
  //Channel 6=========================================
  if(PINB & B00100000 ){                                       //Is input 13 high?
    if(last_channel_6 == 0){                                   //Input 13 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = timeOfLastRead;                                  //Set timer_6 to timeOfLastRead
    }
  }
  else if(last_channel_6 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_ch6 = timeOfLastRead - timer_6;        
  }
}
