#ifndef _MUXCONTROL_H_
#define _MUXCONTROL_H_

#include <Arduino.h>

class MUXShield{ 
public:
  int control0, control1, control2, control3;
  int MUXOutputChannelNum;
  // Constructor
  MUXShield(int control0, int control1, int control2, int control3){
    this->control0 = control0;
    this->control1 = control1;
    this->control2 = control2;
    this->control3 = control3;
    
    //Set MUX control pins to output
    pinMode(control0, OUTPUT);
    pinMode(control1, OUTPUT);
    pinMode(control2, OUTPUT);
    pinMode(control3, OUTPUT);
  }

  void select(int MUXOutputChannelNum ){
    digitalWrite(control0, (MUXOutputChannelNum & 15)>>3); 
    digitalWrite(control1, (MUXOutputChannelNum & 7)>>2);  
    digitalWrite(control2, (MUXOutputChannelNum & 3)>>1);  
    digitalWrite(control3, (MUXOutputChannelNum & 1));    
  }

};

#endif//_MUXCONTROL_H_
