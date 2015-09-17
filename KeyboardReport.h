#ifndef _KEYBOARDREPORT_H_
#define _KEYBOARDREPORT_H_

#include <Arduino.h>
// Key Definition 
uint8_t keyNone[8] = {
  0, 0, 0, 0, 0, 0, 0, 0};
uint8_t keyA[8] = {
  0, 0, 4, 0, 0, 0, 0, 0};
uint8_t key_plus[8] = {
  0, 0, 87, 0, 0, 0, 0, 0};
uint8_t key_minus[8] = {
  0, 0, 86, 0, 0, 0, 0, 0};
uint8_t key_RightArrow[8] = {
  0, 0, 79, 0, 0, 0, 0, 0};
uint8_t key_LeftArrow[8] = {
  0, 0, 80, 0, 0, 0, 0, 0};
uint8_t key_LeftDownArrow[8] = {
  0, 0, 80, 90, 0, 0, 0, 0 };
uint8_t key_RightUpArrow[8] = {
  0, 0, 79, 96, 0, 0, 0, 0}; 
uint8_t key_RightDownArrow[8] = {
  0, 0, 79, 90, 0, 0, 0, 0};
uint8_t key_LeftUpArrow[8] = {
  0, 0, 80, 96, 0, 0, 0, 0}; 

class KeyboardSim{
public:
  unsigned char PreviousFlag;
  // constructor
  KeyboardSim(){
    PreviousFlag = 0;
  }

  int report(unsigned char flag){
    // select key
    switch(flag){
    case 0:
      //Serial.write(key_plus, 8);
      break;
    case 1:
      Serial.write(key_plus, 8);
      break;
    case 2:
      Serial.write(key_minus, 8);
      break;
    case 3:
      Serial.write(key_RightDownArrow, 8);
      break;
    case 4:
      Serial.write(key_LeftUpArrow, 8);
      break;
    case 5:
      Serial.write(key_LeftDownArrow, 8);
      break;
    case 6:
      Serial.write(key_RightUpArrow, 8);
      break;
    case 7:
      Serial.write(key_RightArrow, 8);
      break;
    case 8:
      Serial.write(key_LeftArrow, 8);
      break;
    }// switch(flag)

    if(PreviousFlag != flag){// while flag change, sned end strok
      if(PreviousFlag != 0)// if preevius is netural, dont sent stop strock 
        Serial.write(keyNone, 8);
    }
    else{ // while not change,
      return 0;
    } 

    // update flag
    PreviousFlag = flag;

    return 0;
  }

};

#endif//_BENDINGSTATE_H_



