#ifndef _BENDINGSTATE_H_
#define _BENDINGSTATE_H_

#include <Arduino.h>

// State Definition 
#define NETURAL 0 
#define VERTICAL_CONCAVE 1 
#define VERTICAL_CONVAX 2
#define CONCAVE_45DEGREE 3 
#define CONVAX_45DEGREE 4
#define CONCAVE_135DEGREE 5 
#define CONVAX_135DEGREE 6
#define TORSION_POSTIVE 7 
#define TORSION_NEGTIVE 8
#define HORIZON_CONCAVE 10 
#define HORIZON_CONVAX 20 

struct Node{
  unsigned char BendingState;//flag of Bending State0~255
  int value;
  Strain *SensorLink;
};

class BendingState{
public:  
  Node *node; // sensing node
  int NodeNum; // node number
  unsigned char GestureFlag; // gesture flag
public:

  // constructor
  BendingState(StrainSensors *sensors){
    node = new Node[sensors->SensorNum];
    GestureFlag = 0;
    // Assigne Node to Sensor Here
    for(int i = 0; i < sensors->SensorNum; i++){
      node[i].SensorLink = sensors->p+i;
      node[i].BendingState = NETURAL;
    }//for
  }// BendingState

  // Recognize every node's state  
  int EveryNodeState(){
    Strain *p;
    // Node 0:
    p = node[0].SensorLink;
    node[0].value = p->strain45;
    if(p->strain45 < -11){
      node[0].BendingState = HORIZON_CONVAX; 
    }
    else if(p->strain45 > 11){
      node[0].BendingState = HORIZON_CONCAVE;
    }
    if(p->strain45 < 10 && p->strain45 > -10 ){// netural
      node[0].BendingState = NETURAL;
    }

    // Node 1:
    p = node[1].SensorLink;
    node[1].value = p->AbsMaxStrain;
    /// Torsion
    if(p->strain_max > 15 && p->strain_min < -15){
      if(p->angle_p1_deg > 30 && p->angle_p1_deg < 70)
        node[1].BendingState = TORSION_POSTIVE;

      if(p->angle_p1_deg > 110 && p->angle_p1_deg < 150)
        node[1].BendingState = TORSION_NEGTIVE;
    }// if
    /// NETURAL STATE  
    if(p->AbsMaxStrain < 10 && p->AbsMaxStrain > -10 ){// netural
      node[1].BendingState = NETURAL;
    }

    // Node 2:
    p = node[2].SensorLink;
    node[2].value = p->AbsMaxStrain;
    /// vertical 
    if(p->angle_deg > 75 && p->angle_deg < 105){
      if(p->AbsMaxStrain < -15){
        node[2].BendingState = VERTICAL_CONVAX; 
      }
      else if(p->AbsMaxStrain > 15){
        node[2].BendingState = VERTICAL_CONCAVE;
      }
    }// if angle_deg

    /// 45 DEGREE 
    if(p->angle_deg > 30 && p->angle_deg < 70){
      if(p->AbsMaxStrain < -15){
        node[2].BendingState = CONVAX_45DEGREE; 
      }
      else if(p->AbsMaxStrain > 15){
        node[2].BendingState = CONCAVE_45DEGREE;
      }
    }// if angle_deg
    /// 135 DEGREE 
    if(p->angle_deg > 110 && p->angle_deg < 150){
      if(p->AbsMaxStrain < -15){// concave
        node[2].BendingState = CONVAX_135DEGREE; 
      }
      else if(p->AbsMaxStrain > 15){// convax
        node[2].BendingState = CONCAVE_135DEGREE;
      }
    }// if angle_deg
    /// Netural State
    if(p->AbsMaxStrain < 10 && p->AbsMaxStrain > -10 ){// netural
      node[2].BendingState = NETURAL;
    }

    // Node 3:
    p = node[3].SensorLink;
    node[3].value = p->strain45;
    if(p->strain45 < -11){
      node[3].BendingState = HORIZON_CONVAX; 
    }
    else if(p->strain45 > 11){
      node[3].BendingState = HORIZON_CONCAVE;
    }

    if(p->strain45 < 10 && p->strain45 > -10 ){// netural
      node[3].BendingState = NETURAL;
    }

    return 0; 
  }//EveryNodeState

  void PrintNodeState(int i){
    Serial.println("NodeState");
    Serial.println(int(i));
    Serial.println(int(node[i].BendingState));
  } //PrintNodeState

    int GestureRecognition( ){

    // 45 & 135 BENDING 
    if(node[0].BendingState == HORIZON_CONCAVE 
      && node[2].BendingState == CONCAVE_135DEGREE
      && node[3].BendingState == HORIZON_CONVAX){
      GestureFlag = 4;
      return 0;
    }   

    if(node[3].BendingState == HORIZON_CONCAVE 
      && node[2].BendingState == CONCAVE_45DEGREE
      && node[0].BendingState == HORIZON_CONVAX ){
      GestureFlag = 6;
      return 0;
    }        

    // Horizon Twist
    /*
    if(node[1].BendingState == TORSION_POSTIVE
     && (node[0].BendingState == NETURAL && node[3].BendingState == NETURAL) ){
     GestureFlag = 7;
     return 0;
     }   
     
     if(node[1].BendingState == TORSION_NEGTIVE
     && (node[0].BendingState == NETURAL && node[3].BendingState == NETURAL) ){
     GestureFlag = 8;
     return 0;
     }   
     */

/*
    if(node[1].BendingState == TORSION_POSTIVE)
      if( //abs(node[1].value) > abs(node[2].value) 
         abs(node[1].value) > abs(3*node[0].value) 
        || abs(node[1].value) > abs(3*node[3].value) ){
        GestureFlag = 7;
        return 0;
      }

    if(node[1].BendingState == TORSION_NEGTIVE)
      if( //abs(node[1].value) > abs(node[2].value) 
         abs(node[1].value) > abs(3*node[0].value) 
        || abs(node[1].value) > abs(3*node[3].value) ){
        GestureFlag = 8;
        return 0;
      }
*/
    // node 2
    switch(node[2].BendingState){
    case NETURAL:
      GestureFlag = 0;
      break;
    case VERTICAL_CONCAVE:
      GestureFlag = 1;
      return 0;
    case VERTICAL_CONVAX:
      GestureFlag = 2;
      return 0;
    case CONCAVE_45DEGREE:
      GestureFlag = 3;
      return 0;
    case CONVAX_45DEGREE:
      GestureFlag = 4;
      break;
    case CONCAVE_135DEGREE:
      GestureFlag = 5;
      return 0;
    case CONVAX_135DEGREE:
      GestureFlag = 6;
      break;
    }// (node[2].BendingState){

    /*      
     // node 3
     switch(node[3].BendingState){
     //case NETURAL:
     //break;
     case HORIZON_CONCAVE:
     GestureFlag = 3;
     break;
     case HORIZON_CONVAX:
     GestureFlag = 4;
     break;
     }// (node30].BendingState){  
     
     // node 0
     switch(node[0].BendingState){
     //case NETURAL:
     //break;
     case HORIZON_CONCAVE:
     GestureFlag = 5;
     break;
     case HORIZON_CONVAX:
     GestureFlag = 6;
     break;
     }// (node[0].BendingState){
     */
    return 0;
  }

  void PrintGesture(){
    Serial.println("Gesture");
    Serial.println(GestureFlag);
  } //PrintNodeState

};//class BendingState



#endif//_BENDINGSTATE_H_

