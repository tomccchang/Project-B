/* external mux sheld version 
 * temp for one sensor (for testing and building bending status estimation algorithm)  
 * gages is sensed in serie.
 * Recognize states of joints(sensor) 
 * Gesture Reognition in Cascade Classifyer
 */
#include "MUXControl.h"// MUX V1
#include "MuxShield.h"// MUX V2
#include "MovingAverage.h" 
#include "PrincipalStrain.h"
#include "BendingState.h"
#include "KeyboardReport.h"
// Input Channel Setting
int CH0 = 0; // select the input pin for the sensor
int CH1 = 1; // select the input pin for the sensor
int CH2 = 2; // select the input pin for the sensor

//Moving Average setting
double Noise_Freq=240;// power frequency 60 HZ
double SampleNum=3;// sample pts number for smooth window
///regester
unsigned int DelayTime_us=0; //us
double DelayTime_ms=0; //ms

const int AReadDelay_us = 10000000;
const int AReadDelay = 12;

//Sensor Value Init
const int SensorNum=4;
StrainSensors strain(SensorNum);
//strain.p[0].PositionX=-20;
//gesture analyzer
//Gesture ge;
//Moving Average
//MovingAverage MA( SampleNum, CH0, CH1, CH2);
MovingAverage MA( SampleNum, CH0);
//MUX shiled control
MUXShield mux(5,4,3,2); // MUX V1
MuxShield muxShield; // MUX V2
//Bending state 
BendingState BS(&strain);
//keyboard simulation
KeyboardSim keyboard;

// buffer  
/// Netural value
//int NeturalLev0=0, NeturalLev1=0, NeturalLev2=0;
int NeturalLev[3][SensorNum];

/// devation between measurement and Netural value
int DevLev[3][SensorNum];

void setup() {

 strain.p[0].PositionX = -30;
 strain.p[0].PositionY = 30;
 strain.p[1].PositionX = 0;
 strain.p[1].PositionY = -30;
 strain.p[2].PositionX = 0;
 strain.p[2].PositionY = 0;
 strain.p[3].PositionX = 30;
 strain.p[3].PositionY = 30; 
/*    

 strain.p[0].PositionX = 0;
 strain.p[0].PositionY = 50;
 strain.p[1].PositionX = 0;
 strain.p[1].PositionY = 0;
 strain.p[2].PositionX = 50;
 strain.p[2].PositionY = 0;
 strain.p[3].PositionX = 50;
 strain.p[3].PositionY = 50; 
 
 strain.p[0].PositionX = -20;
 strain.p[0].PositionY = 20;
 strain.p[1].PositionX = -20;
 strain.p[1].PositionY = -20;
 strain.p[2].PositionX = 20;
 strain.p[2].PositionY = -20;
 strain.p[3].PositionX = 20;
 strain.p[3].PositionY = 20; 
/*/
 
  Serial.begin(9600);
  //set delay time
  DelayTime_ms=1000/(Noise_Freq*SampleNum);
  DelayTime_us=(unsigned long)(DelayTime_ms*1000);
  MA.DelaySet(DelayTime_us);
 
  //PRESET MUX
  ///V2
  muxShield.setMode(1,ANALOG_IN);
  muxShield.setMode(2,ANALOG_IN);
  //muxShield.setMode(3,ANALOG_IN);  
  delay(50);//for stablization

  for(int i=0; i<SensorNum;i++){
    // 0 degree (MUX M1/M2 CH0)
    mux.select(3*i+0+1);
    delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    NeturalLev[0][i]=MA.AvgValue0;
    // 45 degree (MUX M1/M2 CH0)
    mux.select(3*i+1+1);
    delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    NeturalLev[1][i]=MA.AvgValue0;
    // 90 degree (MUX M1/M2 CH0)
    mux.select(3*i+2+1);
    delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    NeturalLev[2][i]=MA.AvgValue0;
  }//for
}//setup()

void loop() {
  //Get moving average of each frame
   for(int i=0; i<SensorNum;i++){
    // 0 degree (MUX M1/M2 CH0)
    mux.select(3*i+0+1);
   // delayMicroseconds(AReadDelay_us);
   delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    DevLev[0][i] = MA.AvgValue0 - NeturalLev[0][i];
    
    // 45 degree (MUX M1/M2 CH0)
    mux.select(3*i+1+1);
    //delayMicroseconds(AReadDelay_us);
    delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    DevLev[1][i] = MA.AvgValue0 - NeturalLev[1][i];
    // 90 degree (MUX M1/M2 CH0)
    mux.select(3*i+2+1);
    //delayMicroseconds(AReadDelay_us);
    delay(AReadDelay);
    MA.UpdateAllOne();
    MA.AverageOne();
    DevLev[2][i] = MA.AvgValue0 - NeturalLev[2][i];
    
    // Pass Deviation to Strain Field Analysis    
    strain.p[i].strain0 = int(DevLev[0][i]);//<-----Four Sensor
    strain.p[i].strain45 = int(DevLev[2][i]);
    strain.p[i].strain90 = int(DevLev[1][i]);
    
    //strain.p[i].strain0 = int(DevLev[2][i]);//<----- One Sensor
    //strain.p[i].strain45 = int(DevLev[0][i]);
    //strain.p[i].strain90 = int(DevLev[1][i]);
  }//for

  // Repair
  /// Fix Configuration of Sensor 0
  int buffer1 = strain.p[0].strain0;
  strain.p[0].strain0 = strain.p[0].strain45;
  strain.p[0].strain45 = buffer1;
  
  // Strain Analysis
  //strain.AnalyzeOne(2);
  strain.AnalyzeAll();
  
  // Bending Stauts Estimation
  //strain.VirtualSensor();
  strain.StrainStatus();
  //strain.VirtualSensorMax();

  //strain.BendingAxisAll();
  //strain.BendingAxisEstimation2();
  
  //Gesture Recognition 
   BS.EveryNodeState();
   BS.GestureRecognition();
  
  //Output (debug mode)
  //strain.PrintOne(0);

  //strain.PrintAll();
  //BS.PrintNodeState(0);
  //BS.PrintNodeState(1);
  //BS.PrintNodeState(2);
  //BS.PrintNodeState(3);
  //BS.PrintGesture();

  // Test Algorithms
  //strain.PrintVirtual();
  //strain.PrintBendingAxis(0);
  //strain.PrintBendingAxisAll();
  //strain.PrintEstimateBendingAxis();
  
  // Output (keyboard mode)
  keyboard.report((unsigned char)BS.GestureFlag);
}
