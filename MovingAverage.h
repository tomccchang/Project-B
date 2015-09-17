
 /* 
  *  2015/09/17 updated 
  *  version: ?
  *  Changed Item: ?
  */

#ifndef _MOVINGAVERAGE_H_
#define _MOVINGAVERAGE_H_

#include <Arduino.h>

class MovingAverage{
public:  
  int SampleNum;
  int *SmoothWindowBuffer0,*SmoothWindowBuffer1,*SmoothWindowBuffer2;
  int CH0,CH1,CH2;
  unsigned int DelayTime_us;
  int BufferCount;
  int AvgValue0, AvgValue1, AvgValue2;

  //create function
  MovingAverage(int SampleNum, int CH0, int CH1, int CH2){
    this->SampleNum=SampleNum;
    SmoothWindowBuffer0=new int[int(SampleNum)];
    SmoothWindowBuffer1=new int[int(SampleNum)];
    SmoothWindowBuffer2=new int[int(SampleNum)];
    this->CH0=CH0;
    this->CH1=CH1;
    this->CH2=CH2;    
    this->DelayTime_us=0;
    this->BufferCount=0;
  }

  MovingAverage(int SampleNum, int CH0){
    this->SampleNum=SampleNum;
    SmoothWindowBuffer0=new int[int(SampleNum)];
    this->CH0=CH0; 
    this->DelayTime_us=0;
    this->BufferCount=0;
  }

public:

  void DelaySet(unsigned int DelayTime_us){
    this->DelayTime_us=DelayTime_us; 
  }

  void UpdateALL(){
    //pre-set the buffer
    for(int i=0; i<SampleNum;i++){
      SmoothWindowBuffer0[i] = analogRead(CH0);
      SmoothWindowBuffer1[i] = analogRead(CH1);
      SmoothWindowBuffer2[i] = analogRead(CH2);
      delayMicroseconds(DelayTime_us);
    }//for
  }

  void UpdateAllOne(){
    //pre-set the buffer
    for(int i=0; i<SampleNum;i++){
      SmoothWindowBuffer0[i] = analogRead(CH0);
      delayMicroseconds(DelayTime_us);
    }//for
  }

  void average(){
    int SmoothWindowSum0=0;
    int SmoothWindowSum1=0;
    int SmoothWindowSum2=0;
    for(int i=0; i<SampleNum;i++){
      SmoothWindowSum0+=SmoothWindowBuffer0[i];
      SmoothWindowSum1+=SmoothWindowBuffer1[i];
      SmoothWindowSum2+=SmoothWindowBuffer2[i];
    }
    AvgValue0=SmoothWindowSum0/SampleNum;
    AvgValue1=SmoothWindowSum1/SampleNum;
    AvgValue2=SmoothWindowSum2/SampleNum;
  }

  void AverageOne(){
    int SmoothWindowSum0=0;
    for(int i=0; i<SampleNum;i++){
      SmoothWindowSum0+=SmoothWindowBuffer0[i];
    }
    AvgValue0=SmoothWindowSum0/SampleNum;
  }

  void update(){
    //Sampling and Save data in buffer
    SmoothWindowBuffer0[BufferCount]=analogRead(CH0);
    SmoothWindowBuffer1[BufferCount]=analogRead(CH1);
    SmoothWindowBuffer2[BufferCount]=analogRead(CH2);
    BufferCount++;

    ///BufferCount is looping with 0~SampleNum
    if(BufferCount==SampleNum)
      BufferCount=0;
  }

  void UpdateOne(){
    //Sampling and Save data in buffer
    SmoothWindowBuffer0[BufferCount]=analogRead(CH0);
    BufferCount++;
    ///BufferCount is looping with 0~SampleNum
    if(BufferCount==SampleNum)
      BufferCount=0;
  }

};

#endif//_MOVINGAVERAGE_H_

