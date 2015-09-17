 /* 
  *  2015/09/17 updated 
  *  version: V11
  *  Changed Item: Recognize states of joints(sensor) 
  */

#ifndef _STRAIN_H_
#define _STRAIN_H_

#include <Arduino.h>
#include <math.h>

struct Strain{
  int ID;

  int strain0;
  int strain45;
  int strain90;

  // principal strain
  int strain_min;
  int strain_max;
  // principal strain angle
  double angle_rad;
  double angle_deg;
  double angle_p1_deg ,angle_p2_deg;//angle for strain_max/strain_min
  // physic location
  int PositionX, PositionY;
  // for bending axis
  int AbsMaxStrain; 
  int BendAxisAngle_deg;
  int BendingStatusFlag;
  double x_BA, y_BA;
  // Bending state
  unsigned char  BendingState;
};

void PrinpicalStrain(Strain *p){

  //pre-caculate
  double s_center=0.5*(double(p->strain0)+double(p->strain90));
  double ds1=(double(p->strain0)-double(p->strain45));
  double ds1_sq=ds1*ds1;
  double ds2=(double(p->strain45)-double(p->strain90));
  double ds2_sq=ds2*ds2;
  double ds_sq_sum=ds1_sq+ds2_sq;
  //equation
  double strain_max=s_center+0.7071*sqrt(ds_sq_sum);
  double strain_min=s_center-0.7071*sqrt(ds_sq_sum);
  double x=(2*double(p->strain45)-double(p->strain0)-double(p->strain90))/(double(p->strain0)-double(p->strain90));
  double angle_rad=0.5*atan(x);
  double angle_deg=57.29747*angle_rad;
  double angle2_deg=2*angle_deg;
  //*
  //get angle 2theta in 1st_quadrant and 2nd_quadrant
  double angle_1st_quadrant, angle_2nd_quadrant;
  if(angle2_deg>=0 && angle2_deg<90){
    angle_1st_quadrant=angle2_deg*0.5;
    angle_2nd_quadrant=angle_1st_quadrant+90;
  }
  else if(angle2_deg<0 && angle2_deg>-90)
  {
    angle_1st_quadrant=(180+angle2_deg)*0.5;
    angle_2nd_quadrant=angle_1st_quadrant+90;
  }
  else
  {
    //debug
  }

  //estimate angle2_deg in which quadrant
  if(2*double(p->strain45)>double(p->strain0)+double(p->strain90)){
    p->angle_p1_deg = angle_1st_quadrant;
    p->angle_p2_deg = angle_2nd_quadrant;
  }
  else{
    p->angle_p1_deg = angle_2nd_quadrant;
    p->angle_p2_deg = angle_1st_quadrant;
  }

  // detect angle for abs max strain
  if(abs(strain_max)>abs(strain_min)){
    angle_deg = p->angle_p1_deg;
  }
  if(abs(strain_max)<abs(strain_min)){
    angle_deg = p->angle_p2_deg;
  }

  //store value
  p->strain_max=int(strain_max);
  p->strain_min=int(strain_min);
  //p->angle_rad=angle_rad;//<----------fix
  p->angle_deg = angle_deg;
  p->angle_rad = angle_deg * 0.01745;
}


void SensorPrint(Strain *p){
  Serial.println("ID");
  //Serial.println(p.ID);
  Serial.println(int(p->ID));
  //Serial.println("PositionX");
  Serial.println(p->PositionX);
  //Serial.println("PositionY");
  Serial.println(p->PositionY);
  //Serial.println("Strain0");
  Serial.println(int(p->strain0));
  //Serial.println("Strain45");
  Serial.println(int(p->strain45));
  //Serial.println("Strain90");
  Serial.println(int(p->strain90));
  //Serial.println("StrainP1");
  Serial.println(int(p->strain_max));
  //Serial.println("StrainP2");
  Serial.println(int(p->strain_min));
  //Serial.println("PAngle");
  Serial.println(int(p->angle_deg));
  Serial.println("NEXT");

  Serial.println(int(p->AbsMaxStrain));
  //Serial.println("Debug");//debug
  //Serial.println(int(p->AbsMaxStrain));//debug
}

class StrainSensors{
public:
  // Sensor
  Strain* p;
  Strain* vp;
  int SensorNum; 
  static const int Isostrain_th = 0;//Threshhold for charging strain status in single point 
  // virtual space
  int x_BA ,y_BA; // bending axis position
  int bending; // bending Strength
  int bending_degree_deg;
  int variance;
  //
  int MostActiveSensorIndex;
  
  //creat function
  StrainSensors(int SensorNum){
    this->SensorNum=SensorNum;
    p=new Strain[SensorNum];

    vp=new Strain;

    for(int i=0; i<SensorNum; i++){
      p[i].ID= i;
      p[i].strain0 = 0;
      p[i].strain45 = 0;
      p[i].strain90 = 0;
      p[i].strain_min = 0;
      p[i].strain_max = 0;
      p[i].angle_rad = 0;
      p[i].angle_deg = 0;
    }

  }//StrainSensors

  void AnalyzeOne(int SensorIndex){
    PrinpicalStrain(&p[SensorIndex]);
  }  

  void AnalyzeAll(){
    for(int i=0; i<SensorNum; i++){
      PrinpicalStrain(&p[i]);
    }
  }  

  void PrintOne(int SensorIndex){
    SensorPrint(&p[SensorIndex]);
  }

  void PrintAll(){
    for(int i=0; i<SensorNum; i++){
      SensorPrint(&p[i]);
    }
    Serial.println("NextFrame");// spacer between
  }

  // Bending Status Estimation
  void StrainStatus(){
    for(int i=0; i<SensorNum; i++){
      if(abs(p[i].strain_max) >= abs(p[i].strain_min) + Isostrain_th){
        p[i].AbsMaxStrain=p[i].strain_max;
      }
      else if(abs(p[i].strain_min) >= abs(p[i].strain_max) + Isostrain_th){ 
        p[i].AbsMaxStrain=p[i].strain_min;
      }
      else{
        p[i].AbsMaxStrain=0;
      }
    }//for
  }// StrainStatus()

  void ActiveCanditateGet(){
    // active element 
  }

  // caculate bending axis each sensor
  /*  
   void BendingAxis(int SensorIndex){
   int xs = p[SensorIndex].PositionX;
   int ys = p[SensorIndex].PositionY;
   double m = tan(p[SensorIndex].angle_rad);//slop
   
   //double x_ = (m*xs - ys)/(m + 1/m);
   double x_ = m*(m*xs - ys)/(m*m + 1);
   double y_ = m*(x_ - xs) + ys;
   
   p[SensorIndex].x_BA = int(x_);
   p[SensorIndex].y_BA = int(y_);
   }
   */
  void BendingAxis(int SensorIndex){
    double xs = double(p[SensorIndex].PositionX);
    double ys = double(p[SensorIndex].PositionY);
    double a= cos(p[SensorIndex].angle_rad);
    double b= sin(p[SensorIndex].angle_rad);
    //double a= cos(p[SensorIndex].angle_deg*3.1415/180);
    //double b= sin(p[SensorIndex].angle_deg*3.1415/180);
    double t = -(a*xs + b*ys);
    double x_ = xs + a*t;
    double y_ = ys + b*t;

    p[SensorIndex].x_BA = int(x_);
    p[SensorIndex].y_BA = int(y_);
  }
  
   void BendingAxis2(int SensorIndex){
    double xs = double(p[SensorIndex].PositionX);
    double ys = double(p[SensorIndex].PositionY);
    // bending direction
    double a= cos(p[SensorIndex].angle_rad + 0.5*PI);
    double b= sin(p[SensorIndex].angle_rad + 0.5*PI);
    // line equeation
    double t = -(a*xs + b*ys);
    double x_ = xs + a*t;
    double y_ = ys + b*t;

    p[SensorIndex].x_BA = int(x_);
    p[SensorIndex].y_BA = int(y_);
  }


  void BendingAxisAll(){
    for(int i = 0; i< SensorNum; i++){
      BendingAxis(i);
    }
  }

  // Estimate Bending Axis of Entire Frame
  void BendingAxisEstimation(){
    int x_BA_sum = 0;
    int y_BA_sum = 0;
    int bending_sum = 0;
    int bending_x_sum = 0;
    int bending_y_sum = 0;
    int a_sum = 0;
    int b_sum = 0;
    // Nomrmalization Const
    for(int i = 0; i < SensorNum; i++){
      bending_sum += abs(p[i].AbsMaxStrain);
      a_sum += p[i].x_BA;
      b_sum += p[i].y_BA;
    } 
    // Sampling Sum
    for(int i = 0; i < SensorNum; i++){
      x_BA_sum += p[i].x_BA * abs(p[i].AbsMaxStrain);
      y_BA_sum += p[i].y_BA * abs(p[i].AbsMaxStrain);
      bending_x_sum += p[i].AbsMaxStrain * p[i].x_BA;
      bending_y_sum += p[i].AbsMaxStrain * p[i].y_BA;
    }
    // Expectation
    double x_BA_ = x_BA_sum  / bending_sum;
    double y_BA_ = y_BA_sum  / bending_sum;

    double bending_x = bending_x_sum / a_sum;
    double bending_y = bending_y_sum / b_sum;

    double bending_ = bending_sum / SensorNum;
    // Variance
    double variance_ = 0;
    double D;
    for(int i = 0; i < SensorNum; i++){
      D = bending_ - p[i].AbsMaxStrain;
      variance_ += D*D;
    }
    variance_ = variance_ / SensorNum;
    // return
    x_BA = int(x_BA_);
    y_BA = int(y_BA_);
    bending = int(bending_);
    variance = int(variance_);
  }

  // Estimate Bending Axis of Entire Frame
  void BendingAxisEstimation2(){
    int M_sum = 0;
    int Ma_sum = 0;
    int Mb_sum = 0;
    double Mi_sum = 0;
    double Mj_sum = 0;
    // Nomrmalization Const
    for(int i = 0; i < SensorNum; i++){
      M_sum += p[i].AbsMaxStrain;
    } 
    // Sampling Sum
    double m ,n;
    for(int i = 0; i < SensorNum; i++){
      Ma_sum += p[i].x_BA * p[i].AbsMaxStrain;
      Mb_sum += p[i].y_BA * p[i].AbsMaxStrain; 
      m = cos(p[i].angle_rad);
      n = sin(p[i].angle_rad);
      //m = (cos(p[i].angle_deg*3.1415/180));
      //n = (sin(p[i].angle_deg*3.1415/180));
      Mi_sum += abs(p[i].AbsMaxStrain) * m;
      Mj_sum += abs(p[i].AbsMaxStrain) * n;
    }
    // Expectation
    double x_BA_ = Ma_sum  / M_sum;
    double y_BA_ = Mb_sum  / M_sum;
    double Mi = Mi_sum /  SensorNum;
    double Mj = Mj_sum /  SensorNum;

    double M_ = 0;
    if(Mi != 0 && Mj != 0){
      M_ = Mi*Mi + Mj*Mj;
      M_ = sqrt(M_);
    }
    double M_degree_rad = 0;
    if(Mi != 0){
      //Mi = abs(Mi);
      //Mj = abs(Mj); 
      M_degree_rad = atan2(Mj, Mi);
    }
    else{
      M_degree_rad =  1.07;
    }  
    // Variance


    // Re-asign Bending Direction 
    if(M_sum < 0){
      M_ = -M_;
    }
    // update
    x_BA = int(x_BA_);
    y_BA = int(y_BA_);
    bending = int(M_);
    bending_degree_deg = int(180 * M_degree_rad / PI);
    //variance = int(variance_);
  }


  void BendingAxisEstimation3(){//dispose
    int M_sum = 0;
    int Ma_sum = 0;
    int Mb_sum = 0;
    double Mi_sum = 0;
    double Mj_sum = 0;
    // Nomrmalization Const
    for(int i = 0; i < SensorNum; i++){
      M_sum += p[i].AbsMaxStrain;
    } 
    // Sampling Sum
    Serial.println("test");
    double m, n;
    for(int i = 0; i < SensorNum; i++){
      Ma_sum += p[i].x_BA * p[i].AbsMaxStrain;
      Mb_sum += p[i].y_BA * p[i].AbsMaxStrain; 
      //m = p[i].angle_rad;//<-------
      m = p[i].angle_deg;
      //Mi_sum += p[i].AbsMaxStrain * m;
      Mi_sum += m;
      Serial.println(m);
      //n =(180 * m / 3.14);
      //Serial.println(n);
    }

    // Expectation
    double x_BA_ = Ma_sum  / M_sum;
    double y_BA_ = Mb_sum  / M_sum;
    //double M_degree_rad = Mi_sum /  M_sum;
    double M_degree_deg_ = Mi_sum /  4; 

    // Variance

    // update
    x_BA = int(x_BA_);
    y_BA = int(y_BA_);
    bending = int(20);
    bending_degree_deg = int(M_degree_deg_);
    //variance = int(variance_);

    Serial.println(M_sum);
    Serial.println(Mi_sum);
    Serial.println("testend");
  }

  void PrintBendingAxis(int SensorIndex){
    Serial.println("BendingAxis");
    Serial.println(int(p[SensorIndex].x_BA));
    Serial.println(int(p[SensorIndex].y_BA));
    Serial.println("BendingAxisEnd");
  }

  void PrintBendingAxisAll(){
    for(int i = 0; i< SensorNum; i++){
      PrintBendingAxis(i);
    }
  }

  void PrintEstimateBendingAxis(){
    Serial.println("EstimateBendingAxis");
    Serial.println(int(x_BA));
    Serial.println(int(y_BA));
    Serial.println(int(bending));
    Serial.println(int(bending_degree_deg));
    Serial.println("EstimateBendingAxisEnd");
  }

  // create virtual sensor for bending status estimation
  void VirtualSensor(){
    int strain_E;
    int strain_E_sum = 0;
    double angle_deg_sum = 0;
    int PositionX_sum = 0;
    int PositionY_sum = 0;
    int strain_min_sum = 0;
    int strain_max_sum = 0;

    // Nomrmalization Const
    for(int i=0; i < SensorNum; i++){
      strain_E = abs(p[i].strain_min)+abs(p[i].strain_max);
      strain_E_sum += strain_E;
    } 
    // Sampling Sum
    for(int i=0; i < SensorNum; i++){
      strain_E = abs(p[i].strain_min)+abs(p[i].strain_max);
      angle_deg_sum += strain_E * p[i].angle_deg;
      strain_min_sum += strain_E * p[i].strain_min;
      strain_max_sum += strain_E * p[i].strain_max;
      PositionX_sum += int(strain_E * p[i].PositionX);
      PositionY_sum += int(strain_E * p[i].PositionY);
    }
    // Expectation
    vp->angle_deg = angle_deg_sum  / strain_E_sum;
    vp->strain_min = strain_min_sum  / strain_E_sum;
    vp->strain_max = strain_max_sum  / strain_E_sum;
    vp->PositionX = PositionX_sum  / strain_E_sum;
    vp->PositionY = PositionY_sum  / strain_E_sum;

  }//VirtualSensor

  void VirtualSensorMax(){
    int AbsMaxStrain_Max = 0;
    int Index;
    // Find out most active sensor
    for(int i=0; i < SensorNum; i++){
      if( AbsMaxStrain_Max <  abs(p[i].AbsMaxStrain) ){
        AbsMaxStrain_Max = abs(p[i].AbsMaxStrain);
        Index = i;
      } //if
    } //for
    
    // update virtual sensor
    vp = &p[Index];
    MostActiveSensorIndex = Index;
  }//VirtualSensorMax


  // print out information of Virtual sensor
  void PrintVirtual(){
    SensorPrint(vp);
    Serial.println("ActiveSensorIndex");
    Serial.println(int(MostActiveSensorIndex));
    Serial.println("ActiveSensorIndexEnd");
    //Serial.println("NextFrame");// spacer between
  }

 void BendingState(){
   
   
 }


};// class StrainSensors



#endif//_STRAIN_H_







