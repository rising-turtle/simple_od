/*  
 *  Apr. 12 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  controller for motor and clutch 
 *
 * */

#include "control.h"
#include "motor.h"
// #include "clutch.h"
#include <iostream>

using namespace std; 

Controller::Controller()
{
  init(); 
}

Controller::~Controller()
{
  uninit(); 
}


void Controller::init()
{
  mpMotor = new CMotor; 
  mNPerDegree = mpMotor->readEncoderResolution(); // N per 3.6 degree
  cout << "control.cpp: Resolution is "<<mNPerDegree<<" per 3.6 degree"<<endl;
  mNPerDegree /= 3.6;
  // mpClutch = new Clutch; 
  mbOK = true; // mpClutch->mbOK; 
  cout << "control.cpp: initialization succeed! NPerDegree = "<<mNPerDegree<<endl; 
  return; 
}


void Controller::uninit()
{
  if(mpMotor) {delete mpMotor; mpMotor = 0;}
 // if(mpClutch) {delete mpClutch; mpClutch = 0;}
  mbOK = false; 
  return ; 
}

bool Controller::rotateL(double angle)
{
  if(!mbOK)
  {
    cerr <<"control.cpp: device not work !"<<endl; 
    return false; 
  }
  int targetpos = angle * mNPerDegree; 
 // mpClutch->clutchOn(); 
  bool succeed = mpMotor->writePoseRelative(targetpos); 
 // mpClutch->clutchOff(); 
  if(!succeed)
  {
    cout <<"control.cpp rotate about "<<angle<<" degree maybe stuck!"<<endl; 
    usleep(100*1000); // sleep 100 ms
    return false; 
  }

  return true; 
}

bool Controller::stop(int seconds)
{
  if(!mbOK)
  {
    cerr <<"control.cpp: device not work !"<<endl; 
    return false; 
  }
 // mpClutch->clutchOn(); 
  usleep(seconds * 1000 * 1000); 
  // mpClutch->clutchOff(); 
  return true; 
}

// degree < 0, rotate to right 
// degree > 0, rotate to left
bool Controller::rotateCane(double degree)
{
  // constrain rotate angle 
  // if(degree > 25) degree = 25; 
  // else if(degree <-25) degree = -25; 

  // r = 0.028, 0.056 
  // L = 45 * 3 = 135 cm = 1.35 m 
  // alpha = 30' = PI/6
  // alpha * L = v * r 
  // v = n * 2 * PI
  double L = 1.35 * cos(30*M_PI/180.)*0.75; // 
  double r = 0.028; 
  double alpha = degree; // ;*M_PI/180.; 
  int v = (int)(alpha * L / r); 
  cout << "degree = "<<degree<<" input v = "<<v<<endl; 
  rotateL(v); 
  // usleep(2*1000*1000); 
  
  return true; 
}

bool Controller::rotateR(double angle)
{
  return rotateL(-angle); 
}







