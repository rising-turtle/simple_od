/*
 * Apr.21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 * test controller 
 *
 * */


#include "control.h"
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>

using namespace std; 

int main(int argc, char* argv[])
{
  Controller cc; 

  double round = 180; 
  int n = 1; 

  if(argc >= 2)
    n = atoi(argv[1]); 
 
  // r = 0.028, 0.056 
  // L = 45 * 3 = 135 cm = 1.35 m 
  // alpha = 30' = PI/6
  // alpha * L = v * r 
  // v = n * 2 * PI
 // double L = 1.35 * cos(30*M_PI/180.)*0.75; // 
 // double r = 0.028; 
 // double alpha = n; // ;*M_PI/180.; 
 // int v = (int)(alpha * L / r); 
  double v = n;

  cout <<"input v = "<<v<<endl; 
  cc.rotateL(v); 
  usleep(2*1000*1000); 
  // cc.rotateR(n*round);

  /*
  double round = 360; 
  bool good = false; 
  for(int n = 5; n<20; n+=5)
  {
    good = cc.rotateL(n*round); 
    if(good)
    {
      cout <<"test_control.cpp: succeed rotateL: "<<n<<" rounds!"<<endl;
    }else
    {
      cout <<"test_control.cpp: failed rotateL: "<<n<<" rounds!"<<endl; 
    }
    
    usleep(2*1000*1000); 

 
    good = cc.rotateR(n*round); 
    if(good)
    {
      cout <<"test_control.cpp: succeed rotateR: "<<n<<" rounds!"<<endl;
    }else
    {
      cout <<"test_control.cpp: failed rotateR: "<<n<<" rounds!"<<endl; 
    }
    
    usleep(2*1000*1000); 
  }*/



  return 0; 
}
