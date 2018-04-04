/*
 *  Apr. 3 2018, He Zhang, hxzhang1@ualr.edu 
 *
 *  interface to control the glove 
 *
 * */


#pragma once 

#include <vector>
#include <cstddef>
#include "control/control.h"

class Controller; 

using namespace std; 

class CGloveControl
{
  public:
    CGloveControl();
    virtual ~CGloveControl(); 
    
    void init(); 
    void uninit(); 

    // move to track the object 
    void moveLeft(double );
    void moveRight(double );
    void moveFront(double);
    void moveBack(double); 

    // controller 
    Controller* mpMotorLR; // control movement left/right 
    Controller* mpMotorFB; // control movement front/back 
};




