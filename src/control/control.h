/*  
 *  Apr. 12 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  controller for motor and clutch 
 *
 * */

#pragma once 

// class Clutch; 
class CMotor; 

class Controller 
{
  public: 
    Controller(); 
    virtual ~Controller(); 

    void init(); 
    void uninit(); 

    // Clutch* mpClutch; 
    CMotor* mpMotor; 
    
    double mNPerDegree; // N = angle * N/degree 
    bool mbOK; // whether they all work fine 
    
    bool rotateL(double angle);  // rotate to left with angle for the rolling tip 
    bool rotateR(double angle);  // rotate to right with angle for the rolling tip 

    bool rotateCane(double degree);   // rotate the cane degree
    
    bool stop(int seconds = 2);     // stop for seconds

};
