/*
 *  Apr. 3 2018, He Zhang, hxzhang1@ualr.edu 
 *
 *  interface to control the glove 
 *
 * */

#include "glove_control.h"
#include <iostream>

using namespace std; 

CGloveControl::CGloveControl()
{
  init(); 
}

CGloveControl::~CGloveControl()
{
  uninit(); 
}

void CGloveControl::init()
{
  mpMotorLR = new Controller; 
  // mpMotorFB = new Controller; 
}

void CGloveControl::uninit()
{
  if(mpMotorLR != NULL) 
  {
    cout <<"glove_control.cpp: in uninit()!"<<endl; 
    mpMotorLR->uninit(); 
    delete mpMotorLR;
  }
  // if(mpMotorFB == NULL) delete mpMotorFB; 
}

// move distance between [0, 1]
void CGloveControl::moveLeft(double r)
{
  double n = r * 180.; 
  mpMotorLR->rotateL(n); 
  return ; 
}

void CGloveControl::moveRight(double r)
{
  return moveLeft(-r); 
}

void CGloveControl::moveFront(double r)
{
  // TODO
}

void CGloveControl::moveBack(double r)
{
  // TODO
}

