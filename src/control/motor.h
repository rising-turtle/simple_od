/*
 *  Apr. 21 2017
 *  Modified by He Zhang, hxzhang1@ualr.edu 
 *  
 *  TODO: adjust rotate velocity 
 *
 * */

// #ifndef _SERIALPORTCONTROL_HPP
// #define _SERIALPORTCONTROL_HPP
#pragma once 

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <deque>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <SerialStream.h>

#include <boost/lexical_cast.hpp>

using namespace std; 

class CMotor{
	
  public:
    CMotor(double t= 9600, string port="/dev/ttyUSB0"); 
    virtual ~CMotor(); 
  protected :
    //ATTRIBUTS 

    double _nTick; // ??? still have no idea of what this parameter is

    LibSerial::SerialStream _motor;

    int maxspeed; // check the faulhaber document!!!

    int _enco;
    float _targetSpeed; 
    float _measuredSpeed; 

    bool _verbose;
    bool _readcorrectly;

    std::string PORT;
    double Baud;	

    public:

    /**************************************************
      Accessors
     **************************************************/
    int getEncoder(){return _enco;}
    float getTargetSpeed(){return _targetSpeed;}
    float getMeasuredSpeed(){return _measuredSpeed;}
    LibSerial::SerialStream& getMotor(){return _motor;}
    double getTickNumber(){return _nTick;}
    bool getReadState(){return _readcorrectly;}
    /**************************************************
      Read values in the controller{
      std::string s="M\n";
    }

     **************************************************/
    int readEncoder(); 
    int readTargetSpeed();
    int readRealSpeed();
    int readEncoderResolution();

    /**************************************************
      Set values Manually
     **************************************************/	
    void setTargetSpeed(float s){_targetSpeed=s;}
    void setVerbose(){_verbose=true;}
    void setReadState(){_readcorrectly=true;}

    /**************************************************
      Write values in the controller
     **************************************************/
    void writeEnable();
    void writeDisable();
    void writeGoEncoderIndex();
    void writeHome();
    void writeAcc(int acc);
    void writeDec(int dec);

    void writeMaxSpeed(int ms);
    void writeMinSpeed(int ms);
    void writeTargetSpeed(int ms); 
    bool writePoseRelative(int ms); 
    bool writePoseAbsolute(int ms); 
    bool motorMove(int targetPos); 
    void writeMove();

    /**************************************************
      Basic serial operations
     **************************************************/
    void writePort(std::string& str);
    int readPort();
};

/*********************************************************************************************************
FUNCTIONS
*********************************************************************************************************/

/********************************Read values in the controller**************************************/


inline int CMotor::readEncoder(){
	std::string yo("POS\n");
	writePort(yo);
	return readPort();
}

inline int CMotor::readEncoderResolution(){
	std::string yo("GENCRES\n");
	writePort(yo);
	std::string i;
	_motor>> i;
	while(_motor.rdbuf()->in_avail() >0){
		_motor>> i;
		//std::cout <<i<<std::endl;
		try{
			return boost::lexical_cast<int>(i);
		}
		catch(boost::bad_lexical_cast& blc){
		  continue;
		}
	}

}


inline int CMotor::readTargetSpeed(){
	std::string tSpeed("GV\n");
	writePort(tSpeed);
	return readPort();
}


inline int CMotor::readRealSpeed(){	
	std::string rSpeed("GN\n");
	writePort(rSpeed);
	return readPort();
}

/*****************************CFULHABER CONTROLER CONFIGURATION*******************************/

inline void CMotor::writeEnable(){
	std::string enable("en\n");
	CMotor::writePort(enable);
	std::cout << "motor enabled " << std::endl;
}

inline void CMotor::writeDisable(){
	std::string enable("di\n");
	CMotor::writePort(enable);
	std::cout << "motor disabled " << std::endl;
}

inline void CMotor::writeGoEncoderIndex(){
	std::string enable("GOIX\n");
	CMotor::writePort(enable);
}
inline void CMotor::writeHome(){
	std::string enable("HO\n");
	CMotor::writePort(enable);
}
inline void CMotor::writeAcc(int acc){
	std::string accc=boost::lexical_cast<std::string>(acc);
	std::string enable("AC"+accc+"\n");
	CMotor::writePort(enable);
}
inline void CMotor::writeDec(int dec){
	std::string decc=boost::lexical_cast<std::string>(dec);
	std::string enable("DEC"+decc+"\n");
	CMotor::writePort(enable);
}

/****************************Basic functon*******************************************/

inline void CMotor::writePort(std::string& str){
	_motor << str;//lpBufferToWrite;
	// if(_verbose==true){
	// 	std::cout << "wrote the demand: "<<str.c_str()<< std::endl;
	// }
}

inline int CMotor::readPort(){
	std::string i;
	while(_motor.rdbuf()->in_avail() >0){
		_motor>> i;
		try{
			return boost::lexical_cast<int>(i);
		}
		catch(boost::bad_lexical_cast& blc){
		  continue;
		}
	}	
}

