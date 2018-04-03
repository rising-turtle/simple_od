/*
 *  Apr. 21 2017
 *  Modified by He Zhang, hxzhang1@ualr.edu 
 *  
 *  TODO: adjust rotate velocity 
 *
 * */

#include "motor.h"

using namespace std; 
CMotor::CMotor(double t, std::string port) : 
  PORT(port), 
  Baud(t), 
  maxspeed(100)
{
  std::cout << "The motor is=> Baud "<< Baud<<" Port "<< PORT<<std::endl;
  _motor.Open(PORT);		

  if ( ! _motor.good() ){
    std::cerr << "Error: Could not open serial port "<<PORT<< std::endl ;
    exit(1) ;
  }

  if(Baud==115200){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
    std::cout<<"Going for value of 115200"<<std::endl;
  }
  else if(Baud==1200){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_1200);
    std::cout<<"Going for value of 1200"<<std::endl;
  }
  else if(Baud==2400){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_2400);
    std::cout<<"Going for value of 2400"<<std::endl;
  }
  else if(Baud==19200){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_19200);
    std::cout<<"Going for value of 19200"<<std::endl;
  }
  else if(Baud==38400){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_38400);
    std::cout<<"Going for value of 38400"<<std::endl;
  }
  else if(Baud==57600){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_57600);
    std::cout<<"Going for value of 57600"<<std::endl;
  }
  else if(Baud==9600){
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
    std::cout<<"Going for default value of 9600"<<std::endl;
  }
  else{
    std::cout<<"Value incorrect. Going for default value of 9600"<<std::endl;
    _motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
  }

  if ( ! _motor.good() ){
    std::cerr << "Error: Could not set the baud rate." <<std::endl;
    exit(1);
  }		


  _motor.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
  if ( ! _motor.good() ){
    std::cerr << "Error: Could not set the char size." <<std::endl;
    exit(1);
  }
  _motor.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE ) ;
  if ( ! _motor.good() ){
    std::cerr << "Error: Could not set the parity." <<std::endl;
    exit(1);
  }
  _motor.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
  if ( ! _motor.good() ){
    std::cerr << "Error: Could not set the control." <<std::endl;
    exit(1);
  }
  _motor.SetNumOfStopBits(1) ;
  if ( ! _motor.good() ){
    std::cerr << "Error: Could not set the stopbit." <<std::endl;
    exit(1);
  }

  //setMax Min speed
  writeDisable(); //Stop the motor in case the robot was still moving
  writeEnable();
  writeHome(); //Set the initial position to zero ;)

  writeTargetSpeed(50);
  //writeMaxSpeed(maxspeed);
  //writeMinSpeed(-maxspeed);
  
  // change speed 
  std::string cmd = "VOLTMOD\nV10\nCONTMOD\n"; 
  // writePort(cmd); 
  // cmd  = "VOLTMOD\n"; 
  // writePort(cmd);
  // cmd = "SP100\n";
  // writePort(cmd);
  // cmd = "CONTMOD\n";
  // writePort(cmd);
  // cmd = "en0\n";
  // writePort(cmd); 
  // cmd = "ho\n";
  // writePort(cmd); 
}

CMotor::~CMotor(){
  _motor.Close();
}


void CMotor::writeMaxSpeed(int ms){
  try{
    std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
    MaxSpeed="SP"+MaxSpeed+"\n";
    writePort(MaxSpeed);
  }
  catch(boost::bad_lexical_cast& blc){
    std::cout << "Exception in Max Speed" << blc.what() << std::endl;
    _readcorrectly=false;
    //	  scanf("%d",&testin);
  }
}
void CMotor::writeMinSpeed(int ms){
  try{
    std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
    MaxSpeed="MV"+MaxSpeed+"\n";
    writePort(MaxSpeed);
  }
  catch(boost::bad_lexical_cast& blc){
    std::cout << "Exception in Min Speed" << blc.what() << std::endl;
    //	  scanf("%d",&testin);
    _readcorrectly=false;
  }
}

void CMotor::writeTargetSpeed(int ms){
  try{
    std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
    MaxSpeed="V"+MaxSpeed+"\n";
    writePort(MaxSpeed);
  }
  catch(boost::bad_lexical_cast& blc){
    std::cout << "Exception in Right wheel Speed" << blc.what() << std::endl;
    //	  scanf("%d",&testin);
    _readcorrectly=false;
  }
}

bool CMotor::writePoseRelative(int ms){
  try{
    std::string pos=boost::lexical_cast<std::string>(ms);
    pos="LR"+pos+"\n";
    writePort(pos);
    // int before_n = readEncoder(); 
    int targetPos = ms+readEncoder();

    return motorMove(targetPos);
    // motorMove(targetPos); 
    // int after_n = readEncoder(); 
    // cout <<"before "<<before_n<<" after "<<after_n<<endl; 
    // return true; 
  }
  catch(boost::bad_lexical_cast& blc){
    std::cout << "Exception in Relative Right position" << blc.what() << std::endl;
    // scanf("%d",&testin);
    _readcorrectly=false;
  }
  return false ; 
}

bool CMotor::writePoseAbsolute(int ms){
  try{
    std::string pos=boost::lexical_cast<std::string>(ms);
    pos="LA"+pos+"\n";

    writePort(pos);
    // std::cout << "CMD = "<< pos<<std::endl;

    return motorMove(ms);
  }
  catch(boost::bad_lexical_cast& blc){
    std::cout << "Exception in Absolute Right Position" << blc.what() << std::endl;
    //scanf("%d",&testin);
    _readcorrectly=false;
  }
  return false ; 
}


bool CMotor::motorMove(int targetPos)
{
  std::string pos="M\n";
  writePort(pos);
  //std::cout << "CMD = "<< pos<<std::endl;
  //sleep(4);
  int curPos = readEncoder();
  //std::cout << "curPos = "<< curPos<<std::endl;
  //sstd::cout << "targetPos = "<< targetPos<<std::endl;
  int stopCode = 0;
  int lastPos = 0;
  while((curPos-targetPos)>10 || (curPos-targetPos)<-10 )
  {
    usleep(50);
    lastPos = curPos;
    curPos = readEncoder();
    //std::cout << "curPos = "<< curPos<<std::endl;
    //std::cout << "lastPos = "<< lastPos<<std::endl;

    if((curPos-lastPos)<2 && (curPos-lastPos)>-2 ){
      stopCode ++;
    }
    else{
      stopCode = 0;
    }

    if(stopCode > 5){
      pos="V0\n";
      writePort(pos);
      std::cout << "Stopped!!!"<< std::endl;
      if ((curPos-targetPos)>10 || (curPos-targetPos)<-10){
        std::cout << "curPos = "<< curPos<<std::endl;
        std::cout << "targetPos = "<< targetPos<<std::endl;
        std::cout << "Stuck, failed to move!!!"<< std::endl;
        // for (int i=0 ; i<5; i++){
        //   std::cout << 5-i << std::endl;
        //  sleep(1);
        // }
        return false; 
      }
      // pos="EN0\n";
      // writePort(pos);
      // might need to release the clutch and re-enable the motor
      break;
    }
  }
  return true; 
}


void CMotor::writeMove()
{
  std::string s="M\n";
}




