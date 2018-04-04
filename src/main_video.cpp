/*
 * Apr. 3 2018, He Zhang, hxzhang1@ualr.edu 
 *
 * put imgs into video
 *
 * */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>

using namespace std; 
using namespace cv; 

void push_folder_into_video(); 


int main(int argc, char* argv)
{
  push_folder_into_video(); 

  return 1; 
}

void push_folder_into_video()
{
  Mat img; 
  int strat_cnt = 2;
  int end_cnt = 60; 
  
  VideoWriter* mpVideo = new cv::VideoWriter; 
  Size sz(640, 480);
  // mpVideo->open("video.avi", CV_FOURCC('X','V','I','0'), 30, sz, true); 
  mpVideo->open("out_video.avi", -1, 30, sz, true); 

  if(!mpVideo->isOpened())
  {
    cout<<"main_video: failed to open video "<<endl;
    return ;
  }
  for(int i=strat_cnt; i<=end_cnt; i++)
  {
    stringstream ss;
    ss <<"./video/image_"<<i<<".png";
    Mat img = imread(ss.str().c_str(), -1); 
    if(!img.data) 
    {
      cout <<"main_video: fail to load image "<<ss.str()<<endl; 
      continue; 
    }else{
      cout <<"main_video: try to add image "<<ss.str()<<endl; 
    }
    
    mpVideo->write(img); 
  }

  cout<<"main_video: finish video, release it!"<<endl; 
  mpVideo->release(); 

  delete mpVideo; 
  return; 

}
