/*  
 *  Apr. 3 2018, He Zhang, hxzhang1@ualr.edu 
 *
 *  main process of detecting and tracking object through glove
 *
 * */
#include <librealsense/rs.hpp>
#include "global.h"
#include <cstdio>
#include "opencv2/opencv.hpp"
#include <sstream>
#include <string>
#include "tracker.h"
#include "detector_color.h"
#include "glove_control.h"

using namespace cv; 
using namespace std; 

int detectObjRS(); 

void moveGlove(CGloveControl& g, Point2f& pt, cv::Mat& img);

int main(int argc, char* argv[])
{
	int ret = detectObjRS(); 
	// int ret = detectObjImg(argc, argv); 
	// cv::Mat tmp = Mat::zeros(500, 500, CV_8UC3); 
	// cv::rectangle(tmp, Point(0, 100), Point(300, 300), Scalar(2, 255, 255), -1, 8);
	// imshow("tmp", tmp);
	// waitKey(0);
	return 1; // ret; 
}

int detectObjRS()
{
	rs::context ctx; 
	printf("There are %d connected RealSense devices. \n", ctx.get_device_count()); 
	if(ctx.get_device_count() == 0) return EXIT_FAILURE; 
	
	rs::device * dev = ctx.get_device(0); 
	printf("\nUsing device 0, an %s\n", dev->get_name());
	printf("    Serial number: %s\n", dev->get_serial());
	printf("    Firmware version: %s\n", dev->get_firmware_version());

	dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60); 
	dev->start(); 
	
	char key = 1; 

	CTracker tracker;
	CDetectorColor detector;  
	bool draw_result = false; 
	bool detected = false; 
	Mat show_img; 

        // show glove 
        CGloveControl glove; 
        cout <<"main_glove_track.cpp: start capture stream!"<<endl; 
        
        int image_cnt = 1;

	while(key != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_img(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		cv::cvtColor(raw_img, raw_img, CV_BGR2RGB);
		
		if(!detected) // try to detect 
		{
			Mat mask; 
			vector<cv::Point2f> pts; 
			if(detector.detect(raw_img, mask))
			{	
				cv::Mat gray;
				cvtColor(raw_img, gray, CV_BGR2GRAY);
				// extract and show feature
				// pts.clear(); 
				cv::goodFeaturesToTrack(gray, pts, 10000, 0.05, 5, mask); // 0.1
				if(pts.size() > 5)
				{
					show_img = drawPoint(raw_img, pts);
					tracker.init(gray, pts); 
					detected = true; 
				}else{
					cv::Mat rgb_mask;
					cvtColor(mask, rgb_mask, COLOR_GRAY2RGB);
					addWeighted(rgb_mask, 0.5, raw_img, 0.9, 0, show_img);
				}
			}else
			{
				show_img = raw_img; 
			}
		}else{ // try to track 
			cv::Mat gray; 
			cv::cvtColor(raw_img, gray, CV_BGR2GRAY); 		
			// track it 
			if(tracker.track(gray))
			{
				show_img = drawMatch(raw_img, tracker.mPrePts, tracker.mCurPts);
                                
                                // use glove to guide track
                                Point2f obj_pt = tracker.meanPoint(); 
                                if(obj_pt.x >0 && obj_pt.x < show_img.cols && obj_pt.y > 0 && obj_pt.y < show_img.rows)
                                {
                                  moveGlove(glove, obj_pt, show_img); 
                                }else{
                                  cout <<"main_glove_track.cpp: error: obj_pt: x "<<obj_pt.x<<" y "<<obj_pt.y<<endl; 
                                }
			        
                        }else
			{	
				cout <<"main_od_color.cpp: track fails only "<<tracker.mCurPts.size()<<" are tracked! Switch to detector!"<<endl; 
				detected = false; 
				tracker.uninit(); 
				show_img = raw_img; 
			}
		}
		cv::imshow("show_img", show_img);
                stringstream ss;
                ss<<"./video/image_"<<image_cnt++<<".png"; 
                cv::imwrite(ss.str().c_str(), show_img); 
		key = cv::waitKey(30); 
	}
	return 1; 
}

void moveGlove(CGloveControl& g, Point2f& pt, cv::Mat& show_img)
{
  Point2f central_pt(show_img.cols/2, show_img.rows/2); 
  // within this rectangle do not have to move glove 
  double l = 200; 
  Point2f lu(central_pt.x - l/2, central_pt.y - l/2); 
  Point2f rb(central_pt.x + l/2, central_pt.y + l/2); 
  cv::Rect rect(lu, rb);
  // draw this rectangle 
  Scalar green = Scalar(0, 255, 0); 
  cv::rectangle(show_img, rect, green, 2, CV_AA);

  // within the rectangle 
  if(fabs(pt.x - central_pt.x) < l/2 && fabs(pt.y - central_pt.y) < l/2)
  {
    cout <<"main_glove_track.cpp: traget within rect, go for it!"<<endl; 
  }else
  {
    if(pt.x > central_pt.x + l/2) // object locate at right 
    { 
      // move right 
      double ratio = (pt.x - central_pt.x)/(show_img.cols/2.); 
      cout <<"main_glove_track.cpp: move right, ratio = "<<ratio<<endl; 
      g.moveRight(ratio);
    }else if(pt.x < central_pt.x - l/2) // object locate at left
    {
      // move left 
      double ratio = (central_pt.x - pt.x)/(show_img.cols/2.); 
      cout <<"main_glove_track.cpp: move left, ratio = "<<ratio<<endl; 
      g.moveLeft(ratio); 
    }
    
    if(pt.y > central_pt.y + l/2) // object locate at backward 
    {
      // move backward 
      double ratio = (pt.y - central_pt.y)/(show_img.rows/2.); 
      g.moveBack(ratio); 
      cout <<"main_glove_track.cpp: move backward, rato = "<<ratio<<endl; 
    }else if(pt.y < central_pt.y - l/2) // object locate at the forward
    {
      // move forward 
      double ratio = (central_pt.y - pt.y)/(show_img.rows/2.); 
      g.moveFront(ratio); 
      cout <<"main_glove_track.cpp: move forward, ratio = "<<ratio<<endl; 
    }
  }
  // draw an arrow from central point to the traget point 
  Scalar yellow = Scalar(0, 255, 255); 
  // cv::arrowedLine(show_img, central_pt, pt, yellow, 2); 
  cv::line(show_img, central_pt, pt, yellow, 2); 

  return ; 
}

