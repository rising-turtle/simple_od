/*
	Mar. 29 2018, He Zhang, hxzhang1@ualr.edu

	simply extract contour based on color range, then detect and track features in this contour area

*/

#include <librealsense/rs.hpp>
#include "global.h"
#include <cstdio>
#include "opencv2/opencv.hpp"
#include <sstream>
#include "tracker.h"
#include "detector_color.h"

using namespace cv; 
using namespace std; 

int detectObjRS(); 
int detectObjImg(int argc, char* argv[]); 

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

int detectObjImg(int argc, char* argv[])
{
	CDetectorColor detector; 
	for(int i=1; i<argc; i++)
	{
		Mat raw_img = imread(argv[i], -1);
		if(!raw_img.data)
		{
			cout <<"main_od_color: fail to load image "<<argv[i]<<endl; 
			continue; 
		}
		cv::Mat mask; 
		if(detector.detect(raw_img, mask))
		{
			cv::Mat gray;
			cvtColor(raw_img, gray, CV_BGR2GRAY);
			vector<Point2f> pts;  
			// extract and show feature
			cv::goodFeaturesToTrack(gray, pts, 10000, 0.1, 5, mask); 
			cv::Mat tmp = drawPoint(raw_img, pts);
			cout <<"main_od_color.cpp: succeed to detect object!"<<endl; 
			imshow("detected ", tmp); 
		}else
		{
			// 
			cout <<"main_od_color.cpp: failed to detect object!"<<endl; 
			imshow("raw_img", raw_img); 
			
		}
		cv::waitKey(0); 
		
	}
	return 1;
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
				cv::goodFeaturesToTrack(gray, pts, 10000, 0.1, 5, mask); 
				if(pts.size() > 40)
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
			}else
			{	
				cout <<"main_od_color.cpp: track fails only "<<tracker.mCurPts.size()<<" are tracked! Switch to detector!"<<endl; 
				detected = false; 
				tracker.uninit(); 
				show_img = raw_img; 
			}
		}
		cv::imshow("show_img", show_img);
		key = cv::waitKey(30); 
	}
	return 1; 
}


