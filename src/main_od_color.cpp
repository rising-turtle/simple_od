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
	// int ret = detectObjRS(); 
	int ret = detectObjImg(argc, argv); 
	return ret; 
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
		detector.detect(raw_img, mask); 
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
	bool draw_result = false; 
	bool detected = false; 
	while(key != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_rgb(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		vector<cv::Point2f> pts; 
		
	}
	return 1; 
}


