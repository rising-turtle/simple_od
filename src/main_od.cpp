/*
	Mar. 27 2018, He Zhang, hxzhang1@ualr.edu
	
	track object using rs200's data 

*/

#include <librealsense/rs.hpp>
#include "global.h"
#include <cstdio>
#include <GLFW/glfw3.h>
#include "opencv2/opencv.hpp"
#include <sstream>
#include <string>
#include "detector.h"

using namespace cv; 
using namespace std; 

string g_obj_dir = "../imgs/ob_template";

void detectObj(rs::device* dev); 

int main(int argc, char* argv[])
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
	
	detectObj(dev); 
	return EXIT_SUCCESS; 

}


void detectObj(rs::device* dev)
{
	char key = 1; 

	// get detector 
	CMultiDector detector(g_obj_dir); 	

	bool draw_result = false; 
	while(key != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_rgb(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		cv::Mat rgb; //  = raw_rgb.clone(); 
		cv::cvtColor(raw_rgb, rgb, CV_BGR2RGB);
		
		vector<cv::Point2f> pts; 
		bool suc = detector.detect(raw_rgb, pts, draw_result); 
		if(suc)
		{
			cout <<"main_od: succeed to detect object "<<endl; 
			cv::Mat labeled = drawPoint(rgb, pts); 
			cv::imshow("tracked!", labeled); 
		}else
		{
			cv::imshow("rgb", rgb);
		}
		key = cv::waitKey(30); 

	}
}


