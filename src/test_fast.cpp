/*
	Mar. 28 2018, He Zhang, hxzhang1@ualr.edu 

	Extract fast features from the rs200 data

*/

#include <vector>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fast/fast.h>
#include <librealsense/rs.hpp>
#include "global.h"

using namespace std; 

void detect_and_show_fast(rs::device* dev); 

void detect_and_show_fast_opencv(rs::device* dev); 


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
	
	// detect_and_show_fast(dev); 
	detect_and_show_fast_opencv(dev); 

	return EXIT_SUCCESS; 

}

void detect_and_show_fast_opencv(rs::device* dev)
{
	char key = 1;
	cv::FeatureDetector * detector = new cv::FastFeatureDetector(); 
	while(key != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_rgb(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		cv::Mat rgb; //  = raw_rgb.clone(); 
		cv::cvtColor(raw_rgb, rgb, CV_BGR2RGB);
		
		cv::Mat gray;
		cv::cvtColor(rgb, gray, CV_RGB2GRAY); 
		// vector<cv::Point2f> pts(nm_index.size()); 
	
		vector<cv::KeyPoint> pts; 
		detector->detect(gray, pts); 

		cout <<" test_fast: detect fast points "<<pts.size()<<endl; 
		cv::Mat labeled = drawPoint(rgb, pts); 
		cv::imshow("fast detection", labeled); 

		key = cv::waitKey(30); 
	}


}


void detect_and_show_fast(rs::device * dev)
{
	char key = 1; 

	// get detector 
	// CMultiDector detector(g_obj_dir); 	
	
	bool draw_result = false; 
	while(key != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_rgb(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		cv::Mat rgb; //  = raw_rgb.clone(); 
		cv::cvtColor(raw_rgb, rgb, CV_BGR2RGB);
		
		cv::Mat gray;
		cv::cvtColor(rgb, gray, CV_RGB2GRAY); 
		std::vector<fast::fast_xy> corners;
		fast::fast_corner_detect_10_sse2((fast::fast_byte*)(gray.data), gray.cols, gray.rows, gray.cols, 25 /*75*/, corners); 
		
		vector<int> score;
		vector<int> nm_index; 
		fast::fast_corner_score_10((fast::fast_byte*)(gray.data), gray.cols, corners, 25, score);
		fast::fast_nonmax_3x3(corners, score, nm_index); 
		vector<cv::Point2f> pts(nm_index.size()); 
		// bool suc = detector.detect(raw_rgb, pts, draw_result); 
		for(int i=0; i<pts.size(); i++)
		{
			pts[i].x = corners[nm_index[i]].x; 
			pts[i].y = corners[nm_index[i]].y; 
		}		
		
		cout <<" test_fast: detect fast points "<<pts.size()<<endl; 
		cv::Mat labeled = drawPoint(rgb, pts); 
		cv::imshow("fast detection", labeled); 

		key = cv::waitKey(30); 
	}

}



