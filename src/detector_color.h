/*
	Mar. 29 2018, He Zhang, hxzhang1@ualr.edu 

	detect object simply based on color 
*/

#pragma once

#include "opencv2/opencv.hpp"
#include <sstream>
#include <tuple>
#include <vector>

using namespace cv; 
using namespace std; 

class CDetectorColor
{
public:
	CDetectorColor();
	virtual ~CDetectorColor(); 
	Mat overlay_mask(Mat mask, Mat image); 
	std::tuple<vector<Point>, cv::Mat, bool> find_biggest_contour(cv::Mat mask_clean);
	cv::Mat rect_contour(Mat& overlay, vector<Point>& big_object_contour); 
	cv::Mat circle_contour(Mat& overlay, vector<Point>& big_object_contour); 
	bool detect(Mat img, Mat& mask); 
};

