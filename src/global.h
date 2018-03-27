/*
	Mar. 27 2018, He Zhang, hxzhang1@ualr.edu 

	collect functions used globally

*/

#pragma once

#include <vector>
#include "opencv2/opencv.hpp"

extern cv::Mat drawPoint(cv::Mat img, std::vector<cv::Point2f>& pts); 
extern cv::Mat drawMatch(cv::Mat img, std::vector<cv::Point2f>& fpts, 
			std::vector<cv::Point2f>& tpts); 

