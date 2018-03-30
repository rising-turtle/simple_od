/*
	Mar. 29 2018, He Zhang, hxzhang1@ualr.edu 

	detect object simply based on color 
*/

#include "detector_color.h"

CDetectorColor::CDetectorColor(){}
CDetectorColor::~CDetectorColor(){}

bool CDetectorColor::detect(Mat img, Mat& out_mask)
{
	// 1. color transformation 
	cv::Mat rgb; //  = raw_rgb.clone(); 
	imshow("raw_rgb", img);
	cv::cvtColor(img, rgb, CV_BGR2RGB);
	imshow("rgb", rgb); 

	// 2. denoise 
	cv::Mat rgb_blur;
	cv::GaussianBlur(rgb, rgb_blur, Size(7, 7), 0) ; 

	// 3. rgb to hsv 
	cv::Mat hsv_blur; 
	cv::cvtColor(rgb_blur, hsv_blur, CV_RGB2HSV); 

	// 4. define color range 
	Mat mask1; // hue for red 
	Scalar min_red = Scalar(0, 100, 80); 
	Scalar max_red = Scalar(10, 256, 256); 
	cv::inRange(hsv_blur, min_red, max_red, mask1);
	// cv::imshow("mask1", mask1); 	

	Mat mask2; // brightness  
	Scalar min_red2 = Scalar(170, 100, 80); 
	Scalar max_red2 = Scalar(180, 256, 256); 
	cv::inRange(hsv_blur, min_red2, max_red2, mask2); 
	// cv::imshow("mask2", mask2); 

	Mat mask = mask1 + mask2; 
	// cv::imshow("mask", mask);

	// 5. segmentation
	cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15,15)); 
	cv::Mat mask_closed; 
	morphologyEx(mask, mask_closed, MORPH_CLOSE, kernel); 
	cv::Mat mask_clean;
	morphologyEx(mask_closed, mask_clean, MORPH_OPEN, kernel); 
	cv::imshow("mask_clean", mask_clean); 

	// 6. find the biggest object 
	vector<Point> big_object_contour; 
	Mat mask_object; 
	std::tie(big_object_contour, mask_object) = find_biggest_contour(mask_clean); 
	out_mask = mask_object; 
	cv::imshow("mask_object", mask_object); 
	
	// 7. overlay
	cv::Mat overlay = overlay_mask(mask_object, rgb); 
	cv::imshow("overlay", overlay); 

	// 8. circle the biggest object 
	cv::Mat circled = circle_contour(overlay, big_object_contour); 

	imshow("circled", circled); 
	waitKey(0);  
	return true; 
}

std::tuple<vector<Point>, cv::Mat> 
CDetectorColor::find_biggest_contour(cv::Mat mask_clean)
{
	cv::Mat tmp  = mask_clean.clone(); 
	
	vector<vector<Point> > contours; 
	vector<Vec4i> hierarchy; 
	findContours(tmp, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0)); 
	
	double max_area = -1; 
	double biggest_index = -1; 
	for(int i=0; i<contours.size(); i++)
	{
		double area = contourArea(contours[i]); 
		cout <<"contour "<<i+1<<"'s area is "<<area<<endl;
		if(area > max_area)
		{
			biggest_index = i; 
			max_area = area; 
		}
	}
	// find the largest contour based on 
	// cv::Mat mask = Mat(tmp.rows, tmp.cols, CV_8UC1, 0); 
	cv::Mat mask = Mat::zeros(tmp.size(), CV_8UC1); 
	Scalar color(255); 
	drawContours(mask, contours, biggest_index, color, 2, 8, hierarchy, 0, Point()); 
	return std::make_tuple(contours[biggest_index], mask);
}

cv::Mat CDetectorColor::circle_contour(Mat& image, vector<Point>& big_object_contour)
{
	Mat img_with_ellipse = image.clone();
	RotatedRect ellipse_shape = fitEllipse(Mat(big_object_contour)); 
	Scalar green = Scalar(0, 255, 0);
	cv::ellipse(img_with_ellipse, ellipse_shape, green, 2, CV_AA);
	return img_with_ellipse; 
}

Mat CDetectorColor::overlay_mask(Mat mask, Mat image)
{
	cv::Mat rgb_mask;
	cvtColor(mask, rgb_mask, COLOR_GRAY2RGB); 

	cv::Mat ret; 
	addWeighted(rgb_mask, 0.5, image, 0.5, 0, ret); 
	return ret; 
}


