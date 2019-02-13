/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    simple detector, detect object by matching SURF features
*/

#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/opencv.hpp"

using namespace std; 
using namespace cv;
using namespace cv::xfeatures2d;

class CDetector
{
public:
    CDetector();
    CDetector(cv::Mat obj); 
    virtual ~CDetector();
    void init(cv::Mat obj);

    bool detect(cv::Mat img, vector<cv::Point2f>&, bool draw_result = false);
    bool detect(cv::Mat img, vector<cv::KeyPoint>& pts_scene, cv::Mat& des_scene, vector<cv::Point2f>& , bool draw_result = false); 
    
    void match(cv::Mat& des_obj, cv::Mat& des_scene, vector<cv::DMatch>& matches, vector<cv::DMatch>* all_matches = NULL); 

    void extractFeatures(const cv::Mat img, vector<cv::KeyPoint>& kpts, cv::Mat& des); 
    void rejectWithF(vector<cv::Point2f>& obj, vector<cv::Point2f>& scene, vector<uchar>& status);    

    cv::Mat mObjImg;
    std::vector<cv::KeyPoint> mObjPts;
    cv::Mat mObjDes;
    bool mbIntialized;
    int mMatchThresold; 

    cv::FeatureDetector * mpDetector;  
    cv::DescriptorExtractor * mpDescriptor; 
    

};

class CMultiDector
{
public:
	CMultiDector(std::string dir=""); 
	virtual ~CMultiDector(); 
	void init(std::string dir); 
	
    	bool detect(cv::Mat img, vector<cv::Point2f>&, bool draw_result = false);

	std::vector<CDetector*> mvDector; 
	bool mbIntialized; 
};



