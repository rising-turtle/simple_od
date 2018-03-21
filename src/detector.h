/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    simple detector, detect object by matching SURF features
*/

#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace std; 

class CDetector
{
public:
    CDetector();
    CDetector(cv::Mat obj); 
    virtual ~CDetector();
    void init(cv::Mat obj);

    bool detect(cv::Mat img, vector<cv::Point2f>&, bool draw_result = false);

    void extractFeatures(const cv::Mat img, vector<cv::KeyPoint>& kpts, cv::Mat& des); 
    
    cv::Mat mObjImg;
    std::vector<cv::KeyPoint> mObjPts;
    cv::Mat mObjDes;
    bool mbIntialized;
    int mMatchThresold; 
};

