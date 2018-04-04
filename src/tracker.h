/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    a KLT tracker to track features 
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

class CTracker
{
public:
    CTracker();
    virtual ~CTracker(); 

    // do we need to detect new features? 
    // addFeature();
    
    void init(cv::Mat img, vector<cv::Point2f>& pts);
    void uninit(); 
    //
    bool track(cv::Mat img, bool draw_result = false); 

    bool mbInitialized; 

    // middle point of tracked object 
    cv::Point2f meanPoint(); 

    cv::Mat mPreImg;
    cv::Mat mCurImg; 
    vector<cv::Point2f> mPrePts;
    vector<cv::Point2f> mCurPts; 
    
};
