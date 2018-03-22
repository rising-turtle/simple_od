/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    a KLT tracker to track features 
*/

#include "tracker.h"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/opencv.hpp>

using namespace cv; 

CTracker::CTracker():
mbInitialized(false){}
CTracker::~CTracker(){}

namespace{

template<typename T>
void reduceVector(std::vector<T>& v, vector<uchar>& s)
{
    int j = 0;
    int i = 0;
    for(i=0; i<s.size(); i++)
	if(s[i])
	    v[j++] = v[i];
    v.resize(j);
    return; 
}

}

void CTracker::init(cv::Mat img, vector<Point2f>& pts)
{
    mPreImg = img.clone(); 
    mPrePts = pts;
    mCurImg = mPreImg; 
    mCurPts = mPrePts;
    mbInitialized = true; 
}

bool CTracker::track(cv::Mat img, bool draw_result)
{
    if(mbInitialized == false)
    {
	cout<<"tracker: not initialzied "<<endl;
	return false; 
    }
    vector<uchar> status;
    vector<float> err;
    
    mPreImg = mCurImg;
    mPrePts = mCurPts;
    mCurImg = img; 
    mCurPts.clear(); 
    cv::calcOpticalFlowPyrLK(mPreImg, mCurImg, mPrePts, mCurPts, status, err, cv::Size(21, 21), 3); 
    reduceVector<Point2f>(mPrePts, status); 
    reduceVector<Point2f>(mCurPts, status); 
    // mPrePts = mCurPts; 
    // mPreImg = mCurImg; 
    return true; 
}
