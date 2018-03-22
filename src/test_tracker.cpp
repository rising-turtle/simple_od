/*
    Mar. 21th 2018, He Zhang, hxzhang1@ualr.edu 
    
    test tracker 
*/

#include "tracker.h"
#include <opencv2/legacy/legacy.hpp>

using namespace std; 
using namespace cv;

void webcam_test();
// draw functions
cv::Mat drawFeatPoint(cv::Mat img, vector<cv::Point2f>& pts); 
cv::Mat drawFeatMatch(cv::Mat img, vector<cv::Point2f>& fpts, 
	vector<cv::Point2f>& tpts); 

int main(int argc, char* argv[])
{
    webcam_test(); 
    return 0; 
}

void webcam_test()
{
    cv::VideoCapture input_capture_(0);
    // input_capture_.open(0); 
    if(!input_capture_.isOpened())
    {
	cout <<"test_tracker.cpp: failed to open camera!"<<endl; 
	return ; 
    }   

    CTracker tracker; 

    while( input_capture_.isOpened() && cv::waitKey(30) != 27)
    {
	cv::Mat rgb, frame; 
	input_capture_.read(rgb); 

	cvtColor(rgb, frame, cv::COLOR_RGB2GRAY);

	cv::Mat draw_frame; 
	draw_frame = frame.clone();

	if(tracker.mbInitialized == false)
	{
	    // detect features for the first frame
	    vector<Point2f> n_pts;
	    cv::goodFeaturesToTrack(frame, n_pts, 300, 0.1, 10);
	    tracker.init(frame, n_pts); 
	    draw_frame = drawFeatPoint(frame, n_pts); 
	}else
	{
	    tracker.track(frame); 
	    draw_frame = drawFeatMatch(frame, tracker.mPrePts, tracker.mCurPts);
	}

	cv::imshow("Tracked feature", draw_frame);
    }
}

Mat drawFeatPoint(cv::Mat img, vector<cv::Point2f>& pts)
{
    Mat src_copy;
    img.copyTo(src_copy);
    int thickness =2;
    int radius =1;
    for(int i=0; i < pts.size(); i++)
    {
	cv::circle(src_copy, pts[i], radius, Scalar(255, 0, 0), thickness);
    }
   return src_copy;
}

Mat drawFeatMatch(cv::Mat img, vector<cv::Point2f>& fpts, vector<cv::Point2f>& tpts)
{
    Mat src_copy; 
    img.copyTo(src_copy); 
    for(int i=0; i<fpts.size(); i++)
    {
	circle(src_copy, fpts[i], 1, Scalar(255, 0, 0), 2, 8, 0);
        circle(src_copy, tpts[i], 1, Scalar(0, 0, 255), 2, 8, 0);
        cv::line(src_copy, fpts[i], tpts[i], Scalar(0,255,0) );
    }
    return src_copy; 
}
