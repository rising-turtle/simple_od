/*
	Mar. 27 2018, He Zhang, hxzhang1@ualr.edu 

	collect functions used globally

*/


#include "global.h"

using namespace std;
using namespace cv; 

Mat drawPoint(cv::Mat img, vector<cv::Point2f>& pts)
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

Mat drawMatch(cv::Mat img, vector<cv::Point2f>& fpts, vector<cv::Point2f>& tpts)
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
