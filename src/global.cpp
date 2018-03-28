/*
	Mar. 27 2018, He Zhang, hxzhang1@ualr.edu 

	collect functions used globally

*/


#include "global.h"

using namespace std;
using namespace cv; 

// accord to https://github.com/MasteringOpenCV/code/issues/11
bool niceHomograpy(const cv::Mat_<double>& H)
{
	const double det = H(0, 0) * H(1, 1) - H(1, 0) * H(0, 1);
	if (det < 0)
		return false;

	const double N1 = sqrt(H(0, 0) * H(0, 0) + H(1, 0) * H(1, 0));
	if (N1 > 4 || N1 < 0.1)
		return false;

	const double N2 = sqrt(H(0, 1) * H(0, 1) + H(1, 1) * H(1, 1));
	if (N2 > 4 || N2 < 0.1)
		return false;

	const double N3 = sqrt(H(2, 0) * H(2, 0) + H(2, 1) * H(2, 1));
	if (N3 > 0.002)
		return false;

	return true;
}


Mat drawPoint(cv::Mat img, vector<cv::KeyPoint>& pts)
{
    Mat src_copy;
    img.copyTo(src_copy);
    int thickness =2;
    int radius =1;
    for(int i=0; i < pts.size(); i++)
    {
	cv::circle(src_copy, pts[i].pt, radius, Scalar(255, 0, 0), thickness);
    }
   return src_copy;
}

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
