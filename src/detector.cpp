/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    simple detector, detect object by matching SURF features
*/

#include "opencv2/opencv.hpp"
#include "detector.h"
#include <sstream>
#include "global.h"

using namespace cv;

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

CDetector::CDetector(): 
mbIntialized(false),
mMatchThresold(30)
{}
CDetector::CDetector(cv::Mat obj):
mbIntialized(false),
mMatchThresold(30)
{
    init(obj);     
}

CDetector::~CDetector(){}

void CDetector::init(cv::Mat img)
{
    mObjImg = img.clone(); 
    extractFeatures(mObjImg, mObjPts, mObjDes); 
    mbIntialized = true; 
}

void CDetector::extractFeatures(const cv::Mat img, vector<KeyPoint>& kpts, cv::Mat& des)
{
    int minHessian = 400; 
    SurfFeatureDetector detector(minHessian, 3, 7); 
    
    detector.detect(img, kpts); 
    SurfDescriptorExtractor extractor; 
    
    extractor.compute(img, kpts, des); 
    return ; 
}

void CDetector::rejectWithF(vector<Point2f>& obj, vector<Point2f>& scene, vector<uchar>& status)
{
	// realsense rs200 camera model
	double fx = 615.426; 
	double fy = 625.456;
	double cx = 318.837;
	double cy = 240.594; 
	double m_inv_K11 = 1.0/fx; 
	double m_inv_K13 = -cx/fx;
	double m_inv_K22 = 1.0/fy;
	double m_inv_K23 = -cy/fy; 
	
	vector<Point2f> un_obj(obj.size()); 
	vector<Point2f> un_scene(scene.size()); 
	double xu, yu; 
	double FOCAL_LENGTH = 460; 
	double COL = 640; 
	double ROW = 480; 
	status.clear(); 
	for(int i=0; i<obj.size(); i++)
	{
		xu = m_inv_K11 * obj[i].x + m_inv_K13; 
		un_obj[i].x = FOCAL_LENGTH * (xu) + COL / 2.;
		yu = m_inv_K22 * obj[i].y + m_inv_K23; 
		un_obj[i].y = FOCAL_LENGTH * (yu) + ROW / 2.; 

		xu = m_inv_K11 * scene[i].x + m_inv_K13; 
		un_scene[i].x = FOCAL_LENGTH * (xu) + COL / 2.;
		yu = m_inv_K22 * scene[i].y + m_inv_K23; 
		un_scene[i].y = FOCAL_LENGTH * (yu) + ROW / 2.; 
	}

	double F_THRESHOLD = 1.0; 
	findFundamentalMat(un_obj, un_scene, FM_RANSAC, F_THRESHOLD, 0.99, status); 
	reduceVector<Point2f>(scene, status); 
	return ; 
}

bool CDetector::detect(cv::Mat img, vector<cv::KeyPoint>& pts_scene, cv::Mat& des_scene, vector<Point2f>& out_pts, bool draw_result)
{
	// match them 
	FlannBasedMatcher matcher; 
	std::vector<DMatch> matches; 
	matcher.match(mObjDes, des_scene, matches); 

	double max_dist = 0; double min_dist = 10000;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < mObjDes.rows; i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	for( int i = 0; i < mObjDes.rows; i++ )
	{ 
		if( matches[i].distance < 7*min_dist )
		{ good_matches.push_back( matches[i]); }
	}

	// RANSAC + Homograpy 
	vector<Point2f> obj; 
	vector<Point2f> scene;
	for(int i=0; i<good_matches.size(); i++)
	{
		obj.push_back(mObjPts[good_matches[i].queryIdx].pt);
		scene.push_back(pts_scene[good_matches[i].trainIdx].pt);
	}

	// double h_threshold = 3.0 ; 
	vector<uchar> status; 

	// Mat H = findHomography( obj, scene, CV_RANSAC, h_threshold, status );

	rejectWithF(obj, scene, status);
	// double F_THRESHOLD = 1.0; 
	// findFundamentalMat(obj, scene, FM_RANSAC, F_THRESHOLD, 0.99, status); 
	// reduceVector<Point2f>(scene, status);
	out_pts = scene; 

	// cout <<endl<<H<<endl; 
	/* if(!niceHomograpy(H))
	{
		cout <<"detector.cpp: wired H, fail to detect object!"<<endl; 
		return false; 
	}else{
		cout <<"detector.cpp: nice H, succeed to detect object!"<<endl;
	}*/

	bool ret_status = false; 
	if(out_pts.size() >= mMatchThresold)
	{
		cout <<"detector.cpp: RANSAC result scene points: "<<scene.size()<<endl; 
		ret_status = true;
	}else
	{
		cout <<"detector.cpp: failed due to small matched scene points: "<<scene.size()<<endl;
	}

	if(draw_result)
	{
		reduceVector<Point2f>(obj, status);
		reduceVector<DMatch>(good_matches, status); 

		Mat img_matches; 
		drawMatches(mObjImg, mObjPts, img, pts_scene, 
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( mObjImg.cols, 0 );
		obj_corners[2] = cvPoint( mObjImg.cols, mObjImg.rows ); obj_corners[3] = cvPoint( 0, mObjImg.rows );
		std::vector<Point2f> scene_corners(4);

		Mat H = findHomography(obj, scene); 
		perspectiveTransform( obj_corners, scene_corners, H);

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		line( img_matches, scene_corners[0] + Point2f( mObjImg.cols, 0), scene_corners[1] + Point2f( mObjImg.cols, 0), Scalar(0, 255, 0), 4 );
		line( img_matches, scene_corners[1] + Point2f( mObjImg.cols, 0), scene_corners[2] + Point2f( mObjImg.cols, 0), Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[2] + Point2f( mObjImg.cols, 0), scene_corners[3] + Point2f( mObjImg.cols, 0), Scalar( 0, 255, 0), 4 );
		line( img_matches, scene_corners[3] + Point2f( mObjImg.cols, 0), scene_corners[0] + Point2f( mObjImg.cols, 0), Scalar( 0, 255, 0), 4 );

		//-- Show detected matches
		imshow( "Good Matches & Object detection", img_matches );

		waitKey(100);

		// Draw keypoints 
		cv::Mat img_keypoints;
		vector<KeyPoint> key_pts_scene;
		for(int i=0; i<good_matches.size(); i++)
		{
			key_pts_scene.push_back(pts_scene[good_matches[i].trainIdx]);
		}
		drawKeypoints(img, key_pts_scene, img_keypoints); 
		imshow(" Keypoints", img_keypoints); 
		waitKey(0); 
	}
	return ret_status; 
}

bool CDetector::detect(cv::Mat img, vector<Point2f>& out_pts, bool draw_result)
{
    if(mbIntialized == false)
    {
	cout <<"detector.cpp: detecor has not been initialized, meaning no object is indicated!"<<endl;
	return false;
    }
    std::vector<KeyPoint> pts_scene; 
    Mat des_scene; 
    extractFeatures(img, pts_scene, des_scene); 

    return detect(img, pts_scene, des_scene, out_pts, draw_result); 
}


CMultiDector::CMultiDector(string dir)
{
	init(dir); 
}

CMultiDector::~CMultiDector()
{
	for(int i=0; i<mvDector.size(); i++)
	{
		if(mvDector[i] != NULL)
			delete mvDector[i]; 
	}
}



void CMultiDector::init(string dir)
{
	int cnt = 1;
	mvDector.clear(); 
	while(true)
	{
		stringstream ss; 
		ss <<dir<<"/rgb_"<<cnt++<<".png"; 
		cv::Mat img = imread(ss.str().c_str(), -1);
		if(!img.data)
		{
			cout<<"detector.cpp: No img "<<ss.str()<<endl; 
			break; 
		}else{
			// convert from rgb to gray 
			cv::Mat gray;			
		        cvtColor(img, gray, cv::COLOR_RGB2GRAY);
			// histogram filter 
			cv::Mat filtered_gray; 
			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
			clahe->apply(gray, filtered_gray); 
			
			// create detector 
			CDetector* pDec = new CDetector(filtered_gray); 
			mvDector.push_back(pDec); 
			
			// show the image for debug 
			// cv::Mat show_img = drawPoint(img, pDec->mObjPts); 
			// imshow("new detector", show_img); 
			// waitKey(0); 
		}
	}
	mbIntialized = mvDector.size() > 0; 
	if(mbIntialized)
		cout <<"detector.cpp: succeed to create "<<mvDector.size()<<" detectors!"<<endl; 
	return ; 
}	

bool CMultiDector::detect(cv::Mat img, vector<cv::Point2f>& pts, bool draw_result)
{
	if(mbIntialized == false)
	{
		cout<<"mutliDector has not been initailized!"<<endl;
		return false; 	
	}
	cv::Mat gray; 
	if(img.type() == CV_8UC3)
	{
		cvtColor(img, gray, cv::COLOR_RGB2GRAY); 
	}else{
		gray = img.clone(); 
	}

	cv::Mat filtered_gray; 
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	clahe->apply(gray, filtered_gray); 
	// extract features 
	vector<KeyPoint> kpt_scene; 
	Mat des_scene; 
	((CDetector*)0)->extractFeatures(filtered_gray, kpt_scene, des_scene); 	
	
	bool ret = false; 
	for(int i=0; i<mvDector.size(); i++)
	{
		vector<cv::Point2f> out_pts; 
		bool suc = mvDector[i]->detect(filtered_gray, kpt_scene, des_scene, out_pts, draw_result); 
		if(suc)
		{
			cout <<"multiDetector succeed to detect object!"<<endl; 
			pts = out_pts; 
			ret = true;
			break;
		}
	}
	return ret; 
}


