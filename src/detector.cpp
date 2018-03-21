/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    simple detector, detect object by matching SURF features
*/

#include "detector.h"

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
    SurfFeatureDetector detector(minHessian); 
    
    detector.detect(img, kpts); 
    SurfDescriptorExtractor extractor; 
    
    extractor.compute(img, kpts, des); 
    return ; 
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
    
    double h_threshold = 3.0 ; 
    vector<uchar> status; 
    
    Mat H = findHomography( obj, scene, CV_RANSAC, h_threshold, status );

    reduceVector<Point2f>(scene, status);
    out_pts = scene; 

    bool ret_status = false; 
    if(out_pts.size() >= mMatchThresold)
	ret_status = true;

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

