/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    simple detector, detect object by matching SURF features
*/

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

CDetector::~CDetector()
{
	delete mpDetector;
	delete mpDescriptor; 
}

void CDetector::init(cv::Mat img)
{
	mObjImg = img.clone(); 
	mbIntialized = true; 
	// extract keypoints 
	// mpDetector = new cv::SURF(600.); 
	// mpDetector = new cv::SIFT(); 
	// mpDetector = new cv::FastFeatureDetector(); 
	// mpDetector = new cv::DenseFeatureDetector(); 
	// mpDetector = new cv::GFTTDetector(); 
	// mpDetector = new cv::MSER(); 
	mpDetector = new cv::ORB(); 
	// mpDetector = new cv::StarFeatureDetector(); 
	// mpDetector = new cv::BRISK(); 

	// extract descriptors
	// mpDescriptor = new cv::SURF(600.); 
	// mpDescriptor = new cv::SIFT();      
	mpDescriptor = new cv::FREAK();      
	// mpDescriptor = new cv::ORB();      
	// mpDescriptor = new cv::BRISK();      

	// mpDescriptor = new cv::BriefDescriptorExtractor();      

	extractFeatures(mObjImg, mObjPts, mObjDes); 
  
}

void CDetector::extractFeatures(const cv::Mat img, vector<KeyPoint>& kpts, cv::Mat& des)
{
    // SURF
    // int minHessian = 400; 
    // SurfFeatureDetector detector(minHessian, 3, 7); 
    
    // detector.detect(img, kpts);  
	mpDetector->detect(img, kpts);
    // SurfDescriptorExtractor extractor; 
    
    // extractor.compute(img, kpts, des); 
	mpDescriptor->compute(img, kpts, des); 	
	// cout <<"detector.cpp: kpts have "<<kpts.size()<<" des has "<<des.rows<<endl; 

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

void CDetector::match(cv::Mat& des_obj, cv::Mat& des_scene, vector<DMatch>& out_matches)
{
	// match them 
	// FlannBasedMatcher matcher; 
	// std::vector<DMatch> matches; 
	// matcher.match(des_f, des_t, matches); 
	
	int k = 2;

	cv::Mat results;
	cv::Mat dists;

	vector<vector<DMatch>> matches; 
	bool useBFMatcher = false; // SET TO TRUE TO USE BRUTE FORCE MATCHER
	if(des_obj.type()==CV_8U)
	{
		// Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
		// printf("Binary descriptors detected...\n");
		if(useBFMatcher)
		{
			cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
			matcher.knnMatch(des_obj, des_scene, matches, k);
		}
		else
		{
			// Create Flann LSH index
			cv::flann::Index flannIndex(des_scene, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
			// printf("Time creating FLANN LSH index = %d ms\n", time.restart());

			// search (nearest neighbor)
			flannIndex.knnSearch(des_obj, results, dists, k, cv::flann::SearchParams() );
		}
	}
	else
	{
		// assume it is CV_32F
		// printf("Float descriptors detected...\n");
		if(useBFMatcher)
		{
			cv::BFMatcher matcher(cv::NORM_L2);
			matcher.knnMatch(des_obj, des_scene, matches, k);
		}
		else
		{
			// Create Flann KDTree index
			cv::flann::Index flannIndex(des_scene, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
			// printf("Time creating FLANN KDTree index = %d ms\n", time.restart());

			// search (nearest neighbor)
			flannIndex.knnSearch(des_obj, results, dists, k, cv::flann::SearchParams() );
		}
	}
	// printf("Time nearest neighbor search = %d ms\n", time.restart());

	// Conversion to CV_32F if needed
	if(dists.type() == CV_32S)
	{
		cv::Mat temp;
		dists.convertTo(temp, CV_32F);
		dists = temp;
	}

	// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
	float nndrRatio = 0.8f;
	std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
	std::vector<int> indexes_1, indexes_2; // Used for homography
	std::vector<uchar> outlier_mask;  // Used for homography
	// Check if this descriptor matches with those of the objects
	if(!useBFMatcher)
	{
		for(int i=0; i<des_obj.rows; ++i)
		{
			// Apply NNDR
			//printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
			if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
					dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
			{
				cv::DMatch m; 
				m.queryIdx = i; 
				m.trainIdx = results.at<int>(i, 0);
				out_matches.push_back(m); 
			}
		}
	}
	else
	{
		for(unsigned int i=0; i<matches.size(); ++i)
		{
			// Apply NNDR
			//printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
			if(matches.at(i).size() == 2 &&
					matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
			{

				out_matches.push_back(matches[i][0]); 
			}
		}
	}

	

	return ; 	
}

bool CDetector::detect(cv::Mat img, vector<cv::KeyPoint>& pts_scene, cv::Mat& des_scene, vector<Point2f>& out_pts, bool draw_result)
{
	
	// step 1 - feature match using descriptor  
	const int min_kp_num = mMatchThresold; 
	if(pts_scene.size() < mMatchThresold)
	{
		// cout <<"detector.cpp: extract "<<pts_scene.size()<<" feautes < "<<mMatchThresold<<endl;
		return false; 
	}

	vector<DMatch> good_matches; 
	match(mObjDes, des_scene, good_matches); 

	// step 2 - RANSAC + Homograpy 
	vector<Point2f> obj; 
	vector<Point2f> scene;
	for(int i=0; i<good_matches.size(); i++)
	{
		obj.push_back(mObjPts[good_matches[i].queryIdx].pt);
		scene.push_back(pts_scene[good_matches[i].trainIdx].pt);
	}

	double h_threshold = 1.0 ; 
	vector<uchar> status; 

	// Mat H = findHomography( obj, scene, CV_RANSAC, h_threshold, status );

	if(scene.size() >= mMatchThresold)
		rejectWithF(obj, scene, status);
	else{
		cout<<"detector.cpp: matched features "<<scene.size()<<" less than threshold "<<mMatchThresold<<endl;
		return false;
	}
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
		cout <<"detector.cpp: RANSAC result scene points: "<<scene.size()<<" from "<<status.size()<<endl; 
		ret_status = true;
	}else
	{
		cout <<"detector.cpp: after rejectF small matched scene points: "<<scene.size()<<endl;
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
			cv::Mat filtered_gray = gray; 
			// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
			// clahe->apply(gray, filtered_gray); 
			
			// create detector 
			CDetector* pDec = new CDetector(filtered_gray); 
			mvDector.push_back(pDec); 
			cout <<"detector.cpp: new detector has "<<pDec->mObjPts.size()<<" pts!"<<endl;
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

	cv::Mat filtered_gray = gray; 
	// cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	// clahe->apply(gray, filtered_gray); 
	// extract features 
	vector<KeyPoint> kpt_scene; 
	Mat des_scene; 
	mvDector[0]->extractFeatures(filtered_gray, kpt_scene, des_scene); 	
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


