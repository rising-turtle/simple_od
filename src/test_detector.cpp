/*
    Mar. 21 2018, He Zhang, hxzhang1@ualr.edu 

    test the function of class detector

*/

#include "detector.h"

using namespace cv;

void readme()
{ 
    cout << " Usage: ./test_detector <img1> <img2>" << std::endl; 
}

int main(int argc, char* argv[])
{
    if( argc != 3 )
    { readme(); return -1; }

    Mat img_object = imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_scene = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_object.data || !img_scene.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
    
    CDetector detector(img_object); 
    vector<Point2f> pts;
    if(detector.detect(img_scene, pts, true))
    {
	cout<<"test_detector: succeed to detect object!"<<endl;
    }else
    {
	cout <<"test_detector: failed to detect object!"<<endl;
    }

    return 0; 
}
