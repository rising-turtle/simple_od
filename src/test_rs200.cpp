/*
	Mar. 27 2018, He Zhang, hxzhang1@ualr.edu
	
	show rs200's image and save them on disk

*/

#include <librealsense/rs.hpp>
#include <cstdio>
#include <GLFW/glfw3.h>
#include "opencv2/opencv.hpp"

using namespace cv; 
using namespace std; 

void showGLFW(rs::device* dev); 
void showOpenCV(rs::device* dev); 

int main(int argc, char* argv[])
{
	rs::context ctx; 
	printf("There are %d connected RealSense devices. \n", ctx.get_device_count()); 
	if(ctx.get_device_count() == 0) return EXIT_FAILURE; 
	
	rs::device * dev = ctx.get_device(0); 
	printf("\nUsing device 0, an %s\n", dev->get_name());
	printf("    Serial number: %s\n", dev->get_serial());
	printf("    Firmware version: %s\n", dev->get_firmware_version());

	dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60); 
	dev->start(); 
	
	showOpenCV(dev); 
	return EXIT_SUCCESS; 

}


void showOpenCV(rs::device* dev)
{
	while(cv::waitKey(100) != 27)
	{
		dev->wait_for_frames(); 
		cv::Mat raw_rgb(480, 640, CV_8UC3, dev->get_frame_data(rs::stream::color)); 
		cv::Mat rgb; //  = raw_rgb.clone(); 
		cv::cvtColor(raw_rgb, rgb, CV_BGR2RGB);
		cv::imshow("rgb", rgb);
	}
}


void showGLFW(rs::device* dev)
{
	glfwInit();
	GLFWwindow * win = glfwCreateWindow(640, 480, "show rs200 image", nullptr, nullptr); 
	glfwMakeContextCurrent(win); 

	while(!glfwWindowShouldClose(win))
	{	
		// 
		glfwPollEvents(); 
		dev->wait_for_frames(); 

		glClear(GL_COLOR_BUFFER_BIT); 
		glPixelZoom(1, -1); 

		glRasterPos2f(-1, 1);
		glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::color));

		glfwSwapBuffers(win);
	}
	return ; 
}



