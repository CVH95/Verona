// RoVi1 - VISION - FINAL PROJECT

// @author: Carlos Viescas Huerta

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>
#include <rw/rw.hpp>

using namespace std;
using namespace cv;

vector<double> time_vec;

int main(int argc, char* argv[])
{
	// IMAGE LOADING AND INITIAL PARAMETERS.
	Mat src = imread("/home/student/workspace/RoVi1-Final-Project/sequences/marker_color_hard/001.png", 1);

//----------------------------------------------------------------
	
	// Create timer to get runtime 
	long countdown = 5000; // miliseconds
	rw::common::Timer timer = rw::common::Timer(countdown);
	timer.resetAndPause();

//----------------------------------------------------------------

	// PRE-PROCESSING
	
	timer.resetAndResume(); // Start timer
	GaussianBlur(src, src, Size(3,3), 2, 2);

//----------------------------------------------------------------

	// CHANGING COLOR SPACE TO HSV
	
	// Conversion
	Mat hsv;
	cvtColor(src, hsv, CV_BGR2HSV);
	Mat src_hsv = hsv.clone(); // Clone src for later display

	// Split HSV space in planes
	Mat planes[3];  //    planes[0] = Hue   |   planes[1] = Saturation    |    planes[2] = Value
	split(hsv, planes);

	// Thresolding at each individual plane
	
	// Red Circle detection
	Mat redCircle;
	int minHue_red = 3; int maxHue_red= 6;
	inRange(planes[0], minHue_red, maxHue_red, redCircle);
	
	// Blue circles detection 
	Mat blueCircle;
	int minHue_blue = 112; int maxHue_blue = 117;
	inRange(planes[0], minHue_blue, maxHue_blue, blueCircle);

	// Combine both thresholds.
	Mat combined;
	addWeighted(redCircle, 1.0, blueCircle, 1.0, 0.0, planes[0]);
	combined = planes[0].clone();

	Mat thresholded;
	merge(planes, 3, thresholded);

	GaussianBlur(combined, combined, Size(9,9), 2, 2);

	// Morphological operations
	int m_size = 2;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5*m_size + 1, 5*m_size + 1), Point(m_size, m_size));
	morphologyEx(combined, combined, MORPH_OPEN, element, Point(-1, -1), 6);

	/* Thresholding */
	int th = 80;
	threshold(combined, combined, th, 255, 0);

//----------------------------------------------------------------

	// FINDING CONTOURS

	// Canny Edge detector
	int canny_th = 100;
	Canny(combined, combined, canny_th, canny_th, 3);

	// Find contours
	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;

	findContours(combined, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// Approximate contours to circles
	int imax = contours.size();
  	vector<vector<Point> > contours_cc( contours.size() );
  	vector<Point2f>center( contours.size() );
  	vector<float>radius( contours.size() );

  	for( int i = 0; i < imax; i++ )
     	{ 
		approxPolyDP( Mat(contours[i]), contours_cc[i], 3, true );
	       	minEnclosingCircle( (Mat)contours_cc[i], centers[i], radius[i] );
	}
	
	double time_past = timer.getTime();
	timer.resetAndPause();
	time_vec.push_back(time_past);

//---------------------------------------------------------------

	// Draw polygonal contour + bonding rects + circles
  
  	for( int j = 0; j< imax; j++ )
     	{
       		// drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       		// rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
      		circle( thresholded, centers[j], (int)radius[j], Scalar(0,255,255), 4, 8, 0 );
     	}
	

//---------------------------------------------------------------

	cout << "CIRCLE DETECTION AT FRAME:" << endl;
	cout << endl;
	cout << "	>> Runtime: " << time_past  << " [secs]" << endl;
	cout << endl;
	cout << "Centers --> ";
	int h_max = center.size();
	// Store centers
	for(int h=0; h < h_max; h = h+2)
	{
		cout << "(" << center[h] << ", " << center[h+1] << ")     ";
	} // for h

	cout << endl;
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;
	
//------------------------------------------------------------------

	// DISPLAYING
	
	// Window to show combined segmentation
	namedWindow("HSV combined segmentation", WINDOW_NORMAL);
	namedWindow("HSV circle detection", WINDOW_NORMAL);

	// Display circles
	imshow("HSV combined segmentation", combined);
	imshow("HSV circle detection", thresholded);

	waitKey();
	return 0;
}

