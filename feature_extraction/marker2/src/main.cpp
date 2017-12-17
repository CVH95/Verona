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

// GLOBAL VARIABLES

// Mats
Mat src; 
Mat dst;

// To save tracking points
ofstream file;
vector<int> pointsToTrack;

// Containers for performance analysis
vector<int> TP_vec, FP_vec, FN_vec;
vector<double> time_vec;

vector<String> filenames;
String path_easy = "/home/student/workspace/RoVi1-Final-Project/sequences/marker_color/*.png";
String path_hard = "/home/student/workspace/RoVi1-Final-Project/sequences/marker_color_hard/*.png";

# define M_PI           3.14159265358979323846




//===================================================================================================================================================



								// SET OF FUNCTIONS



// Personal notes
void theoryScript()
{
	ofstream fs;
	fs.open("/home/student/workspace/RoVi1-Final-Project/feature_extraction/marker2/genfiles/theory_script.txt");
	fs << endl;
	fs << "=======================================================================" << endl;
	fs << "			FEATURE EXTRACTION - COLOR MARKER" << endl;
	fs << "=======================================================================" << endl;
	fs << endl;
	fs << "  APPROACH:" << endl;
	fs << endl;
	fs << "		>> 1. Pre-processing stage to get the image ready for detection." << endl;
	fs << "		>> 2. Color segmentation in:" << endl;
	fs << "						+ BGR space." << endl;
	fs << "						+ HSV space -----> BETTER SOLUTION" << endl;
	fs << "		>> 3. Use or oopening transformation to produce a binary image with the marker circles and background ." << endl;
	fs << "		>> 4. Apply an edge detector and find resulting contours. " << endl;
	fs << "		>> 5. Enclose each contour in a surrounding circle and store its center as tracking point. " << endl; 
	fs << endl;
	fs.close();
}


//------------------------------------------------------------------------------------------------------------------------------


// Function to get individual frame performance-statistics
void frameStats(int i, int total)
{
	// CONFUSSION MATRIX PARAMETERS
	
	double progress = (double) (i+1) / (total);

	// Variables
	int TP, FP, FN;
	// accumulators: vector<int> TP_vec, FP_vec, FN_vec;

	// User interface
	cout << endl;
	cout << "  FRAME #" << i+1 << " EVALUATION" << endl;
	cout << endl;
	cout << "(" << i+1 << " out of " << total << " | " << progress*100 << "%)" << endl;
	cout << endl;
	cout << endl;
	cout << "	>> TRUE circles DETECTED in this frame: TP = ";
	cin >> TP;
	cout << "	>> TRUE circles MISSED in this frame: FN = "; 
	cin >> FN;
	cout << "	>> FALSE circles DETECTED in this frame: FP = "; 
	cin >> FP;
	cout << endl;
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;
	
	TP_vec.push_back(TP);
	FN_vec.push_back(FN);
	FP_vec.push_back(FP);

}


//----------------------------------------------------------------------------------------------------------------------------------


// Detector function
void detector(Mat src, int examp, int total)
{	
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

	imshow("combined", combined);

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

	// CIRCLE DETECTION USING CONTOURS IN BINARY IMAGES

	// FINDING CONTOURS

	// Canny Edge detector
	int canny_th = 100;
	Canny(combined, combined, canny_th, canny_th, 3);

	// Find contours
	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;

	findContours(combined, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

//----------------------------------------------------------------

	// Approximate contours to circles
	int imax = contours.size();
  	vector<vector<Point> > contours_cc(contours.size());
  	vector<Point2f>centers(contours.size());
  	vector<float>radius(contours.size());

  	for( int i = 0; i < imax; i++ )
     	{ 
		approxPolyDP( Mat(contours[i]), contours_cc[i], 3, true );
	       	minEnclosingCircle( (Mat)contours_cc[i], centers[i], radius[i] );
	}
	
	double time_past = timer.getTime();
	timer.resetAndPause();
	time_vec.push_back(time_past);

//------------------------------------------------------------------

	// DRAWING DETECTED CIRCLES

	for( int j = 0; j< imax; j++ )
     	{
		// Storing centers 
		int cx = round(centers[j].x);
		pointsToTrack.push_back(cx);
		int cy = round(centers[j].y);
		pointsToTrack.push_back(cy);
		
		// Drawing 
      		circle( thresholded, centers[j], (int)radius[j], Scalar(0,255,255), 4, 8, 0 );
     	}
 

	cout << "CIRCLE DETECTION AT FRAME #" << examp+1 << ":" << endl;
	cout << endl;
	cout << "	>> Circles found: " << imax/2 << endl;
	cout << endl;
	cout << "	>> Runtime: " << time_past  << " [secs]" << endl;
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

	int sleep_time = 1000000;
	waitKey(sleep_time);

//------------------------------------------------------------------

	// FRAME STATS
	frameStats(examp, total);
}


//----------------------------------------------------------------------------------------------------------------------------


// Function to return global statistics
void globalStats(int sequence_size, string path)
{
	// Global TP, FP and FN:
	int TP_f, FP_f, FN_f;
	TP_f = std::accumulate(TP_vec.begin(), TP_vec.end(), 0);
	FP_f = std::accumulate(FP_vec.begin(), FP_vec.end(), 0);
	FN_f = std::accumulate(FN_vec.begin(), FN_vec.end(), 0);
	
	// Confusion Matrix
	int confMat[2][2];
	confMat[0][0] = TP_f;
	confMat[0][1] = FP_f;
	confMat[1][0] = FN_f; 
	confMat[1][1] = 0;

	// Precision and recall
	double precision = (double) (TP_f) / (TP_f + FP_f);
	double recall = (double) (TP_f) / (TP_f + FN_f);

	// Per Image analysis (FPPI)
	double miss_rate = (double) 1 - recall;
	double FPPI = (double) (FP_f) / (sequence_size); 

	// Runtime
	double time_sum = std::accumulate(time_vec.begin(), time_vec.end(), 0.0);
	// cout << "time_sum   " << time_sum;
	double avg_time = (double) (time_sum) / (sequence_size);

	// Display
	cout << endl;
	cout << "  OVERALL STATISTICS" << endl;
	cout << endl;
	cout << "(Tested over the entire sequence: " << path << ")" << endl;
	cout << endl;
	cout << "	>> Total number of frames: " << sequence_size;
	cout << endl;
	cout << "	>> Confusion Matrix:" << endl;
	cout << endl;
	cout << "                             True class" << endl;
	cout << endl;
	cout << "                        TP = " << confMat[0][0] << "      FP = " << confMat[0][1] << endl; 
	cout << "Hypothesized class" << endl;
	cout << "			FN = " << confMat[1][0] << "      TN = " << confMat[1][1] << endl; 
	cout << endl;
	cout << endl;
	cout << "  RATES:" << endl;
	cout << endl;
	cout << "	>> Recall (or TP rate): " << recall*100 << "%" << endl;
	cout << "	>> Precision: " << precision*100 << "%" << endl;
	cout << "	>> False positives per frame (FPPI): " << FPPI << endl;
	cout << "	>> Miss-rate: " << miss_rate*100 << "%" << endl;
	cout << endl;
	cout << "  RUNTIME:" << endl;
	cout << endl;
	cout << "	>> Average runtime: " << avg_time << " [secs]" << endl;
	cout << endl;
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;
	
} 




//===================================================================================================================================================



									//MAIN



int main(int argc, char* argv[])
{
	// IMAGE LOADING AND INITIAL PARAMETERS.
	
	cout << endl;
	cout << "		---------------------------------------------" << endl; 
	cout << "		    OBJECT RECOGNITION FOR IMAGE TRACKING" << endl;
	cout << "		---------------------------------------------" << endl; 
	cout << endl; cout << endl; cout << endl;
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;

//----------------------------------------------------------------------------------
	
	// Loading Sequence	
	glob(path_hard, filenames);
	int imgNum;
	int total = filenames.size();

	cout << "DETECTION BASED ON COLOR SEGMENTATION AND CIRCLE DETECTION" << endl;
	cout << endl;	
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;

	file.open("/home/student/workspace/RoVi1-Final-Project/feature_extraction/marker2/genfiles/Points_to_Track_hard.txt");

	for(imgNum = 0; imgNum < total; imgNum++)
	{
		src = imread(filenames[imgNum], 1);
		if (src.empty()) 
		{
        		std::cout << "Input image not found" << endl;
        		return 1;
    		} // if 

		cout << endl;
		cout << "	>> Image #" << imgNum+1 << " loaded correctly." << endl;
		cout << "	>> Type: " << src.type() << endl; 
		cout << endl;

	//-----------------------------------------------------------
		
		// APPLYING DETECTOR TO SEQUENCE

		detector(src, imgNum, total);
	
	//-----------------------------------------------------------
		
		// SAVING DETECTED CENTERS TO FILE

		file << endl;
		file << "Frame #" << imgNum+1 << endl;
		file << "Centers --> ";
		int h_max = pointsToTrack.size();
		
		// Store centers
		for(int h=0; h < h_max; h = h+2)
		{
			file << "(" << pointsToTrack[h] << ", " << pointsToTrack[h+1] << ")     ";
		} // for h

		file << endl;

		// Empty center accumulator for a new frame
		pointsToTrack.clear();

	} // for ingNum

	
//-------------------------------------------------------------------------------

	// OVERALL PERFORMANCE STATISTICS

	globalStats(total, path_hard);
	
//-------------------------------------------------------------------------------

	// FINISHING

	theoryScript();

	// Display message in console to see that everything was succesfully done.
	cout << "Program executed succesfully" << endl;
	cout << endl;
	cout << "		---------------------------------------------" << endl; 
	cout << "		 Carlos Viescas Huerta | SDU Robotics | RoVi1" << endl;
	cout << "		---------------------------------------------" << endl; 
	cout << endl;
	
	waitKey();
	return 0;
}
