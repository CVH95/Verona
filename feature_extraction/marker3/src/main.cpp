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
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

// GLOBAL VARIABLES
Mat temp, src;

// To save tracking points
ofstream file;
//vector<int> pointsToTrack;

// Containers for performance analysis
vector<int> PF_vec, FP_vec, MF_vec;
vector<double> time_vec;

vector<String> filenames;
String path_easy = "/home/student/workspace/RoVi1-Final-Project/sequences/marker_corny/*.png";
String path_hard = "/home/student/workspace/RoVi1-Final-Project/sequences/marker_corny_hard/*.png";

# define M_PI           3.14159265358979323846




//===================================================================================================================================================



								// SET OF FUNCTIONS


// Personal notes
void theoryScript()
{
	ofstream fs;
	fs.open("/home/student/workspace/RoVi1-Final-Project/feature_extraction/marker3/genfiles/theory_script.txt");
	fs << "=======================================================================" << endl;
	fs << "			FEATURE EXTRACTION - CORNY MARKER" << endl;
	fs << "=======================================================================" << endl;
	fs << endl;
	fs << "Outline of the method:" << endl;
	fs << endl;
	fs << "		>> Conversion to grayscale." << endl;
	fs << "		>> Keypoint detection using SURF detector." << endl;
	fs << "		>> Matching descriptors using FLANN matcher." << endl;
	fs << "		>> Points used for tracking ---> resulting matching points." << endl;
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
	int p_found, FP, m_found;
	// accumulators: vector<int> TP_vec, FP_vec, FN_vec;

	// User interface
	cout << endl;
	cout << "  FRAME #" << i+1 << " EVALUATION" << endl;
	cout << endl;
	cout << "(" << i+1 << " out of " << total << " | " << progress*100 << "%)" << endl;
	cout << endl;
	cout << endl;
	cout << "	>> MINIMUM # OF POINTS REQUIRED to identify the marker: p_min = 3  " << endl;
	cout << "	>> NUMBER OF POINTS FOUND: p_found = "; 
	cin >> p_found;
	cout << "	>> MARKER FOUND (1 or 0): ";
	cin >> m_found;
	cout << "	>> FALSE marker points DETECTED in this frame: FP = "; 
	cin >> FP;
	cout << endl;
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;
	
	PF_vec.push_back(p_found);
	MF_vec.push_back(m_found);
	FP_vec.push_back(FP);

}

//----------------------------------------------------------------------------------------------------------------------------------

// Detector function --- OpenCV https://docs.opencv.org/master/d5/d6f/tutorial_feature_flann_matcher.html
void detector(Mat temp, Mat src, int examp, int total)
{	
	// Create timer to get runtime 
	long countdown = 5000; // miliseconds
	rw::common::Timer timer = rw::common::Timer(countdown);
	timer.resetAndPause();

//------------------------------------------------------------------

	// DETECTION

	timer.resetAndResume();

	//-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
	int minHessian = 400;
	Ptr<SURF> detector = SURF::create();
	detector->setHessianThreshold(minHessian);
	std::vector<KeyPoint> keypoints_temp, keypoints_src;
	Mat descriptors_temp, descriptors_src;
	detector->detectAndCompute( temp, Mat(), keypoints_temp, descriptors_temp );
	detector->detectAndCompute( src, Mat(), keypoints_src, descriptors_src );

	//-- Step 2: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_temp, descriptors_src, matches );
	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors_temp.rows; i++ )
	{ double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.

	std::vector< DMatch > good_matches;
	for( int i = 0; i < descriptors_temp.rows; i++ )
	{ if( matches[i].distance <= max(2*min_dist, 0.02) )
	   { good_matches.push_back( matches[i]); }
	}

	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches( temp, keypoints_temp, src, keypoints_src,
	             good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	             vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//------------------------------------------------------------------

	double time_past = timer.getTime();
	timer.resetAndPause();
	time_vec.push_back(time_past);

//------------------------------------------------------------------

	cout << "KEYPOINT DETECTION AT FRAME #" << examp+1 << ":" << endl;
	cout << endl;
	cout << "  MAXIMUM AND MINIMUM DISTANCES BETWEEN KEYPOINTS" << endl;
	cout << endl;
	cout << "	>> Maximum distance: " << max_dist << endl;
	cout << "	>> Minimum distance: " << min_dist << endl;
	cout << endl;
	cout << "	>> Number of DETECTED candidate points: " <<  matches.size() << endl;
	cout << "	>> Number of ACTUAL keypoints (good matches): " << good_matches.size() << endl;
	cout << endl;
	cout << "	>> Runtime: " << time_past  << " [secs]" << endl;
	cout << endl;

//------------------------------------------------------------------

	// Save matches to vector
	for( int i = 0; i < (int)good_matches.size(); i++ )
	{ 
		file << "Good Match " << i << "---> (" << good_matches[i].queryIdx << ", " << good_matches[i].trainIdx  << ")     |     "; 
	}// for i

	file << endl; file << endl;

//------------------------------------------------------------------
	
	//Show detected matches
	imshow( "Good Matches", img_matches );
	int sleep_time = 1000000;
	waitKey(sleep_time);

//-----------------------------------------------------------------

	// STATS
	frameStats(examp, total);

}

//-------------------------------------------------------------------------------------------------------

// Function to return global statistics
void globalStats(int sequence_size, string path)
{
	// Global TP, FP and FN:
	int PF_f, FP_f, MF_f;
	PF_f = std::accumulate(PF_vec.begin(), PF_vec.end(), 0);
	FP_f = std::accumulate(FP_vec.begin(), FP_vec.end(), 0);
	MF_f = std::accumulate(MF_vec.begin(), MF_vec.end(), 0);

	// Percentages
	double recon = (double) (MF_f) / (sequence_size);
	double failed = (double) (1 - recon);  

	// Average Points 
	double avg_recon = (double) (PF_f) / sequence_size;
	double FPPI = (double) (FP_f) / sequence_size;

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
	cout << "MARKER RECOGNITION STATISTICS" << endl;
	cout << endl;
	cout << "	>> Total number of marker frames: " << sequence_size << endl;
	cout << "	>> Markers recognised: " << MF_f << endl;
	cout << "	>> Percentage of markers recognised: " << recon * 100 << "%" << endl;
	cout << "	>> Markers NOT recognised: " << (sequence_size - MF_f) << endl;
	cout << "	>> Percentage of fails: " << failed * 100 << "%" << endl;
	cout << "	>> Average number of points per marker: " << avg_recon << endl;
	cout << "	>> False positives per image (FPPI): " << FPPI << endl;
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



int main( int argc, char** argv )
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
	
	// Loading template image
	temp = imread( "/home/student/workspace/RoVi1-Final-Project/sequences/corny_template.png", IMREAD_GRAYSCALE );
	if (temp.empty()) 
	{
        	std::cout << "Template image not found" << endl;
        	return 1;
    	} // if

	// Loading sequence of images	
	glob(path_hard, filenames);
	int imgNum;
	int total = filenames.size();

	cout << "DETECTION BASED ON KEYPOINT DETECTION AND MATCHING" << endl;
	cout << endl;	
	cout << "---------------------------------------------------------------------" << endl;
	cout << endl;

	file.open("/home/student/workspace/RoVi1-Final-Project/feature_extraction/marker3/genfiles/good_matches_hard.txt");
	file << "KEYPOINT MATCHES" << endl;
	file << endl;
	file << " 	>> Format:  Good Match # ---> (keypoint_template, keypoint_frame)" << endl;
	file << endl; file << endl;

	for(imgNum = 0; imgNum < total; imgNum++)
	{
		src = imread(filenames[imgNum], IMREAD_GRAYSCALE );
		if (src.empty()) 
		{
        		std::cout << "Input image not found" << endl;
        		return 1;
    		} // if 

		cout << endl;
		cout << "	>> Image #" << imgNum+1 << " loaded correctly." << endl;
		cout << endl;

		file << "	>> Frame #" << imgNum+1 << ":" << endl;

		// Apply detector
		detector(temp, src, imgNum, total);

	} // for imgNum

	file.close();

//-------------------------------------------------------------------------------

	// OVERALL STATISTICS

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
	  
	  waitKey(0);
	  return 0;
}

