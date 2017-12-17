#include "color.hpp"

vector<Point2f> color::colorDetector(Mat src)
{
	//imshow("src", src);

	// PRE-PROCESSING
	
	GaussianBlur(src, src, Size(3,3), 2, 2);

//----------------------------------------------------------------

	// CHANGING COLOR SPACE TO HSV
	
	// Conversion
	Mat hsv;
	cvtColor(src, hsv, CV_BGR2HSV);
	//imshow("hsv", hsv);
	

	// Split HSV space in planes
	Mat planes[3];  //    planes[0] = Hue   |   planes[1] = Saturation    |    planes[2] = Value
	split(hsv, planes);
	

	// Thresolding at each individual plane
	
	// Red Circle detection
	Mat redCircle;
	int minHue_red = 5; int maxHue_red= 15; // Readjusted thresholds for simulated images
	inRange(planes[0], minHue_red, maxHue_red, redCircle);
	//imshow("red", redCircle);
	
	
	// Blue circles detection 
	Mat blueCircle;
	int minHue_blue = 95; int maxHue_blue = 145; // Adjusted thresholds for simulated images
	inRange(planes[0], minHue_blue, maxHue_blue, blueCircle);
	//imshow("blue", blueCircle);

	// Combine both thresholds.
	Mat combined;
	addWeighted(redCircle, 1.0, blueCircle, 1.0, 0.0, planes[0]);
	combined = planes[0].clone();

	//imshow("thresholded", combined);

	// Morphological operations                      ----> Parameters readjusted for simulated images
	int m_size = 5;
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5*m_size + 1, 5*m_size + 1), Point(m_size, m_size));
	morphologyEx(combined, combined, MORPH_OPEN, element, Point(-1, -1), 2);

	//imshow("morph", combined);	

	/* Thresholding */
	int th = 80;
	threshold(combined, combined, th, 255, 0);
	//imshow("combined", combined);

//----------------------------------------------------------------

	// CIRCLE DETECTION USING CONTOURS IN BINARY IMAGES

	// FINDING CONTOURS

	// Canny Edge detector
	int canny_th = 100;
	Canny(combined, combined, canny_th, canny_th, 3);
	//imshow("Object Perception (Recognition)", combined);

	// Find contours
	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;

	findContours(combined, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

//----------------------------------------------------------------

	// Approximate contours to circles
	size_t imax = contours.size();
  	vector<vector<Point> > contours_cc(contours.size());
  	vector<Point2f>centers(contours.size());
  	vector<float> radius(contours.size());
	

  	for( size_t i = 0; i < imax; i++ )
     	{ 
		approxPolyDP( Mat(contours[i]), contours_cc[i], 3, true );
	       	minEnclosingCircle( (Mat)contours_cc[i], centers[i], radius[i] );

	} // for i	

	return centers;	

} // colorDetector


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


vector<rw::math::Vector2D<double> > color::storeKeyPoints(vector<Point2f> centers)
{
	// Declare
	vector<rw::math::Vector2D<double > > pointsToTrack;
	if( centers.size() < 3 )
	{
		for( size_t j=0; j<centers.size(); j++ )
     		{
			// Storing centers
			rw::math::Vector2D<double> point; 
			point(0) = (double) centers[j].x;
			point(1) = (double) centers[j].y;
			pointsToTrack.push_back(point);
		
     		} // for j

	} // if imax

	else 
	{
		for( size_t i=0; i<3; i++ )
		{
			// Storing centers
			rw::math::Vector2D<double> pointt; 
			pointt(0) = (double) centers[i].x;
			pointt(1) = (double) centers[i].y;
			pointsToTrack.push_back(pointt);

		} // for i

	} // else

	return pointsToTrack;

} // storeKeyPoints
