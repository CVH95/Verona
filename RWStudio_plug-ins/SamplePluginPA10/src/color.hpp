#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>
#include <rw/math.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

# define M_PI           3.14159265358979323846

class color {
public:
	vector<Point2f> colorDetector(Mat src);
	vector<rw::math::Vector2D<double> > storeKeyPoints(vector<Point2f> centers);
private:
	int imgMax;
};
