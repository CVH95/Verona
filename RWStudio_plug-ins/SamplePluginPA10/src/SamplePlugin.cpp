#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>
#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h> 
#include <numeric>
#include <string>



using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::trajectory;
using namespace rw::graphics;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;


//=========================================================================================================================================================================================================

// GLOBAL VARIABLES

// Booleans
bool mode1 = false; // tracking 1 point (btn1)
bool mode2 = false; // tracking 3 points (btn2)
//bool mode3 = false; // using vision with color marker (btn3)

// Counters
int counter = 0;
int bt1 = 0;
int bt2 = 0;
long countdown = 1000; 

//Focal length
double z = 0.5;
double f = 823;
int u0, v0; // Initial image coordinates of the marker (the ones that we aim to keep constant).

// Tolerance level (epsilon) for Inverse Kinematics
float eps = 1e-2;

// Marker motion files
string file_slow = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/motions/MarkerMotionSlow.txt"; //MarkerMotionSlow
string file_medium = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/motions/MarkerMotionMedium.txt";
string file_fast = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/motions/MarkerMotionFast.txt";

// Ofstream files containing different variables
ofstream t3d, q7, tcpf;

// Output files
string test1_slow = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/genfiles/test1_slow.txt";
string test1_med = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/genfiles/test1_med.txt";
string test1_fast = "/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/genfiles/test1_fast.txt";


//=========================================================================================================================================================================================================



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    	connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn3    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}



SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}



void SamplePlugin::initialize() 
{
	
	log().info() << "\n";
	log().info() << "·········································" << "\n";
    	log().info() << "\n";
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

	// Motion file
    	markerMovs = markerMovements(file_slow);
	total_movs = markerMovs.size();
	
	// Keep modes off until a button is toggled.
	mode1 = false;
	mode2 = false;

	// Initialize counters
	bt1 = 0;
	bt2 = 0;

	// Initialize chronometer
	// Create timer 
	long countdown = 1000; // miliseconds
	tempo = (double) countdown/1000;
	chrono = rw::common::Timer(countdown);
	chrono.resetAndPause();

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/workspace/workcells/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	config = rw::math::Q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);

	// Load Lena image
	Mat im, image;
	im = imread("/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/src/pa_icon.png", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_winTitle->setText(QString("Camera View"));
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
	log().info() << "\n";
    	log().info() << "·········································" << "\n";
   	log().info() << "\n";

} // initialize



void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "		      ****************************" << "\n";
    log().info() << "			VISUAL SERVOING PLUG-IN" << "\n";
    log().info() << "		      ****************************" << "\n";
    log().info() << "\n";
    log().info() << "\n";
    log().info() << "INSTRUCTIONS:" << "\n";
    log().info() << "\n";
    log().info() << "		>> btn0 ---> Initial settings." << "\n";
    log().info() << "		>> btn1 ---> Start/Stop timer for TRACKING 1 POINT (simulated vision mode)." << "\n";
    log().info() << "		>> btn2 ---> Start/Stop timer for TRACKING 3 POINTS (simulation vision mode)." << "\n";
    log().info() << "		>> btn3 ---> Restart simulations (return to default scenario)." << "\n";
    log().info() << "\n";
    log().info() << "----------------------------------------------------------------------------------------" << "\n";
    log().info() << "\n";

    _wc = workcell;
    _state = _wc->getDefaultState();
    dev = _wc->getDevices().front();
    dev->setQ(config, _state);

    if (_wc != NULL) 
    {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL)
	{
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	} // if textureFrame
	
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	} // if bgFrame
	
	// Create a movable fame for marker
	MovableFrame *marker = (MovableFrame *)_wc->findFrame("Marker");
        if (marker == NULL)
        {
            RW_THROW("MARKER FRAME NOT FOUND");
        } // if marker

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	if (cameraFrame != NULL) 
	{
		if (cameraFrame->getPropertyMap().has("Camera")) 
		{
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);

			// Starting image
			_framegrabber->grab(cameraFrame, _state);       
                	const Image &image = _framegrabber->getImage(); 
                	// Convert to OpenCV image
                	Mat im = toOpenCVImage(image);
                	Mat imFlipLabel;
	                // Fit imageinto Qlabel
	                cv::flip(im, imFlipLabel, 0);
	
		} // if getPropertyMap
	} // if cameraFrame

	// Find marker
        marker = (MovableFrame*) _wc->findFrame("Marker");
        T_marker_default = marker->getTransform(_state); 

	log().info() << "\n";
	log().info() << "\n";
	log().info() << "GENERAL INFORMATION:" << "\n";
	log().info() << "\n";
	log().info() << "	>> WorkCell:  " << workcell->getFilename() << "\n";
    	log().info() << "	>> Device:  " << dev->getName() << "\n";
	log().info() << "	>> Number of joints:  " << dev->getDOF() << "\n";
	log().info() << " 	>> Base coordinates:  " << dev->getBase() << "\n";
	log().info() << "	>> End coordinates of the device:  " << dev->getEnd() << "\n";
    	log().info() << "	>> Default joint configuration:  " << config << "\n";
	log().info() << "	>> Default marker coordinates:  " << T_marker_default << "\n";	
	log().info() << "\n";
	log().info() << "\n";

	
	robot_bounds = dev->getBounds(); // To get Joint constraints std::pair is required.
	vel_limits = dev->getVelocityLimits();
	accelerationLimits = dev->getAccelerationLimits();
	
	log().info() << "ROBOT CONSTRAINTS AND LIMITS:" << "\n";
	log().info() << "\n";
	log().info() << "	>> Joint costraints:  " << "\n";
	log().info() << "				+ Upper bounds --> " << robot_bounds.first << "\n";
	log().info() << "				+ Lower bounds --> " << robot_bounds.second << "\n";
	log().info() << "	>> Velocity limits (maximal):  " << vel_limits << "\n";
	log().info() << "	>> Joint velocity minimal limits are considered to be q_v_min = - q_v_max. " << "\n";
	log().info() << "	>> Acceleration limits (maximal):  " << accelerationLimits << "\n";
	log().info() << "	>> Joint acceleration minimal limits are considered to be q_a_min = - q_a_max. " << "\n";
        log().info() << "\n";	
	log().info() << "\n";
	log().info() << "--------------------------------------------------------------------------------" <<"\n";
	log().info() << "\n";
	log().info() << "\n";
	log().info() << "ACTIONS:" << "\n";
	log().info() << "\n";

    } // if _wc

} // open


void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
	if(obj==_btn0) // Initial settings
	{
		log().info() << "Button 0 ---> Setup\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/student/workspace/RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/backgrounds/texture2.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();

	} // if _btn0 

	else if(obj==_btn1) // track 1 point, no vision
	{
		// activate mode 1.
		mode1 = true;
		mode2 = false;

		// INITIAL CONFIGURATION
	
		// If bt1 is pressed for the first time after uploading/restarting the plugin:		
		if( bt1 == 0 )
		{
			// Set marker to initial position
			MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
			Frame* camera = _wc->findFrame("Camera");
			dev->setQ(config, _state);
			marker->moveTo(markerMovs[0], _state);
			getRobWorkStudio()->setState(_state);
	
			// Get camera in initial view
			if (_framegrabber != NULL) 
			{
				// Get the image as a RW image
				Frame* cameraFrame = _wc->findFrame("CameraSim");
				_framegrabber->grab(cameraFrame, _state);
				const Image& image = _framegrabber->getImage();
	
				// Convert to OpenCV image
				Mat im = toOpenCVImage(image);
				Mat imflip;
				cv::flip(im, imflip, 0);

				// Show in QLabel
				QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
				QPixmap p = QPixmap::fromImage(img);
				unsigned int maxW = 400;
				unsigned int maxH = 800;
				_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		
			} // if _frame grabber

			// Get target point (Center of the marker)
			imgReference = track_1_point(f, marker, camera);

			bt1 = 2; // update PushButton1 counter 
	
			// Start timer for the first time
			_timer->start(countdown); // run 10 Hz timer
			log().info() << "Button 1 ---> Timer initialized\n";
 
		} // if bt1

		//--------------------------------------------------------------------
	
		// TIMER		

		// when bt1 has already been toggled once
		else
		{
			// Toggle the timer on and off
			if (!_timer->isActive())
			{
				 _timer->start(1000); // run 10 Hz
				chrono.resume();
				log().info() << "Button 1 ---> Timer running\n";

			} // if _timer
	
			else
			{
				_timer->stop();
				chrono.pause();
				log().info() << "Button 1 ---> Timer stopped\n";

			} // else _timer

		} // else bt1
		
	} // else if _btn1  

	else if(obj==_btn2) // track 3 points, no vision
	{
		mode1 = false;
		mode2 = true;
		
		// INITIAL CONFIGURATION
	
		// If bt2 is pressed for the first time after uploading/restarting the plugin:		
		if( bt2 == 0 )
		{
			// Set marker to initial position
			MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
			Frame* camera = _wc->findFrame("Camera");
			dev->setQ(config, _state);
			marker->moveTo(markerMovs[0], _state);
			getRobWorkStudio()->setState(_state);
	
			// Get camera in initial view
			if (_framegrabber != NULL) 
			{
				// Get the image as a RW image
				Frame* cameraFrame = _wc->findFrame("CameraSim");
				_framegrabber->grab(cameraFrame, _state);
				const Image& image = _framegrabber->getImage();
	
				// Convert to OpenCV image
				Mat im = toOpenCVImage(image);
				Mat imflip;
				cv::flip(im, imflip, 0);

				// Show in QLabel
				QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
				QPixmap p = QPixmap::fromImage(img);
				unsigned int maxW = 400;
				unsigned int maxH = 800;
				_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
		
			} // if _frame grabber

			// Get target point (Center of the marker)
			imgRefVec = track_3_points(f, marker, camera);

			bt2 = 2; // update PushButton1 counter 
	
			// Start timer for the first time
			_timer->start(countdown); // run 10 Hz timer
			log().info() << "Button 2 ---> Timer initialized\n";
 
		} // if bt2

		//--------------------------------------------------------------------
	
		// TIMER		

		// when bt2 has already been toggled once
		else
		{
			// Toggle the timer on and off
			if (!_timer->isActive())
			{
				 _timer->start(1000); // run 10 Hz
				chrono.resume();
				log().info() << "Button 2 ---> Timer running\n";

			} // if _timer
	
			else
			{
				_timer->stop();
				chrono.pause();
				log().info() << "Button 2 ---> Timer stopped\n";

			} // else _timer

		} // else bt2
		
	} // else if btn2
	
	else if(obj==_btn3) // restart
	{
		MovableFrame *marker = (MovableFrame*) _wc->findFrame("Marker");
		log().info() << "Button 3 ---> Restore default scenario\n";
		default_restart(dev, marker, config, T_marker_default);
	
	}// else if btn3

	else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}


void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

//===========================================================================================================================================================================================================

		// GLOBAL ACTIONS

// Function to get marker movements
vector<rw::math::Transform3D<double> > SamplePlugin::markerMovements(string fileName)
{
	// Open file
	file = ifstream(fileName);
	log().info() << "Using motion file:  " << fileName << "\n";

	// Read line
	std::string line;
	double x, y, z, R, P, Y;

	vector<rw::math::Transform3D<double> > motions;

       	if( file.is_open() ) 
	{
		while( std::getline(file,line) )
		{
			std::stringstream lineStream(line);
			lineStream >> x;
			lineStream >> y;
			lineStream >> z;
			lineStream >> R;
			lineStream >> P;
			lineStream >> Y;

			rw::math::Vector3D<double> xyz_vector = rw::math::Vector3D<double>(x, y, z);
			rw::math::RPY<double> RPY_vector = rw::math::RPY<double>(R, P, Y);

			rw::math::Transform3D<double> T_line = rw::math::Transform3D<double>(xyz_vector, RPY_vector.toRotation3D());

			motions.push_back(T_line);

		} // while

		log().info() << "Number of movements:  " << motions.size() << "\n";		

	} // if file

    	return motions;

} // markerMovements


//------------------------------------------------------------------


// Function to restart simulation
void SamplePlugin::default_restart(Device::Ptr dev, rw::kinematics::MovableFrame* marker, rw::math::Q config, rw::math::Transform3D<double> T_marker_default)
{
	// Clear all
	markerMovs.clear();
	bt1 = 0;
	bt2 = 0;
	counter = 0;
	mode1 = false;
	mode2 = false;
	
	log().info() << "\n";
	log().info() << "==========================================================================" <<"\n";
	log().info() << "\n";
	log().info() << "Restoring default scene configuration..." << "\n";
	log().info() << "Default joint configuration:  " << config << "\n";
	log().info() << "Default marker coordinates:  " << T_marker_default << "\n";
	log().info() << "\n";
	// Stop timer if necessary

	if (_timer->isActive())
	{
	        _timer->stop();
	        log().info() << "Timer was running. Now it is stopped." << "\n";
	}

	// Set robot into initial state:
	dev->setQ(config, _state);
	// Set marker to initial position
	marker->setTransform(T_marker_default, _state);
	
	// update 
    	getRobWorkStudio()->setState(_state);

	log().info() << "Robot set to:  " << dev->getQ(_state) << "\n";
	log().info() << "Default marker coordinates:  " << marker->getTransform(_state) << "\n";
	log().info() << "\n";
	log().info() << "==========================================================================" <<"\n";
	log().info() << "\n";

	// close motion file
	file.close();

	// Re-initialize
	initialize();

} // default_restart

//---------------------------------------------------------------------------------------------------------

// Euclidean distance between two points
float SamplePlugin::du_dvEuclidean( rw::math::Vector2D<double> target, rw::math::Vector2D<double> current )
{
	// d(t, c) = sqrt( (c0-t0)² + (cq-t1)² );
	float distance;
	double d0 = current(0) - target(0);
	double d1 = current(1) - target(1);
	distance = (float) sqrt( d0*d0 + d1*d1 );
	
	return distance;

} // du_dvEuclidean

//===========================================================================================================================================================================================================



									// ROBOT KINEMATICS
	

// Check Velocity limits for a new Q_new = Q_past + dQ.
// true == within limits
// false == need of time scaling  -> Function also performs scaling
rw::math::Q SamplePlugin::checkVelocityLimits(Q dq, Q limit, double delta_t)
{
	vector<int> scores;
	int score = 0;
	int dq_size = dq.size();
	for( size_t i = 0; i < limit.size(); i++ )
	{
		double lim = abs(limit(i));
		double div = dq(i) / delta_t;
		double k = abs(div);
		
		// First condition --> Check if dq is within velocity limits.

		// Sbagliato
		if( k > lim )
		{
			score = 0;
	
			// Scaling
			if( dq(i) > 0) // Positive velocity
			{
				dq(i) = lim * delta_t;

			} // if dq
			
			else //Negative velocity
			{			
				dq(i) = - (lim * delta_t);
			
			} // else dq

		} // if k

		// A posto
		else 
		{
			score = score + 1;
			scores.push_back(score);

			dq(i) = dq(i);

		} // else k
	}

	// log info
	int scores_size = scores.size();
	if (scores_size == dq_size)
	{
		log().info() << "	>> Within velocity limits:   TRUE." << "\n";

	} // if
	
	else
	{	
		log().info() << "	>> Within velocity limits:   FALSE." << "\n";

	} // else 

	scores.clear();

	return dq;
	
} // CheckVelocityLimits

//-------------------------------------------------------------------------------

// Funtion that calculates the image Jacobian Jimage for a certain point P.
// Inputs 	---->		P(x,y,z)->Fcam
//				P(u,v)->image 
//				f == focal length [pixels]
// Implementation of the formula given at Robotics Notes page 51.

rw::math::Jacobian SamplePlugin::image_Jacobian(double z, double f, rw::math::Vector2D<double> imgPoint)
{
	// Constructor
	rw::math::Jacobian Jimage(2,6);

	// Decomposing image coordinates (u, v)
	double u = imgPoint(0);
	double v = imgPoint(1);

	// Row u:
	Jimage(0,0) = (double) -(f/z);
	Jimage(0,1) = (double) 0;
	Jimage(0,2) = (double) u/z;
	Jimage(0,3) = (double) (u*v)/f;
	Jimage(0,4) = (double) -(f*f + u*u)/f;
	Jimage(0,5) = (double) v;

	// Row v:
	Jimage(1,0) = (double) 0;
	Jimage(1,1) = (double) -(f/z);
	Jimage(1,2) = (double) v/z;
	Jimage(1,3) = (double) (f*f + v*v)/f;
	Jimage(1,4) = (double) -(u*v / f);
	Jimage(1,5) = (double) -u;

	return Jimage;
}

//--------------------------------------------------------------------------------

rw::math::Jacobian SamplePlugin::stackJacs(double z, double f, vector<rw::math::Vector2D<double> > uv)
{
	vector<rw::math::Jacobian> Jimage_mult;
	for( size_t i = 0; i<uv.size(); i++ )
	{
		rw::math::Jacobian Jimg_i = image_Jacobian(z, f, uv[i]);
		Jimage_mult.push_back(Jimg_i);		
		
	} // for i

	Eigen::MatrixXd mat(0, Jimage_mult[0].e().cols() );

	for( size_t j=0; j<Jimage_mult.size(); j++ )
	{
		Eigen::MatrixXd acc(mat.rows() + Jimage_mult[j].e().rows(), Jimage_mult[j].e().cols());
		acc << mat, Jimage_mult[j].e();
		mat = acc;

	} // for j
	
	rw::math::Jacobian jacs_combo(mat);
		
	return jacs_combo;

} //stackJacs
//--------------------------------------------------------------------------------

rw::math::Jacobian SamplePlugin::duToBase(rw::math::Transform3D<double> T_0)
{	
	// Defining Rotation matix from base to tool
	rw::math::Rotation3D<double> R_bt = T_0.R();
	// Transposing R_bt ------> The transpose of a rotation matrix is the same as the inverse.
	rw::math::Rotation3D<double> R_bt_T = rw::math::inverse(R_bt);
	
	// Building S_q
	rw::math::Jacobian S_q = Jacobian(R_bt_T);	

	return S_q;
}

//--------------------------------------------------------------------------------

//Zimage_q (2xn=6)= Jimage_r(2x6) * S_q(6x6) * J_q(6xn=6)"
Eigen::MatrixXd SamplePlugin::compute_Z_image_q(rw::math::Jacobian Jimage, rw::math::Jacobian S_q, rw::math::Jacobian J_q)
{
	// Calculate Zimage_q
	Eigen::MatrixXd Zimage_q = Jimage.e()*S_q.e()*J_q.e();

	return Zimage_q;
	
}

//-----------------------------------------------------------------------------------

rw::math::Q SamplePlugin::solve_dq(rw::math::Vector2D<double> uv, rw::math::Vector2D<double> reference, Eigen::MatrixXd Zimage)
{
	// Calculate displacement (du, dv)
	rw::math::Vector2D<double> du_dv;
	du_dv(0) = reference(0) - uv(0);
	du_dv(1) = reference(1) - uv(1);

	log().info() << "	>> Displacement (du, dv):  " << du_dv << "\n";
	log().info() << "\n";

	log().info() << "	>> Zimage:  " << "\n";
	log().info() << Zimage << "\n";
	log().info() << "\n";

	// Calculate Zimage pseudoinverse:
	Eigen::MatrixXd ZiT = Zimage.transpose(); // Tranpose of Zi
	Eigen::MatrixXd Zinv = ZiT * LinearAlgebra::pseudoInverse(Zimage*ZiT);
	
	rw::math::Jacobian dqE(Zinv*du_dv.e());
	

	// Convert to joint vector
	rw::math::Q dq(dqE.e());

	return dq;

} // solve_dq

//-----------------------------------------------------------------------------


//===========================================================================================================================================================================================================


								// TRACKING MARKER FRAME (NO VISION)

rw::math::Vector2D<double> SamplePlugin::track_1_point(double f, Frame *marker, Frame *camera)
{
	// Marker point in the image
	rw::math::Vector2D<double> point2D;

	// Get homogeneous 3D coordinates of the center of the marker plate
	rw::math::Vector3D<double> markerCenter = rw::math::Vector3D<double>(0, 0, 0);
	rw::math::Transform3D<double> T_marker_cam = rw::kinematics::Kinematics::frameTframe(camera, marker, _state); // inverse(marker->....)
	
	rw::math::Vector3D<double> pos3D = T_marker_cam * markerCenter;
	
	// Image coordinates 
	point2D(0) = f * pos3D(0) / pos3D(2);
	point2D(1) = f * pos3D(1) / pos3D(2);
	
	return point2D;

} // track_1_point

//----------------------------------------------------------------------------------------------------

vector<rw::math::Vector2D<double> > SamplePlugin::track_3_points(double f, Frame *marker, Frame *camera)
{
	// Marker point in the image
	vector<rw::math::Vector2D<double> > point2D;

	// Get homogeneous 3D coordinates of the 3 points of the marker plate
	rw::math::Vector3D<double> P1 = rw::math::Vector3D<double>(0.15, 0.15, 0);
	rw::math::Vector3D<double> P2 = rw::math::Vector3D<double>(0.15, -0.15, 0);
	rw::math::Vector3D<double> P3 = rw::math::Vector3D<double>(-0.15, 0.15, 0);

	rw::math::Transform3D<double> T_marker_cam = rw::kinematics::Kinematics::frameTframe(camera, marker, _state); // inverse(marker->....)
	
	vector<rw::math::Vector3D<double> > pos3D; 
	pos3D.push_back(T_marker_cam * P1);
	pos3D.push_back(T_marker_cam * P2);
	pos3D.push_back(T_marker_cam * P3);
	
	// Image coordinates 
	for (size_t i=0; i<pos3D.size(); i++ ) 
	{
		rw::math::Vector3D<double> current = pos3D[i];
		rw::math::Vector2D<double> D_i; 
		D_i(0) = f * current(0) / current(2);
		D_i(1) = f * current(1) / current(2);
		point2D.push_back(D_i);

	} // for i
	
	return point2D;

} // track_3_point

//-----------------------------------------------------------------------------------------------------

rw::math::Jacobian SamplePlugin::duvStack(vector<rw::math::Vector2D<double> > du_dv)
{
	vector<rw::math::Jacobian> duv_j;
	for( size_t i=0; i<du_dv.size(); i++)
	{
		duv_j.push_back(rw::math::Jacobian( du_dv[i].e() ) );
	
	} // for i
	
	Eigen::MatrixXd mat(0, duv_j[0].e().cols() );

	for( size_t j=0; j<duv_j.size(); j++ )
	{
		Eigen::MatrixXd acc(mat.rows() + duv_j[j].e().rows(), duv_j[j].e().cols());
		acc << mat, duv_j[j].e();
		mat = acc;

	} // for j
	
	rw::math::Jacobian duv_combo(mat);

	return duv_combo;

} // duvStack

//===========================================================================================================================================================================================================



									// COMPUTER VISION





//===========================================================================================================================================================================================================

		// TIMER

void SamplePlugin::timer()
{ 
	chrono.resetAndResume(); // crhono to calculate dt

	log().info() << "\n";
	log().info() << "--------------------------------------------------------------------------------" <<"\n";
	log().info() << "\n";
	
	MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
	Frame* camera = _wc->findFrame("Camera");
	
	
	// Move marker
	marker->moveTo(markerMovs[counter++], _state);	
	log().info() << "	>> Marker moved to position # " << counter << ":   " << markerMovs[counter] << "\n";
	log().info() << "\n";

	if (_framegrabber != NULL) 
	{
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		Frame* markerFrame = _wc->findFrame("Marker");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);
	
		// current joint configuration
		rw::math::Q q_prev = dev->getQ(_state);

	//---------------------------------------------------------

			     // CONTROL LOOP:

	//---------------------------------------------------------

		if ( mode1 == true && mode2 == false ) // tracking 1 point
		{

			// Get new image point:
			rw::math::Vector2D<double> newP = track_1_point(f, markerFrame, cameraFrame);

			// Distance 
			float distance = du_dvEuclidean( imgReference, newP );

			double tempoFE = chrono.getTime();
			double dt = tempo - tempoFE;
			log().info() << "	>> Feature extraction calculated in:   " << tempoFE << "\n";
			log().info() << "	>> Image coordinates of marker center:   " << newP << "\n";
			log().info() << "	>> Target coordinates:   " << imgReference << "\n";
			log().info() << "	>> Euclidean distance:   " << distance << "\n";
			log().info() << "\n";

			// IK solver
		
			// Get J(q) Jacobian and S(q) matrix.			
			rw::math::Jacobian J_q = dev->baseJframe(cameraFrame, _state);
			rw::math::Transform3D<double> T_current = dev->baseTframe(cameraFrame, _state);
			rw::math::Jacobian S_q = duToBase(T_current);

			// Get Jimage and Zimage
			rw::math::Jacobian Jimage = image_Jacobian(z, f, newP);
			Eigen::MatrixXd Zimage = compute_Z_image_q(Jimage, S_q, J_q);

			// Solve dq
			rw::math::Q dq_raw = solve_dq( newP, imgReference, Zimage );
			rw::math::Q dq = checkVelocityLimits(dq_raw, vel_limits, dt);

			// Update robot state
			rw::math::Q q = dev->getQ(_state);
			q += dq;
	
			// Update robot state
			dev->setQ(q, _state);
		
			// Find frames
			Frame* checkMarker = _wc->findFrame("Marker");
			Frame* checkCam = _wc->findFrame("CameraSim");

			// Update newP
			newP = track_1_point(f, checkMarker, checkCam );
			distance = du_dvEuclidean(imgReference, newP); // Update distance
			log().info() << "	>> New image coordinates ------------------------->  " << newP << "\n";
			log().info() << "	>> New Euclidean distance ------------------------->  " << distance << "\n";
		
			// End of IK solver 

			double tempoEND = chrono.getTime();
			double tempoIK = tempoEND - tempoFE;
			chrono.resetAndPause();
	
			// Update RWStudio scene
			getRobWorkStudio()->setState(_state);

			// Display info
			rw::math::Q q_current = dev->getQ(_state);
			log().info() << "	>> Joint displacement dq:   " << q_current - q_prev << "\n";
			log().info() << "\n";
			log().info() << "	>> PREVIOUS joint vector Q:   " << q_prev << "\n";
			log().info() << "	>> UPDATED joint vector Q:   " << q_current << "\n";
			log().info() << "\n";
			log().info() << "	>> Inverse Kinematics solved in:   " << tempoIK << "\n";
			log().info() << "\n";

			rw::math::Transform3D<double> TCP = dev->baseTframe(camera, _state);		
			log().info() << "	>> Updated tool pose:   " << TCP << "\n";
			log().info() << "	>> Visual servoing operating time:   " << tempoEND << "\n";
			log().info() << "	>> Time consumed ------------------>  " << (tempoEND / tempo)*100 << "%" << "\n";
		
		} // if mode1
		
		else if (mode2 == true && mode1 == false) // tracking 3 points
		{  
			// Get new image points 
			vector<rw::math::Vector2D<double> > newVec2D = track_3_points(f, marker, camera);

			log().info() << "	>> Reference points:" << "\n";
			log().info()<< "\n";
			for ( size_t i = 0; i < imgRefVec.size(); i++ )
			{
				log().info() << imgRefVec[i] << "\n";
			
			} // for i

			log().info()<< "\n";

			log().info() << "	>> Points being tracked:" << "\n";
			log().info()<< "\n";
			for ( size_t j = 0; j < newVec2D.size(); j++ )
			{
				log().info() << newVec2D[j] << "\n";

			} // for j

			log().info()<< "\n";

			double tempoFE3 = chrono.getTime();
			double dt3 = tempo - tempoFE3;
			log().info() << "	>> Feature extraction calculated in:   " << tempoFE3 << "\n";
			log().info() << "\n";
			
			vector<rw::math::Vector2D<double> > du_dv;
			for( size_t k = 0; k < newVec2D.size(); k++ )
			{
				rw::math::Vector2D<double> t;
				rw::math::Vector2D<double> po = newVec2D[k];
				rw::math::Vector2D<double> re = imgRefVec[k];
				t(0) = re(0) - po(0);
				t(1) = re(1) - po(1);
				du_dv.push_back(t);
				log().info() << "	>> Displacement of point " << k << ":  " << du_dv[k] <<"\n";

			} // for k

			log().info() << "\n";	
	
			vector<float> euclides;
			for( size_t l=0; l < newVec2D.size(); l++ )
			{
				float euc = du_dvEuclidean(imgRefVec[l], newVec2D[l]);
				log().info() << "	>> Euclidean distance between points " << l << ":  " << euc << "\n";
				euclides.push_back(euc);

			} // for l		

			log().info()<< "\n";
		
			// IK solver

			// Group du_dv 
			rw::math::Jacobian disp = duvStack(du_dv);
	
	//---------------------------------------------------------

			     // CONTROL LOOP:

	//---------------------------------------------------------

			// Stack jacobians (2*npointsx6)
			rw::math::Jacobian Jimage = stackJacs(z, f, newVec2D);

			// Compute Sq (6x6)
			rw::math::Transform3D<double> T0 = dev->baseTframe(camera, _state);
			rw::math::Jacobian Sq = duToBase(T0);

			// Compute Jq (6x7)
			rw::math::Jacobian Jq = dev->baseJframe(camera, _state);

			// Compute Zimage (2*npointsx7)
			Eigen::MatrixXd Zimage = compute_Z_image_q(Jimage, Sq, Jq);
		
			// Calculate Zimage pseudoinverse:
			Eigen::MatrixXd ZiT = Zimage.transpose(); // Tranpose of Zi 
			Eigen::MatrixXd Zinv = ZiT * LinearAlgebra::pseudoInverse(Zimage*ZiT);

			rw::math::Jacobian dqE(Zinv*disp.e());
	
			// Convert to joint vector
			rw::math::Q dq_raw3(dqE.e());
	
			//rw::math::Q dq_raw3 = solve_dq_robust( newVec2D, imgRefVec,  marker, camera);
			rw::math::Q dq3 = checkVelocityLimits(dq_raw3, vel_limits, dt3);

			// Update robot state
			rw::math::Q q3 = dev->getQ(_state);
			q3 += dq3;
	
			// Update robot state
			dev->setQ(q3, _state);

			// End of IK solver 

			double tempoEND3 = chrono.getTime();
			double tempoIK3 = tempoEND3 - tempoFE3;
			chrono.resetAndPause();
		
			// Find frames
			Frame* checkMarker3 = _wc->findFrame("Marker");
			Frame* checkCam3 = _wc->findFrame("Camera");

			// Update newP
			newVec2D = track_3_points(f, checkMarker3, checkCam3 );
			log().info() << "	>> New camera position of tracked points:" << "\n";
			log().info()<< "\n";
			for ( size_t h = 0; h < newVec2D.size(); h++ )
			{
				log().info() << newVec2D[h] << "\n";

			} // for h
			
			log().info()<< "\n";

			// Error measured in Euclidean distance
			vector<float> error;
			for( size_t m=0; m < newVec2D.size(); m++ )
			{
				float er = du_dvEuclidean(imgRefVec[m], newVec2D[m]);
				log().info() << "	>> Error measured in Euclidean distance (P" << m << "):  " << er << "\n";
				error.push_back(er);

			} // for l		

			log().info()<< "\n";

			// Update RWStudio scene
			getRobWorkStudio()->setState(_state);

			// Display info
			rw::math::Q q_current3 = dev->getQ(_state);
			log().info() << "	>> Joint displacement dq:   " << dq3 << "\n";
			log().info() << "\n";
			log().info() << "	>> PREVIOUS joint vector Q:   " << q_prev << "\n";
			log().info() << "	>> UPDATED joint vector Q:   " << q_current3 << "\n";
			log().info() << "\n";
			log().info() << "	>> Inverse Kinematics solved in:   " << tempoIK3 << "\n";
			log().info() << "\n";

			rw::math::Transform3D<double> TCP3 = dev->baseTframe(cameraFrame, _state);		
			log().info() << "	>> Updated tool pose:   " << TCP3 << "\n";
			log().info() << "	>> Visual servoing operating time:   " << tempoEND3 << "\n";
			log().info() << "	>> Time consumed ------------------>  " << (tempoEND3 / tempo)*100 << "%" << "\n";

		} // else if mode 2

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

		// Update target reference
		// imgReference = newP;

	} // if _framegrabber

	log().info() << "\n";
	log().info() << "--------------------------------------------------------------------------------" <<"\n";
	log().info() << "\n";
	
	// Finish .txt file
	if (counter == total_movs)
	{
		_timer->stop();
		chrono.resetAndPause(); // crhono to calculate dt
		log().info() << "\n";
		log().info() << "Timer stopped\n";
		log().info() << "Task completed. Press PushButton 2 to restore scene or read instructions for more options.\n";
		log().info() << "\n";

	} // if counter

} // timer()
