#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes

#include <rw/models/WorkCell.hpp>
#include <rw/common.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>
#include "color.hpp"

// Qt
#include <QTimer>
#include "ui_SamplePlugin.h"

//-------------------------------------------------------

// NAMESPACES

using namespace std;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace cv;
using namespace rw::common;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::proximity;

//------------------------------------------------------

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    	SamplePlugin();
	virtual ~SamplePlugin();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();
	virtual void initialize();

private slots:

	void btnPressed();
	void timer();

	void stateChangedListener(const rw::kinematics::State& state);

 	// NEW FUNCTIONS

	// Global use functions
	vector<rw::math::Transform3D<double> > markerMovements(string fileName);
	void default_restart(Device::Ptr dev, rw::kinematics::MovableFrame* marker, rw::math::Q config, rw::math::Transform3D<double> T_marker_default);
	float du_dvEuclidean( rw::math::Vector2D<double> target, rw::math::Vector2D<double> current );
	void store_jointVector( rw::math::Q q );
	void store_toolPose( rw::math::Transform3D<double> T );
	void store_velVector( rw::math::Q q, double dt );

	// Inverse Kinematics
	rw::math::Q checkJointLimits(Q dq, Q limit,  Q prev);
	rw::math::Q checkVelocityLimits(Q dq, Q limit, double delta_t);
	rw::math::Jacobian image_Jacobian(double z, double f, rw::math::Vector2D<double> imgPoint);
	rw::math::Jacobian stackJacs(double z, double f, vector<rw::math::Vector2D<double> > uv);
	rw::math::Jacobian duToBase(rw::math::Transform3D<double> T_0);
	Eigen::MatrixXd compute_Z_image_q(rw::math::Jacobian Jimage, rw::math::Jacobian S_q, rw::math::Jacobian J_q);	
	rw::math::Q solve_dq(rw::math::Vector2D<double> uv, rw::math::Vector2D<double> reference, Eigen::MatrixXd Zimage);
	
	// Tracking marker frame (no vision)
	rw::math::Vector2D<double> track_1_point(double f, Frame *marker, Frame *camera);
	vector<rw::math::Vector2D<double> > track_3_points(double f, Frame *marker, Frame *camera);
	rw::math::Jacobian duvStack(vector<rw::math::Vector2D<double> > du_dv);

	// Vision
	vector<rw::math::Vector2D<double> > offset( vector<rw::math::Vector2D<double> > points );


private:
	color * rastreator;
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;

    	std::ifstream file;
	std::ofstream qStore, erStore, tpStore, trStore, velStore, timeStore;
	vector<rw::math::Transform3D<double> > markerMovs;
	int total_movs;
	rw::common::Timer chrono;
	double tempo;
    	rw::models::WorkCell::Ptr _wc;
    	rw::kinematics::State _state;
    	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    	rwlibs::simulation::GLFrameGrabber* _framegrabber;

    	Device::Ptr dev;
    	rw::math::Q config;
    	rw::math::Transform3D<double> T_marker_default;
	std::pair<rw::math::Q, rw::math::Q> robot_bounds;
	rw::math::Q vel_limits;
	rw::math::Q accelerationLimits;
	
	rw::math::Vector2D<double> imgReference;
	vector<rw::math::Vector2D<double> > imgRefVec;
	vector<rw::math::Vector2D<double> > visRef;
	vector<double> VStime;
	vector<float> error_container;
	vector<float> max_errors;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
