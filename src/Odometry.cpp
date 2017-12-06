#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_CPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_CPP

#include "cnbiros_wheelchair/Odometry.hpp"

namespace cnbiros {
	namespace wheelchair {

Odometry::Odometry(ros::NodeHandle* node, double axle, double diameter, 
				   int revolution, std::string name) : 
				   cnbiros::core::NodeInterface(node, name) {

	// Initialize odometry pointer
	this->odometry_ = nullptr;

	// Initialize wheelchair parameters
	this->axle_			= axle;
	this->diameter_		= diameter;
	this->revolution_	= revolution;

	// Initialize transformation message
	this->mtransform_.header.frame_id = "odom";
	this->mtransform_.child_frame_id  = "base_link";

	// Initialize odometry message
	this->modometry_.header.frame_id = "odom";
	this->modometry_.child_frame_id  = "base_link";

	// Initialize publisher
	this->rospub_ = node->advertise<nav_msgs::Odometry>("odom", 100);
}

Odometry::~Odometry(void) {
	if(this->odometry_ != nullptr)
		delete this->odometry_;
}

int Odometry::Open(const std::string& lport, const std::string& rport) {

	int retcod = 0;

	this->lport_ = lport;
	this->rport_ = rport;

	try {
		this->odometry_ = new OdometryThread(this->lport_.c_str(), 
											 this->rport_.c_str(),
											 this->axle_, this->diameter_,
											 this->revolution_);
	} catch (std::runtime_error& e) {
		retcod = -1;
		throw std::runtime_error(e.what());
	}

	ROS_INFO("USB ports %s and %s opened. Odometry set.\n", this->lport_.c_str(),
															this->rport_.c_str());

	this->Reset();

	return retcod;
}

int Odometry::Reset(void) {

	int retcod = -1;

	if(this->odometry_ != nullptr) {
		this->odometry_->resetOdometry();
		retcod = 0;
	}

	return retcod;
}

int Odometry::Set(double x, double y, double theta) {
	int retcod = -1;

	if(this->odometry_ != nullptr) {
		this->odometry_->setOdometry(x, y, theta);
		retcod = 0;
	}

	return retcod;
}

int Odometry::Get(double *x, double *y, double *theta,
				  double *vx, double *vy, double *vtheta) {
	
	int retcod = -1;

	if(this->odometry_ != nullptr) {
		this->odometry_->getOdometry(x, y, theta, vx, vy, vtheta);
		retcod = 0;
	}

	return retcod;
}

void Odometry::onRunning(void) {

	double x, y, theta;
	double vx, vy, vtheta;
	geometry_msgs::Quaternion oquat;
	static tf::TransformBroadcaster	broadcaster; 
	ros::Time ctime;

	if(this->odometry_ != nullptr) {

		this->odometry_->getOdometry(&x, &y, &theta, &vx, &vy, &vtheta);

		// Get current time and create quaternion
		ctime = ros::Time::now();
		oquat = tf::createQuaternionMsgFromYaw(-theta);	

		// Transformation
		this->mtransform_.header.stamp = ctime;
		this->mtransform_.transform.translation.x	= x;
		this->mtransform_.transform.translation.y	= y;
		this->mtransform_.transform.translation.z	= 0.0f;
		this->mtransform_.transform.rotation		= oquat;

		// Odometry message
		this->modometry_.header.stamp = ctime;
		this->modometry_.pose.pose.position.x  = x;
		this->modometry_.pose.pose.position.y  = y;
		this->modometry_.pose.pose.position.z  = 0.0f;
		this->modometry_.pose.pose.orientation = oquat;
		this->modometry_.twist.twist.linear.x  = vx;
		this->modometry_.twist.twist.linear.y  = vy;
		this->modometry_.twist.twist.angular.z = vtheta;
		
	
		// Broadcasting transformation and publishing odometry
		broadcaster.sendTransform(this->mtransform_);
		this->rospub_.publish(this->modometry_);
	}
}


	}
}

#endif
