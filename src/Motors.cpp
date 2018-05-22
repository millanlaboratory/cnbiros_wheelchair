#ifndef CNBIROS_WHEELCHAIR_MOTORS_CPP
#define CNBIROS_WHEELCHAIR_MOTORS_CPP

#include "cnbiros_wheelchair/Motors.hpp"

namespace cnbiros {
	namespace wheelchair {

Motors::Motors(void) : private_nh_("~") {

	// Initialize dxgpsb pointer
	this->dxgpsb_ = nullptr;

	// Configure
	this->configure();


	// Initialize subscriber
	this->sub_ = this->nh_.subscribe("cmd_vel", 1, &Motors::on_command_velocity, this);

	// Add services
	this->srv_stop_		   = this->private_nh_.advertiseService("stop", &Motors::on_srv_stop_, this);
	this->srv_forward_	   = this->private_nh_.advertiseService("forward", &Motors::on_srv_forward_, this);
	this->srv_setvelocity_ = this->private_nh_.advertiseService("setvelocity", &Motors::on_srv_setvelocity_, this);
	this->srv_getvelocity_ = this->private_nh_.advertiseService("getvelocity", &Motors::on_srv_getvelocity_, this);
}

Motors::~Motors(void){
	if(this->dxgpsb_ != nullptr)
		delete this->dxgpsb_;
}

bool Motors::configure(void) {
	this->private_nh_.param<std::string>("port", this->port_, "/dev/controller");
	this->private_nh_.param<float>("max_vel_lin", this->max_vel_lin_, 0.290f);
	this->private_nh_.param<float>("min_vel_lin", this->min_vel_lin_, -0.290f);
	this->private_nh_.param<float>("max_vel_th", this->max_vel_th_, 0.300f);
	this->private_nh_.param<float>("min_vel_th", this->min_vel_th_, -0.300f);

	ROS_INFO("Motors controller port: %s", this->port_.c_str());
	ROS_INFO("Motors maximum linear  velocity: %f", this->max_vel_lin_);
	ROS_INFO("Motors minimum linear  velocity: %f", this->min_vel_lin_);
	ROS_INFO("Motors maximum angular velocity: %f", this->max_vel_th_);
	ROS_INFO("Motors minimum angular velocity: %f", this->min_vel_th_);

	return true;

}

int Motors::Open(void) {
	return this->Open(this->port_);
}

int Motors::Open(const std::string& port) {

	int retcod = 0;

	this->port_ = port;
	try {
		this->dxgpsb_ = new DxgpsbThread(this->port_);
		ROS_INFO("Wheelchair motors connected at port %s", port.c_str());
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
		return -1;
	}

	this->Stop();
	return retcod;
}

int Motors::SetVelocity(float v, float w) {
	int ret = -1;
	float nv, nw;
	float lv, lw;
	if(this->dxgpsb_ != nullptr) {

		nv = this->normalize_velocity(v, this->max_vel_lin_, this->min_vel_lin_);	
		nw = this->normalize_velocity(w, this->max_vel_th_, this->min_vel_th_);	

		lv = this->limit_velocity(nv, 1.0f, -1.0f);
		lw = this->limit_velocity(nw, 1.0f, -1.0f);
	
		// Change sign for angular velocity
		lw = -lw;

		ret = this->dxgpsb_->setVelocities(lv, lw);

		ROS_DEBUG_NAMED("velocity_debug", "Actual velocities:     (%f, %f) [m/s]", v, w);
		ROS_DEBUG_NAMED("velocity_debug", "Normalized velocities: (%f, %f) [-1, 1]", lv, lw);
	}

	return ret;
}

float Motors::normalize_velocity(float v, float max_v, float min_v) {

	return 2.0f*((v - min_v)/(max_v - min_v)) - 1.0f;
}

float Motors::limit_velocity(float v, float max_v, float min_v) {
	float lv;

	if(v > max_v)
		lv = max_v;
	else if(v < min_v)
		lv = min_v;
	else
		lv = v;

	return lv;
}

int Motors::GetVelocity(float& v, float& w) {

	int ret = -1;
	
	if(this->dxgpsb_ != nullptr)
		ret = this->dxgpsb_->getVelocities(v, w);

	return ret;
}

int Motors::Stop(void) {
	return this->SetVelocity(0.0f, 0.0f);
}

int Motors::Forward(float v) {
	return this->SetVelocity(v, 0.0f);
}

void Motors::on_command_velocity(const geometry_msgs::Twist& msg) {

	float v, w;
	v = msg.linear.x;
	w = msg.angular.z;
	this->SetVelocity(v, w);
}


bool Motors::on_srv_stop_(cnbiros_wheelchair::Stop::Request& req,
						  cnbiros_wheelchair::Stop::Response& res) {

	bool retcode = false;
	
	if(this->Stop() == 0)
		retcode = true;

	res.result = retcode;
	return retcode;
}

bool Motors::on_srv_forward_(cnbiros_wheelchair::Forward::Request& req,
						     cnbiros_wheelchair::Forward::Response& res) {

	bool retcode = false;
	
	if(this->Forward() == 0)
		retcode = true;

	res.result = retcode;
	return retcode;
}

bool Motors::on_srv_setvelocity_(cnbiros_wheelchair::SetVelocity::Request& req,
								 cnbiros_wheelchair::SetVelocity::Response& res) {

	bool retcode = false;

	if(this->SetVelocity(req.v, req.w) == 0) 
		retcode = true;

	res.result = retcode;
	return retcode;
}

bool Motors::on_srv_getvelocity_(cnbiros_wheelchair::GetVelocity::Request& req,
								 cnbiros_wheelchair::GetVelocity::Response& res) {

	float v = 0.0f;
	float w = 0.0f;
	bool retcode = false;
	
	if(this->GetVelocity(v, w) == 0)
		retcode = true;

	res.result = retcode;
	res.v = v;
	res.w = w;
	
	return retcode;
}

	}
}


#endif
