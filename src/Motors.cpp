#ifndef CNBIROS_WHEELCHAIR_MOTORS_CPP
#define CNBIROS_WHEELCHAIR_MOTORS_CPP

#include "cnbiros_wheelchair/Motors.hpp"

namespace cnbiros {
	namespace wheelchair {

Motors::Motors(ros::NodeHandle* node) : 
			   cnbiros::core::NodeInterface(node, "motors") {

	// Initialize dxgpsb pointer
	this->dxgpsb_ = nullptr;
	
	// Initialize subscriber
	this->rossub_ = node->subscribe("cmd_vel", 1, &Motors::on_command_velocity, this);

	// Add services
	this->rossrv_stop_ = node->advertiseService(
						 ros::this_node::getName() + "/stop", 
						 &Motors::on_srv_stop_, this);
	
	this->rossrv_forward_ = node->advertiseService(
							ros::this_node::getName() + "/forward", 
							&Motors::on_srv_forward_, this);
	
	this->rossrv_setvelocity_ = node->advertiseService(
								ros::this_node::getName() + "/setvelocity", 
								&Motors::on_srv_setvelocity_, this);
	
	this->rossrv_getvelocity_ = node->advertiseService(
								ros::this_node::getName() + "/getvelocity", 
								&Motors::on_srv_getvelocity_, this);
}

Motors::~Motors(void){
	if(this->dxgpsb_ != nullptr)
		delete this->dxgpsb_;
}

int Motors::Open(const std::string& port) {

	int retcod = 0;

	this->port_ = port;
	try {
		this->dxgpsb_ = new DXGPSBThread(this->port_);
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}

	this->Stop();
	return retcod;
}

int Motors::SetVelocity(float v, float w) {
	int ret = -1;
	if(this->dxgpsb_ != nullptr) {
		if (v > CNBIROS_WHEELCHAIR_VELOCITY_LINEAR_MAX) {
			ROS_WARN("Maximum linear velocity %f. Provided velocity: %f", 
					 CNBIROS_WHEELCHAIR_VELOCITY_LINEAR_MAX, v);
		} else if(v < CNBIROS_WHEELCHAIR_VELOCITY_LINEAR_MIN) {
			ROS_WARN("Minimum linear velocity %f. Provided velocity: %f", 
					 CNBIROS_WHEELCHAIR_VELOCITY_LINEAR_MIN, v);
		} else if(w > CNBIROS_WHEELCHAIR_VELOCITY_ROTATION_MAX) {
			ROS_WARN("Maximum rotation velocity %f. Provided velocity: %f", 
					 CNBIROS_WHEELCHAIR_VELOCITY_ROTATION_MAX, w);
		} else if(w < CNBIROS_WHEELCHAIR_VELOCITY_ROTATION_MIN) {
			ROS_WARN("Minimum rotation velocity %f. Provided velocity: %f", 
					 CNBIROS_WHEELCHAIR_VELOCITY_ROTATION_MIN, w);
		} else {
			ret = this->dxgpsb_->setVelocities(v, w);
		}
	}

	return ret;
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
