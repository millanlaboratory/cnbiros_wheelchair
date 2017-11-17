#ifndef CNBIROS_WHEELCHAIR_MOTORS_CPP
#define CNBIROS_WHEELCHAIR_MOTORS_CPP

#include "cnbiros_wheelchair/Motors.hpp"

namespace cnbiros {
	namespace wheelchair {

Motors::Motors(ros::NodeHandle* node) : 
			   cnbiros::core::NodeInterface(node, "motors") {

	this->rossub_ = node->subscribe("cmd_vel", 1, &Motors::on_command_velocity, this);
	this->dxgpsb_ = nullptr;
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

	// to initialize (to be done better)
	this->dxgpsb_->setVelocities(0.0f, 0.0f);
	return retcod;
}

void Motors::on_command_velocity(const geometry_msgs::Twist& msg) {

	float v, w;
	v = msg.linear.x;
	w = msg.angular.z;
	this->dxgpsb_->setVelocities(v, w);
}


	}
}


#endif
