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


void Motors::on_srv_stop_(cnbiros_wheelchair::Stop::Request& req,
						  cnbiros_wheelchair::Stop::Response& res) {

	this->SetVelocity(0.0f, 0.0f);

	// Add result depending on SetVelocity output
	res.result = true;
}

void Motors::on_srv_forward_(cnbiros_wheelchair::Forward::Request& req,
						     cnbiros_wheelchair::Forward::Response& res) {

	this->SetVelocity(CNBIROS_WHEELCHAIR_DEFAULT_VELOCITY, 0.0f);

	// Add result depending on SetVelocity output
	res.result = true;
}

void Motors::on_srv_setvelocity_(cnbiros_wheelchair::SetVelocity::Request& req,
								 cnbiros_wheelchair::SetVelocity::Response& res) {

	this->SetVelocity(req.v, req.w);

	// Add result depending on SetVelocity output
	res.result = true;
}

void Motors::on_srv_setvelocity_(cnbiros_wheelchair::SetVelocity::Request& req,
								 cnbiros_wheelchair::SetVelocity::Response& res) {

	float v, w;
	this->GetVelocity(&v, &w);

	// Add result depending on GetVelocity output
	res.result.v = v;
	res.result.w = w;
}

	}
}


#endif
