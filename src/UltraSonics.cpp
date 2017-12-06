#ifndef CNBIROS_WHEELCHAIR_ULTRASONICS_CPP
#define CNBIROS_WHEELCHAIR_ULTRASONICS_CPP

#include "cnbiros_wheelchair/UltraSonics.hpp"

namespace cnbiros {
	namespace wheelchair {

UltraSonics::UltraSonics(ros::NodeHandle* node, unsigned int nsonars, std::string name) : 
						 cnbiros::core::NodeInterface(node, name) {

	// Initialize sonars pointer
	this->sonars_ = nullptr;			 
	
	// Initialize addresses
	this->nsonars_  = nsonars;
	for(auto i=0; i<this->nsonars_; i++) 
		this->addresses_.push_back(0xe0 + (i * 2));

	// Initialize ranges
	this->ranges_.resize(this->nsonars_);
	this->ranges_.assign(this->nsonars_, 0);

	// Initialize message
	this->msonar_.radiation_type = 0;
	this->msonar_.field_of_view = CNBIROS_WHEELCHAIR_ULTRASONICS_FIELDOFVIEW; // 30 degrees
	this->msonar_.min_range = CNBIROS_WHEELCHAIR_ULTRASONICS_MINRANGE; 
	this->msonar_.max_range = CNBIROS_WHEELCHAIR_ULTRASONICS_MAXRANGE;

	// Add services
	this->rossrv_setmaxrange_ = node->advertiseService(
								ros::this_node::getName() + "/maxrange", 
								&UltraSonics::on_srv_setmaxrange_, this);
	this->rossrv_setminrange_ = node->advertiseService(
								ros::this_node::getName() + "/minrange", 
								&UltraSonics::on_srv_setminrange_, this);
	this->rossrv_setfieldofview_ =	node->advertiseService(
									ros::this_node::getName() + "/fieldofview", 
									&UltraSonics::on_srv_setfieldofview_, this);
	// Initialize publishers
	this->setup_publishers(node, name);

}

UltraSonics::UltraSonics(ros::NodeHandle* node, const std::vector<unsigned char>& addresses,
						 std::string name) : 
						 cnbiros::core::NodeInterface(node, name) {

	// Initialize sonars pointer
	this->sonars_ = nullptr;			 

	// Initialize addresses
	this->addresses_ = addresses;
	this->nsonars_  = this->addresses_.size();
	
	// Initialize ranges
	this->ranges_.resize(this->nsonars_);
	this->ranges_.assign(this->nsonars_, 0);

	// Initialize message
	this->msonar_.radiation_type = 0;
	this->msonar_.field_of_view = CNBIROS_WHEELCHAIR_ULTRASONICS_FIELDOFVIEW; // 30 degrees
	this->msonar_.min_range = CNBIROS_WHEELCHAIR_ULTRASONICS_MINRANGE; 
	this->msonar_.max_range = CNBIROS_WHEELCHAIR_ULTRASONICS_MAXRANGE;
	
	// Add services
	this->rossrv_setmaxrange_ = node->advertiseService(
								ros::this_node::getName() + "/maxrange", 
								&UltraSonics::on_srv_setmaxrange_, this);
	this->rossrv_setminrange_ = node->advertiseService(
								ros::this_node::getName() + "/minrange", 
								&UltraSonics::on_srv_setminrange_, this);
	this->rossrv_setfieldofview_ =	node->advertiseService(
									ros::this_node::getName() + "/fieldofview", 
									&UltraSonics::on_srv_setfieldofview_, this);

	// Initialize publishers
	this->setup_publishers(node, name);

}

UltraSonics::~UltraSonics(void) {
	if(this->sonars_ != nullptr)
		delete this->sonars_;
}

int UltraSonics::SetFieldOfView(float field) {
	this->viewfield_ = field;
	this->msonar_.field_of_view = this->viewfield_;
	return 0;
}

int UltraSonics::SetMaxRange(float maxrange) {
	this->maxrange_ = maxrange;
	this->msonar_.max_range = this->maxrange_;
	return 0;
}

int UltraSonics::SetMinRange(float minrange) {
	this->minrange_ = minrange;
	this->msonar_.min_range = this->minrange_;
	return 0;
}

bool UltraSonics::on_srv_setmaxrange_(cnbiros_wheelchair::SetMaxRange::Request& req,
						  cnbiros_wheelchair::SetMaxRange::Response& res) {
	bool retcod = false;	

	if(this->SetMaxRange(req.value) == 0) 
		retcod = true;

	res.result = retcod;

	return retcod;
}

bool UltraSonics::on_srv_setminrange_(cnbiros_wheelchair::SetMinRange::Request& req,
						  cnbiros_wheelchair::SetMinRange::Response& res) {
	bool retcod = false;	

	if(this->SetMinRange(req.value) == 0) 
		retcod = true;

	res.result = retcod;

	return retcod;
}

bool UltraSonics::on_srv_setfieldofview_(cnbiros_wheelchair::SetFieldOfView::Request& req,
						  cnbiros_wheelchair::SetFieldOfView::Response& res) {
	bool retcod = false;	

	if(this->SetFieldOfView(req.value) == 0) 
		retcod = true;

	res.result = retcod;

	return retcod;
}

int UltraSonics::Open(const std::string& port) {

	int retcod = 0;
	std::string saddr;

	this->port_ = port;
	try {
		this->sonars_ = new SonarThread(this->port_.c_str(), this->nsonars_, this->addresses_.data());
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
		return -1;
	}

	for(auto it=this->addresses_.begin(); it!=this->addresses_.end(); ++it) {
		std::stringstream ss;
		ss << "0x" << std::hex << (int)(*it);
		saddr += ss.str() + " ";
	}

	ROS_INFO("USB port %s opened. %d ultrasound sensors registered: %s", 
			 this->port_.c_str(), this->nsonars_, saddr.c_str());

	return retcod;

}

int UltraSonics::GetRanges(std::vector<int>& ranges) {

	int retcode = -1;
	int cranges[this->nsonars_];

	if(this->sonars_ != nullptr) {
		retcode = this->sonars_->getReadings(cranges);
	}

	if (retcode != -1)
		ranges.assign(cranges, cranges+this->nsonars_);	


	return retcode;
}

int UltraSonics::SetGain(unsigned char gain, unsigned char address) {
	int retcode = -1;
	std::stringstream ssa, ssg;
	
	if(this->sonars_ !=nullptr) {
		retcode = this->sonars_->setAnalogueGain(address, gain);
	}
	
	ssa << "0x" << std::hex << (int)address;
	ssg << "0x" << std::hex << (int)gain;

	if(retcode != -1) {
		ROS_INFO("Gain set at %s for sensor %s", ssg.str().c_str(), ssa.str().c_str());
	} else {
		ROS_WARN("Cannot set the gain for sensor %s", ssa.str().c_str());
	}

	return retcode;
}

int UltraSonics::SetGain(unsigned char gain) {

	for(auto it=this->addresses_.begin(); it!=this->addresses_.end(); ++it) {
		this->SetGain(gain, (*it));
	}
	
	return 0;
}

void UltraSonics::setup_publishers(ros::NodeHandle* node, std::string basename) {


	for(auto it=this->addresses_.begin(); it!=this->addresses_.end(); ++it) {
		std::stringstream ss;
		ss << "0x" << std::hex << (int)(*it);
		ros::Publisher pub = node->advertise<sensor_msgs::Range>(basename + "/" + ss.str(), 1);
		this->rospubs_.push_back(pub);
	}
}

void UltraSonics::onRunning(void) {

	unsigned int sonarId = 0;
	int crange;


	if(this->GetRanges(this->ranges_) != -1) {
		
		for(auto it=this->rospubs_.begin(); it!=this->rospubs_.end(); ++it) {
			std::stringstream ss;
			ss << "sonar_0x" << std::hex << (int)this->addresses_.at(sonarId) << "_link";
			this->msonar_.header.frame_id = ss.str();
			this->msonar_.header.stamp = ros::Time::now();
			this->msonar_.range = (float)(this->ranges_.at(sonarId)/100.0f);

			(*it).publish(this->msonar_);
			sonarId++;
		}

	}

}



	}
}

#endif

