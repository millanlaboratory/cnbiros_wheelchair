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
	this->msonar_.field_of_view = 0.52; // +/- 30 degrees
	this->msonar_.min_range = 0.06f;
	this->msonar_.max_range = 2.00f;
	

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
	
	// Initialize publishers
	this->setup_publishers(node, name);

}

UltraSonics::~UltraSonics(void) {
	if(this->sonars_ != nullptr)
		delete this->sonars_;
}

int UltraSonics::Open(const std::string& port) {

	int retcod = 0;

	this->port_ = port;
	try {
		this->sonars_ = new SonarThread(this->port_.c_str(), this->nsonars_, this->addresses_.data());
	} catch (std::runtime_error& e) {
		throw std::runtime_error(e.what());
	}

	return retcod;

}

int UltraSonics::GetRanges(std::vector<int>& ranges) {

	int retcode;
	int cranges[this->nsonars_];

	if(this->sonars_ != nullptr) {
		retcode = this->sonars_->getReadings(cranges);
	}

	if (retcode != -1)
		ranges.assign(cranges, cranges+this->nsonars_);	


	return retcode;
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

	this->msonar_.header.stamp = ros::Time::now();

	if(this->GetRanges(this->ranges_) != -1) {
		
		for(auto it=this->rospubs_.begin(); it!=this->rospubs_.end(); ++it) {
			std::stringstream ss;
			ss << "sonar_0x" << std::hex << (int)this->addresses_.at(sonarId) << "_link";
			this->msonar_.header.frame_id = ss.str();
			this->msonar_.range = (float)(this->ranges_.at(sonarId)/100.0f);

			(*it).publish(this->msonar_);
			sonarId++;
		}

	}

}



	}
}

#endif

