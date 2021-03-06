#include <ros/ros.h>
#include "cnbiros_wheelchair/UltraSonics.hpp"

int main (int argc, char** argv) {

	std::string port;
	unsigned char taddr;
	std::vector<int> iaddr;
	std::vector<unsigned char> addr;
	float viewfield, maxrange, minrange;

	ros::init(argc, argv, "sonars");
	ros::NodeHandle node;

	ros::param::get("~port", port);
	ros::param::get("~addresses", iaddr);


	for(auto it=iaddr.begin(); it!=iaddr.end(); ++it) {
		addr.push_back(static_cast<unsigned char>((*it)));
	}

	cnbiros::wheelchair::UltraSonics sonars(&node, addr);
	sonars.SetRate(20);
	
	if(ros::param::get("~fieldofview", viewfield)) {
		sonars.SetFieldOfView(viewfield);
		ROS_INFO("Sonars field of view set at %f", viewfield);
	}
	if(ros::param::get("~maxrange", maxrange)) {
		sonars.SetMaxRange(maxrange);
		ROS_INFO("Sonars max set at %f", maxrange);
	}
	
	if(ros::param::get("~minrange", minrange)) {
		sonars.SetMinRange(minrange);
		ROS_INFO("Sonars max set at %f", minrange);
	}

	try{
		sonars.Open(port);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
		ros::shutdown();
	}
	ROS_INFO("Wheelchair sonars connected at port %s", port.c_str());

	sonars.Run();

	ros::shutdown();

	return 0;
}
