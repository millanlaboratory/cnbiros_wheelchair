#include <ros/ros.h>
#include "cnbiros_wheelchair/UltraSonics.hpp"

int main (int argc, char** argv) {

	std::string port;

	ros::init(argc, argv, "sonars");
	ros::NodeHandle node;

	cnbiros::wheelchair::UltraSonics sonars(&node, 2);
	//cnbiros::wheelchair::UltraSonics sonars(&node, {0xe0, 0xf2});
	sonars.SetRate(50);

	ros::param::get("~port", port);


	try{
		sonars.Open(port);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
		ros::shutdown();
	}
	ROS_INFO("Wheelchair sonars connected at port %s", port.c_str());

	sonars.Run();


	return 0;
}
