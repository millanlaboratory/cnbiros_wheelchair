#include <ros/ros.h>
#include "cnbiros_wheelchair/Motors.hpp"

int main (int argc, char** argv) {

	std::string port;

	ros::init(argc, argv, "motors");
	ros::NodeHandle node;

	cnbiros::wheelchair::Motors	motors(&node);
	motors.SetRate(50);

	ros::param::get("~port", port);


	try{
		motors.Open(port);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
		ros::shutdown();
	}
	ROS_INFO("Wheelchair motors connected at port %s", port.c_str());

	motors.Run();


	return 0;
}
