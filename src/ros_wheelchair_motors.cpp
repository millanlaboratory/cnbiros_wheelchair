#include <ros/ros.h>
#include "cnbiros_wheelchair/Motors.hpp"

int main (int argc, char** argv) {

	ros::init(argc, argv, "motors");

	cnbiros::wheelchair::Motors	motors;

	try{
		motors.Open();
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
		ros::shutdown();
	}

	ros::spin();

	return 0;
}
