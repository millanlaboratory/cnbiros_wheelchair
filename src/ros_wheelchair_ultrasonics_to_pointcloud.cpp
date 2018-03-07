#include <ros/ros.h>
#include "cnbiros_wheelchair/UltraSonicsToPointCloud.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "sonar_to_pointcloud");
	
	cnbiros::wheelchair::UltraSonicsToPointCloud u;

	u.Run();

	
	return 0;
}
