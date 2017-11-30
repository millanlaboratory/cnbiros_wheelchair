#include <ros/ros.h>
#include "cnbiros_wheelchair/Odometry.hpp"

#define CNBIROS_WHEELCHAIR_WHEEL_AXLE		0.5240f	// [m] distance between wheels
#define CNBIROS_WHEELCHAIR_WHEEL_DIAMETER	0.3556f	// [m]
#define CNBIROS_WHEELCHAIR_WHEEL_REVOLUTION	280		// Number of counted tick for a wheel revolution

int main (int argc, char** argv) {

	std::string lport, rport;
	double axle		= CNBIROS_WHEELCHAIR_WHEEL_AXLE;
	double diameter = CNBIROS_WHEELCHAIR_WHEEL_DIAMETER;
	int revolution  = CNBIROS_WHEELCHAIR_WHEEL_REVOLUTION;

	ros::init(argc, argv, "odometry");
	ros::NodeHandle node;

	ros::param::get("~leftport",  lport);
	ros::param::get("~rightport", rport);
	ros::param::get("~axle", axle);
	ros::param::get("~diameter", diameter);
	ros::param::get("~revolution", revolution);

	cnbiros::wheelchair::Odometry odometry(&node, axle, diameter, revolution);
	odometry.SetRate(50);

	try{
		odometry.Open(lport, rport);
	} catch (std::runtime_error& e) {
		ROS_ERROR("%s", e.what());
		ros::shutdown();
	}
	ROS_INFO("Wheelchair odometry connected at ports %s and %s", 
			  lport.c_str(), rport.c_str());

	odometry.Run();

	ros::shutdown();

	return 0;
}
