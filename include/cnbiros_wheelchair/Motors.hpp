#ifndef CNBIROS_WHEELCHAIR_MOTORS_HPP
#define CNBIROS_WHEELCHAIR_MOTORS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_core/NodeInterface.hpp"

#include "DXGPSBThread.hpp"

namespace cnbiros {
	namespace wheelchair {

class Motors : public cnbiros::core::NodeInterface {

	public:
		Motors(ros::NodeHandle* node);
		virtual ~Motors(void);

		int Open(const std::string& port);
	private:
		void on_command_velocity(const geometry_msgs::Twist& msg);

	private:
		ros::Subscriber		rossub_;
		DXGPSBThread*		dxgpsb_;
		std::string			port_;

};



	}
}

#endif
