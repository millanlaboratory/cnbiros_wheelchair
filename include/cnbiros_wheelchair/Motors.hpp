#ifndef CNBIROS_WHEELCHAIR_MOTORS_HPP
#define CNBIROS_WHEELCHAIR_MOTORS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_wheelchair/Stop.h"
#include "cnbiros_wheelchair/Forward.h"
#include "cnbiros_wheelchair/SetVelocity.h"
#include "cnbiros_wheelchair/GetVelocity.h"

#include "DXGPSBThread.hpp"

#define CNBIROS_WHEELCHAIR_DEFAULT_VELOCITY		0.5f

namespace cnbiros {
	namespace wheelchair {

class Motors : public cnbiros::core::NodeInterface {

	public:
		Motors(ros::NodeHandle* node);
		virtual ~Motors(void);

		int Open(const std::string& port);

		int SetVelocity(float v, float w);
		int GetVelocity(float* v, float* w);

	private:
		void on_command_velocity(const geometry_msgs::Twist& msg);
		void on_srv_stop_(cnbiros_wheelchair::Stop::Request& req,
						  cnbiros_wheelchair::Stop::Response& res);
		void on_srv_forward_(cnbiros_wheelchair::Forward::Request& req,
						     cnbiros_wheelchair::Forward::Response& res);
		void on_srv_setvelocity_(cnbiros_wheelchair::SetVelocity::Request& req,
								 cnbiros_wheelchair::SetVelocity::Response& res);
		void on_srv_getvelocity_(cnbiros_wheelchair::GetVelocity::Request& req,
								 cnbiros_wheelchair::GetVelocity::Response& res);

	private:
		ros::Subscriber		rossub_;
		DXGPSBThread*		dxgpsb_;
		std::string			port_;

		ros::ServiceServer	rossrv_stop_;
		ros::ServiceServer	rossrv_forward_;
		ros::ServiceServer	rossrv_setvelocity_;
		ros::ServiceServer	rossrv_getvelocity_;
		

};



	}
}

#endif
