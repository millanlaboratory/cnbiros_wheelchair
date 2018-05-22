#ifndef CNBIROS_WHEELCHAIR_MOTORS_HPP
#define CNBIROS_WHEELCHAIR_MOTORS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "cnbiros_wheelchair/Stop.h"
#include "cnbiros_wheelchair/Forward.h"
#include "cnbiros_wheelchair/SetVelocity.h"
#include "cnbiros_wheelchair/GetVelocity.h"

#include "DxgpsbThread.hpp"


namespace cnbiros {
	namespace wheelchair {

class Motors {

	public:
		Motors(void);
		virtual ~Motors(void);

		int Open(void);
		int Open(const std::string& port);

		int SetVelocity(float v, float w);
		int GetVelocity(float& v, float& w);
		int Stop(void);
		int Forward(float v = 1.0f);

	private:
		virtual bool configure(void);
		
		float normalize_velocity(float v, float max_v, float min_v);
		float limit_velocity(float v, float max_v = 1.0f, float min_v = -1.0f);


		void on_command_velocity(const geometry_msgs::Twist& msg);
		bool on_srv_stop_(cnbiros_wheelchair::Stop::Request& req,
						  cnbiros_wheelchair::Stop::Response& res);
		bool on_srv_forward_(cnbiros_wheelchair::Forward::Request& req,
							 cnbiros_wheelchair::Forward::Response& res);
		bool on_srv_setvelocity_(cnbiros_wheelchair::SetVelocity::Request& req,
								 cnbiros_wheelchair::SetVelocity::Response& res);
		bool on_srv_getvelocity_(cnbiros_wheelchair::GetVelocity::Request& req,
								 cnbiros_wheelchair::GetVelocity::Response& res);

	private:
		ros::NodeHandle		nh_;
		ros::NodeHandle		private_nh_;
		ros::Subscriber		sub_;
		DxgpsbThread*		dxgpsb_;
		std::string			port_;
		float				max_vel_lin_;
		float				min_vel_lin_;
		float				max_vel_th_;
		float				min_vel_th_;


		ros::ServiceServer	srv_stop_;
		ros::ServiceServer	srv_forward_;
		ros::ServiceServer	srv_setvelocity_;
		ros::ServiceServer	srv_getvelocity_;
		

};



	}
}

#endif
