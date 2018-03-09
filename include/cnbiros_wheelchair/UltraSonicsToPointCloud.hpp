#ifndef CNBIROS_WHEELCHAIR_ULTRASONICSTOPOINTCLOUD_HPP
#define CNBIROS_WHEELCHAIR_ULTRASONICSTOPOINTCLOUD_HPP

// ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

namespace cnbiros {
	namespace wheelchair {

class UltraSonicsToPointCloud {

	public:
		UltraSonicsToPointCloud(void);
		virtual ~UltraSonicsToPointCloud(void);

		virtual bool configure(void);

		virtual void Run(void);

	private:
		void on_received_ultrasonic(const sensor_msgs::Range& msg);

	protected:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		std::vector<ros::Subscriber>	subs_;
		ros::Publisher					pub_;
		std::string						ptopic_;
		std::vector<std::string>		stopics_;

		std::string		frame_id_;

		sensor_msgs::PointCloud						pointcloud_;
		std::vector<geometry_msgs::PointStamped>	readings_;

};


	}
}


#endif
