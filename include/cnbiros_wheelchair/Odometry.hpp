#ifndef CNBIROS_WHEELCHAIR_ODOMETRY_HPP
#define CNBIROS_WHEELCHAIR_ODOMETRY_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_wheelchair/OdometryThread.hpp"

namespace cnbiros {
	namespace wheelchair {

class Odometry : public cnbiros::core::NodeInterface {

	public:
		Odometry(ros::NodeHandle* node, 
				 double axle, double diameter, int revolution, 
				 std::string name="odometry");
		virtual ~Odometry(void);

		int Open(const std::string& lport, const std::string& rport);

		int Reset(void);
		int Set(double x, double y, double theta);
		int Get(double *x, double *y, double *theta,
				double *vx = nullptr, double *vy = nullptr,
				double *vtheta = nullptr);

		void onRunning(void);

	private:
		std::string			lport_;
		std::string			rport_;
		double				axle_;
		double				diameter_;
		int					revolution_;
		OdometryThread*		odometry_;

		ros::Publisher		rospub_;
		nav_msgs::Odometry	modometry_;
		geometry_msgs::TransformStamped mtransform_;
};

	}
}
#endif

