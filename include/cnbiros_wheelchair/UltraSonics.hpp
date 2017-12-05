#ifndef CNBIROS_WHEELCHAIR_ULTRASONICS_HPP
#define CNBIROS_WHEELCHAIR_ULTRASONICS_HPP

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_wheelchair/SonarThread.hpp"
#include "cnbiros_wheelchair/SetMaxRange.h"
#include "cnbiros_wheelchair/SetMinRange.h"
#include "cnbiros_wheelchair/SetFieldOfView.h"

#define CNBIROS_WHEELCHAIR_ULTRASONICS_FIELDOFVIEW	0.52f
#define CNBIROS_WHEELCHAIR_ULTRASONICS_MAXRANGE		0.80f
#define CNBIROS_WHEELCHAIR_ULTRASONICS_MINRANGE		0.06f

namespace cnbiros {
	namespace wheelchair {

class UltraSonics : public cnbiros::core::NodeInterface {

	public:
		UltraSonics(ros::NodeHandle* node, unsigned int nsonars, std::string name = "sonars");
		UltraSonics(ros::NodeHandle* node, const std::vector<unsigned char>& addresses, 
					std::string name = "sonars");
		virtual ~UltraSonics(void);

		int Open(const std::string& port);

		int SetFieldOfView(float field);
		int SetMaxRange(float maxrange);
		int SetMinRange(float minrange);
		int SetGain(unsigned char gain, unsigned char address);
		int SetGain(unsigned char gain);
		int GetRanges(std::vector<int>& ranges);

		void onRunning(void);
	
	protected:
		void setup_publishers(ros::NodeHandle* node, std::string basename);
	private:

		bool on_srv_setmaxrange_(cnbiros_wheelchair::SetMaxRange::Request& req,
						  cnbiros_wheelchair::SetMaxRange::Response& res);
		bool on_srv_setminrange_(cnbiros_wheelchair::SetMinRange::Request& req,
						  cnbiros_wheelchair::SetMinRange::Response& res);
		bool on_srv_setfieldofview_(cnbiros_wheelchair::SetFieldOfView::Request& req,
						  cnbiros_wheelchair::SetFieldOfView::Response& res);
	private:
		std::string						port_;
		unsigned int					nsonars_;
		std::vector<unsigned char>		addresses_;
		std::vector<ros::Publisher>		rospubs_;
		SonarThread*					sonars_;
		std::vector<int>				ranges_;
		sensor_msgs::Range				msonar_;
		float							maxrange_;
		float							minrange_;
		float							viewfield_;

		ros::ServiceServer	rossrv_setmaxrange_;
		ros::ServiceServer	rossrv_setminrange_;
		ros::ServiceServer	rossrv_setfieldofview_;
};

	}
}

#endif
