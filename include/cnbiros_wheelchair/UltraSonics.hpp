#ifndef CNBIROS_WHEELCHAIR_ULTRASONICS_HPP
#define CNBIROS_WHEELCHAIR_ULTRASONICS_HPP

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

#include "cnbiros_core/NodeInterface.hpp"
#include "cnbiros_wheelchair/SonarThread.hpp"


namespace cnbiros {
	namespace wheelchair {

class UltraSonics : public cnbiros::core::NodeInterface {

	public:
		UltraSonics(ros::NodeHandle* node, unsigned int nsonars, std::string name = "sonars");
		UltraSonics(ros::NodeHandle* node, const std::vector<unsigned char>& addresses, 
					std::string name = "sonars");
		virtual ~UltraSonics(void);

		int Open(const std::string& port);

		int SetGain(const std::string& gain, const std::string& address);
		int GetRanges(std::vector<int>& ranges);

		void onRunning(void);

	protected:
		void setup_publishers(ros::NodeHandle* node, std::string basename);

	private:
		std::string						port_;
		unsigned int					nsonars_;
		std::vector<unsigned char>		addresses_;
		std::vector<ros::Publisher>		rospubs_;
		SonarThread*					sonars_;
		std::vector<int>				ranges_;
		sensor_msgs::Range				msonar_;
};

	}
}

#endif
