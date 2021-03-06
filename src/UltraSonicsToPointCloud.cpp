#ifndef CNBIROS_WHEELCHAIR_ULTRASONICSTOPOINTCLOUD_CPP
#define CNBIROS_WHEELCHAIR_ULTRASONICSTOPOINTCLOUD_CPP

#include "cnbiros_wheelchair/UltraSonicsToPointCloud.hpp"

namespace cnbiros {
	namespace wheelchair {

UltraSonicsToPointCloud::UltraSonicsToPointCloud(void) : private_nh_("~"), listener_(ros::Duration(10)) {

	// Configure node
	this->configure();

	// Initialize subscribers
	for(auto it=this->stopics_.begin(); it!=this->stopics_.end(); ++it) {
		this->subs_.emplace_back(this->nh_.subscribe((*it), 100, &UltraSonicsToPointCloud::on_received_ultrasonic, this));
	}

	// Initialize publisher
	this->pub_ = this->nh_.advertise<sensor_msgs::PointCloud>(this->ptopic_, 100);


};

UltraSonicsToPointCloud::~UltraSonicsToPointCloud(void) {}

bool UltraSonicsToPointCloud::configure(void) {

	// Getting parameters
	this->private_nh_.param<std::string>("frame_id", this->frame_id_, "base_link");
	this->private_nh_.param<std::string>("pointcloud", this->ptopic_, "/sonar_pointcloud");
	this->private_nh_.getParam("topics", this->stopics_);

	// Dumping parameters
	ROS_INFO("UltraSonicsToPointCloud frame_id:				%s", this->frame_id_.c_str());
	ROS_INFO("UltraSonicsToPointCloud pointcloud topic:		%s", this->ptopic_.c_str());
	for(auto it=this->stopics_.begin(); it!=this->stopics_.end(); ++it)
		ROS_INFO("UltraSonicsToPointCloud subscribed topic:	%s", (*it).c_str());

	this->pointcloud_.points.clear();
	
	return true;
}

void UltraSonicsToPointCloud::on_received_ultrasonic(const sensor_msgs::Range& msg) {

	geometry_msgs::PointStamped	range_point;

	// Fill the local range point with the message values
	range_point.header.frame_id	= msg.header.frame_id;
	range_point.header.stamp	= msg.header.stamp;
	range_point.point.x			= msg.range;
	range_point.point.y			= 0.0f;
	range_point.point.z			= 0.0f;

	// Try to transform the range point into cloud point
	//try {
	//	printf("\tWait for transform...");
	//	listener.waitForTransform(this->frame_id_, msg.header.frame_id, 
	//							  ros::Time(0), ros::Duration(10.0));
	//	printf("..tranform ready\n");
	//	listener.transformPoint(this->frame_id_, range_point, cloud_point);
	//} catch (tf::TransformException &ex) {
	//	ROS_ERROR("%s", ex.what());
	//}

	//current_point.x = cloud_point.point.x;
	//current_point.y = cloud_point.point.y;
	//current_point.z = cloud_point.point.z;

	// Store the cloud point in readings_
	this->readings_.emplace_back(range_point);
	
}

void UltraSonicsToPointCloud::Run(void) {
	ros::Rate rate(20);

	geometry_msgs::PointStamped	cloud_point;
	geometry_msgs::Point32		simple_point;
	std::string errorstr;

	while(this->nh_.ok()) {

		if(this->readings_.empty() == false) {

			// For each point in the readings vector, make a transformation on
			// the point cloud frame and add to the point cloud
			for(auto it=this->readings_.begin(); it!= this->readings_.end(); ++it) {
			
				// Transform the current reading point (sensor frame) to the
				// pointcloud frame
				this->listener_.waitForTransform(this->frame_id_, (*it).header.frame_id, 
											  ros::Time::now() - ros::Duration(0.1), ros::Duration(10.0), ros::Duration(0.0001), &errorstr);
				try {
					this->listener_.transformPoint(this->frame_id_, (*it), cloud_point);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s", ex.what());
					continue;
				}

				// Convert the current Point in the cloud_point (PointStamped)
				// into a Point32 (for the pointcloud)
				simple_point.x = cloud_point.point.x;
				simple_point.y = cloud_point.point.y;
				simple_point.z = cloud_point.point.z;


				// Add the current cloud point to the pointcloud
				this->pointcloud_.points.emplace_back(simple_point);
			}


			// Update the header and timestamp
			this->pointcloud_.header.frame_id = "base_link";
			this->pointcloud_.header.stamp	  = ros::Time::now();

			// Publish the current pointcloud
			this->pub_.publish(this->pointcloud_);

			// Reset the readings and the Pointcloud
			this->readings_.clear();
			this->pointcloud_.points.clear();
		}
		
		rate.sleep();
		ros::spinOnce();
	}

}

	}
}

#endif
