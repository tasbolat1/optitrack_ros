#ifndef DATA_COLLECT_H
#define DATA_COLLECT_H

#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include "std_msgs/String.h"
#include "optitrack/RigidBodyArray.h"
#include "optitrack/SingleMarkerArray.h"


using namespace std;

class DataCollector{
	public:
	DataCollector();
	~DataCollector();
	void startTimeCB(const std_msgs::String::ConstPtr& msg);
	void timeCollectCB(const optitrack::RigidBodyArray::ConstPtr& msg);
	void dataCollectRigidCB(const optitrack::RigidBodyArray::ConstPtr& msg);
	void dataCollectMarkersCB(const optitrack::SingleMarkerArray::ConstPtr& msg);

	private:
	std::string param;

	// ROS Subscribers
	ros::Subscriber rigid_data_sub;
	ros::Subscriber markers_data_sub;
	ros::Subscriber time_collect_sub;
	ros::Subscriber start_timer_sub;

	// Streaming Files
	ofstream timestamp_file;
	ofstream rigid_body_pose_file;
	ofstream rigid_body_markers_file;
	ofstream marker0_file;
	ofstream marker1_file;
	ofstream marker2_file;

};

#endif