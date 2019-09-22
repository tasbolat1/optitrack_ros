#include "ros/ros.h"
#include "data_collect/data_collect.h"
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "optitrack/RigidBodyArray.h"
#include "optitrack/SingleMarkerArray.h"


using namespace std;


DataCollector::DataCollector(){
	ros::NodeHandle n("~");
	n.getParam("mode", param);
	ROS_INFO("Got parameter : %s", param.c_str());

	if(param == "train"){
        timestamp_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/timestamp.csv", ios::app);

        // ROS Subscriber initialisation
		time_collect_sub = n.subscribe("optitrack/rigid_bodies", 1000, &DataCollector::timeCollectCB, this);
    }
    else if(param == "test"){
        rigid_body_pose_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_pose.csv", ios::app);
        rigid_body_markers_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_markers.csv", ios::app);
		marker0_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/marker0.csv", ios::app);
		marker1_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/marker1.csv", ios::app);
		marker2_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/marker2.csv", ios::app);

		// ROS Subscriber initialisation
		rigid_data_sub = n.subscribe("optitrack/rigid_bodies", 1000, &DataCollector::dataCollectRigidCB, this);
		markers_data_sub = n.subscribe("optitrack/rigid_bodies", 1000, &DataCollector::dataCollectMarkersCB, this);

    }
    else{
        cout << "Don't run anything !! " << endl;
    }
}

DataCollector::~DataCollector(){
	if (param == "train"){
		timestamp_file.close();
	}
	else{
		rigid_body_pose_file.close();
		rigid_body_markers_file.close();
		marker0_file.close();
		marker1_file.close();
		marker2_file.close();
	}
}

void DataCollector::timeCollectCB(const optitrack::RigidBodyArray::ConstPtr& msg){
	timestamp_file << msg->header.stamp << "\n";
}

void DataCollector::dataCollectRigidCB(const optitrack::RigidBodyArray::ConstPtr& msg){
	rigid_body_pose_file << msg->header.stamp << "," << msg->bodies[0].pose.position.x << msg->bodies[0].pose.position.y << msg->bodies[0].pose.position.z << ",";
	rigid_body_pose_file << msg->bodies[0].pose.orientation.x << msg->bodies[0].pose.orientation.y << msg->bodies[0].pose.orientation.z << msg->bodies[0].pose.orientation.w << "\n";

	rigid_body_markers_file << msg->bodies[0].markers[0].x << "," << msg->bodies[0].markers[0].y << "," << msg->bodies[0].markers[0].z << ",";
	rigid_body_markers_file << msg->bodies[0].markers[1].x << "," << msg->bodies[0].markers[1].y << "," << msg->bodies[0].markers[1].z << ",";
	rigid_body_markers_file << msg->bodies[0].markers[2].x << "," << msg->bodies[0].markers[2].y << "," << msg->bodies[0].markers[2].z << ",";
	rigid_body_markers_file << msg->bodies[0].markers[3].x << "," << msg->bodies[0].markers[3].y << "," << msg->bodies[0].markers[3].z << "\n";
}

void DataCollector::dataCollectMarkersCB(const optitrack::SingleMarkerArray::ConstPtr& msg){
	marker0_file << msg->header.stamp << "," << msg->bodies[0].pose.position.x << "," << msg->bodies[0].pose.position.y << "," << msg->bodies[0].pose.position.z << "\n";
	marker1_file << msg->header.stamp << "," << msg->bodies[1].pose.position.x << "," << msg->bodies[1].pose.position.y << "," << msg->bodies[1].pose.position.z << "\n";
	marker2_file << msg->header.stamp << "," << msg->bodies[2].pose.position.x << "," << msg->bodies[2].pose.position.y << "," << msg->bodies[2].pose.position.z << "\n";
}

int main(int argc, char** argv){
	ros::init(argc, argv, "data_collect");
	
	DataCollector data_collector;

	ros::spin();

	return 0;
}