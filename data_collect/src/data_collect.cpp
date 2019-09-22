#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "optitrack/RigidBodyArray.h"

using namespace std;

ofstream timestamp_file;

ofstream rigid_body0_file;
ofstream rigid_body1_file;
ofstream rigid_body2_file;
ofstream rigid_body3_file;


void timeCollectCB(const optitrack::RigidBodyArray::ConstPtr& msg){
	timestamp_file << msg->header.stamp << "\n";
}

void dataCollectCB(const optitrack::RigidBodyArray::ConstPtr& msg){
	rigid_body0_file << msg->header.stamp << "," << msg->bodies[0].markers[0].x << "," << msg->bodies[0].markers[0].y << "," << msg->bodies[0].markers[0].z << "\n";
	rigid_body1_file << msg->header.stamp << "," << msg->bodies[0].markers[1].x << "," << msg->bodies[0].markers[1].y << "," << msg->bodies[0].markers[1].z << "\n";
	rigid_body2_file << msg->header.stamp << "," << msg->bodies[0].markers[2].x << "," << msg->bodies[0].markers[2].y << "," << msg->bodies[0].markers[2].z << "\n";
	rigid_body3_file << msg->header.stamp << "," << msg->bodies[0].markers[3].x << "," << msg->bodies[0].markers[3].y << "," << msg->bodies[0].markers[3].z << "\n";
}

int main(int argc, char** argv){
	std::string param;
	ros::init(argc, argv, "data_collect");
	ros::NodeHandle n("~");
	n.getParam("mode", param);
	ROS_INFO("Got parameter : %s", param.c_str());


	if(param == "train"){
        cout << "train" << endl;
        timestamp_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/timestamp.csv", ios::app);

        // ROS Subscriber initialisation
		ros::Subscriber sub = n.subscribe("optitrack/rigid_bodies", 1000, timeCollectCB);
    }
    else if(param == "test"){
    	cout << "test" << endl;
        rigid_body0_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_marker1.csv", ios::app);
		rigid_body1_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_marker2.csv", ios::app);
		rigid_body2_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_marker3.csv", ios::app);
		rigid_body3_file.open("/home/crslab/catkin_ws/src/optitrack_ros/data_collect/rigid_body_marker4.csv", ios::app);

		// ROS Subscriber initialisation
		ros::Subscriber sub = n.subscribe("optitrack/rigid_bodies", 1000, dataCollectCB);
    }
    else{
        cout << "Don't run anything !! " << endl;
        return 0;
    }

	ros::spin();

	if (param == "train"){
		timestamp_file.close();
	}
	else{
		rigid_body0_file.close();
		rigid_body1_file.close();
		rigid_body2_file.close();
		rigid_body3_file.close();
	}

	return 0;
}