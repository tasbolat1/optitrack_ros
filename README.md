# optitrack_ros

This code works well for ROS kinetic and *not tested* for others. Main difference from https://github.com/crigroup/optitrack/tree/feat/support-kinetic is that this code can track single marker. Anyways, this code is adopted from https://github.com/crigroup/optitrack/tree/feat/support-kinetic.

**Install** <br />
Follow folling link to install vrpn before installing other packages <br />
https://answers.ros.org/question/285887/ros_vrpn_client-installation-help/ <br />

Install optirx package <br />
`pip install optirx --user` <br />
Go to your catkin workspace <br />
`cd ~/catkin_ws/src` <br />
Clone the repisotory <br />
`git clone https://github.com/tasbolat1/optitrack_ros.git` <br />
Install any missing files <br />
`rosdep update` <br />
`rosdep install --from-paths . --ignore-src -y` <br />
Compile your ROS <br />
`cd ~/catkin_ws && catkin_make` <br />
**How to run:** <br />
1. Get network name: <br />
`ifconfig` <br />
2. Run your program <br />
`roslaunch optitrack optitrack_pipeline.launch iface:="network_name"`



**How to run: Optitrack recorder** <br />
1. Make sure that `motive_ip` (optitrack_ros/scripts/control_commands) is set to Motive's PC's IP <br />
2. Make sure that iface command is given <br />
3. Make sure that the connection is there
3. To run:
`rosrun optitrack control_commands.py`
4. To start recording:
`rostopic pub -1 /filename std_msgs/String "name_of_take"`
5. To stop recording
`pub -1 /filename std_msgs/String "stop"`
