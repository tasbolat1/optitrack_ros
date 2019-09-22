# optitrack_ros

This code works well for ROS kinetic and *not tested* for others. Main difference from https://github.com/crigroup/optitrack/tree/feat/support-kinetic is that this code can track single marker. Anyways, all the credit goes to https://github.com/crigroup/optitrack/tree/feat/support-kinetic.

**Install** <br />
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
