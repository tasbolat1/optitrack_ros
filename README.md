# optitrack_ros
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
