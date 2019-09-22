# optitrack_ros
** Install ** <br />
Install optirx package <br />
`pip install optirx --user` <br />
**How to run:** <br />
1. Get network name:

`ifconfig`

2. Run your program

`roslaunch optitrack optitrack_pipeline.launch iface:="network_name"`
