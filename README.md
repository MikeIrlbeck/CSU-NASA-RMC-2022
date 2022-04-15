# CSU-NASA-RMC-2022

Mike: these are the commands I have been running to try and create a map from logged data.

http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

1. Terminal one
- roscore
2. Terminal two
- rosparam set use_sim_time true
- rosrun gmapping slam_gmapping scan:=base_scan
3. Terminal three - play the bag of saved data
- roscd moony_2dnav_simulation/bag
- rosbag play --clock basic_localization_stage.bag
4. Optional, Terminal four: visualize the data (you can add laser scan data and map data)
- rosrun rviz rviz

How to pull in submodule code (i.e. rplidar_ros)
    git submodule init
    git submodule update

How to get update recent updates from github
- cd catkin_ws/src/CSU-NASA-RMC-2022/
- git pull

How to push updates to our github
- git add *
- git commit -m "decription of updates"
- git push
> then you need to enter your username and token

How to start action server and client - using 3 terminals
1. roscore
2. rosrun your_package your_python_server.py
3. rosrun your_package your_python_client.py