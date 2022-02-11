# CSU-NASA-RMC-2022

Mike: these are the commands I have been running to try and create a map from logged data.

http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

1. Terminal one
roscore
2. Terminal two
rosparam set use_sim_time true
rosrun gmapping slam_gmapping scan:=base_scan
3. Terminal three - play the bag of saved data
rosbag play --clock basic_localization_stage.bag
4. Optional, Terminal four: visualize the data (you can add laser scan data and map data)
rosrun rviz rviz
