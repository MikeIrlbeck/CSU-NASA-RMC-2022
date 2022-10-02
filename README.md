# CSU-NASA-RMC-2022
For easy testing, go to The Construct website and use their free online simulator.  See sections below for help on using git with Github.

## Run the robot at competition
Here are the instructions to run the robot at competition (assuming that you have properly built and sourced ROS and the required packages):

Terminal 1 
`roscore`

Terminal 2 
`rosrun joy joy_node`

Terminal 3 
`roslaunch moony_bringup start_moony.launch`
OR
`roslaunch moony_bringup start_moonyWithLidar.launch`
OR
`roslaunch moony_bringup start_moonyWithLidarAndBeacon.launch`

Terminal 4 - enable communication to the Tensy
`rosrun rosserial_python serial_node.py /dev/ttyACM1`

Terminal 5 - for topic visulization and sending commands
`rqt`
- see topic monitor
- see topic publisher
- dynamic reconfigure to change PID parameters

### Misc

To view USB ports
	`ls /dev/ | grep USB`
To view ACM ports
	`ls /dev/ | grep ACM`
    

## Navigation simulation in Gazebo
Each of the following commands should be performed in sequential order in individual terminals resulting in multiple terminal commands running at one time.
- `roscore`
- `roslaunch moony_description rviz_view_move_base.launch # for data visualization`
- `roslaunch moony_navigation_gazebo competition_main.launch`
- `roslaunch moony_navigation_gazebo gmapping.launch`
- `rosrun moony_autonomy_navigate navigate_server.py`
- `rosrun moony_autonomy_navigate navigate_client.py`

## How to pull in submodule code (i.e. rplidar_ros)
`git submodule init
git submodule update`

## How to get update recent updates from github
`cd catkin_ws/src/CSU-NASA-RMC-2022/
git pull`

## How to push updates to our github
`git add *
git commit -m "decription of updates"
git push`
> then you need to enter your username and token

## How to start action server and client - using 3 terminals
1. `roscore`
2. `rosrun your_package your_python_server.py`
3. `rosrun your_package your_python_client.py`

## Extras: Try to create map from logged data
These are the commands I have been running to try and create a map from logged data.

http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData

1. Terminal one
- `roscore`
2. Terminal two
- `rosparam set use_sim_time true`
- `rosrun gmapping slam_gmapping scan:=base_scan`
3. Terminal three - play the bag of saved data
- `roscd moony_2dnav_simulation/bag`
- `rosbag play --clock basic_localization_stage.bag`
4. Optional, Terminal four: visualize the data (you can add laser scan data and map data)
- `rosrun rviz rviz`
