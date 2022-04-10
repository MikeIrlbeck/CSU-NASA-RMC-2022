#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "moony_autonomy_navigation/navigation_goal.h"

int main(int argc, char** argv) {
 
    return 0;
}
// #include <chores/DoDishesAction.h>  // Note: "Action" is appended
// typedef actionlib::SimpleActionServer<chores::DoDishesAction> Server;

// void execute(const chores::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
// {
//   // Do lots of awesome groundbreaking robot stuff here
//   as->setSucceeded();
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "do_dishes_server");
//   ros::NodeHandle n;
//   Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
//   server.start();
//   ros::spin();
//   return 0;
// }


// #include "ros/ros.h"
// #include "std_srvs/Empty.h"
// #include "moony_autonomy_navigation/NavigationStringMessage.h"
// #include <iostream>
// #include <string>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// // Import the service message header file generated from the Empty.srv message

// using namespace::std;
// typedef moony_autonomy_navigation::NavigationStringMessage NavMsg;

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// bool sendMiningGoal();

// // We define the callback function of the service
// bool my_callback(NavMsg::Request  &req, NavMsg::Response &res) {
//   string location(req.location);
//   if(location.compare("mine") == 0) {
//     cout << "time to go mine!\n";
//     if(!sendMiningGoal())
//         ROS_ERROR("Cannot find a valid mining goal.");
//   }
//   else if(location.compare("dump") == 0) {
//     cout << "time to go dump!\n";
//   }
//   else {
//     ROS_ERROR("Indeterminate mining request");
//     res.success = false;
//     return false;
//   }
//   res.success = true;
//   return true;
// }
 
// int main(int argc, char **argv) {
//   ros::init(argc, argv, "navigate_server");
//   ros::NodeHandle nh;
  
//   ros::ServiceServer my_service = nh.advertiseService("/moony_navigate", my_callback); // create the Service called                                                                                          // my_service with the defined                                                                                        // callback
//   ros::spin(); // mantain the service open.
 
//   return 0;
// }

// bool sendMiningGoal() {
//   MoveBaseClient actionMove("move_base", true);
//   while(!actionMove.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//     //we'll send a goal to the robot to move 1 meter forward
//   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.stamp = ros::Time::now();

//   goal.target_pose.pose.position.x = -1.0;
//   goal.target_pose.pose.orientation.w = 1.0;

//   ROS_INFO("Sending goal");
//   actionMove.sendGoal(goal);

// //   actionMove.waitForResult();

// //   if(actionMove.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// //     return true;
// //   else
//     return false;

// }