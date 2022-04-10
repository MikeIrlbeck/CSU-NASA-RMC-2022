#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "moony_autonomy_navigation/NavigationStringMessage.h"
#include "moony_autonomy_navigation/navigation_goal.h"
  
// #include "iri_wam_reproduce_trajectory/ExecTraj.h" // could not find this header???
    // but it worked
#include <ros/package.h> // for searching, ros::package::getPath()
#include <iostream>
using namespace std;
// Import rospackage
typedef moony_autonomy_navigation::NavigationStringMessage NavMsg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigate_client");
    ros::NodeHandle nh;
    
    
    // Create the connection to the service /execute_trajectory
    ros::ServiceClient exec_traj_service = nh.serviceClient<NavMsg>("/moony_navigate");
    // This ros::package::getPath works in the same way as $(find name_of_package) in the launch files.
    // trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/get_food.txt";

    NavMsg msg_mine;
    msg_mine.request.location = "mine";
    exec_traj_service.call(msg_mine);

    NavMsg msg_dump;
    msg_dump.request.location = "dump";
    exec_traj_service.call(msg_dump);
    
    // NavMsg msg_other;
    // msg_other.request.location = "";
    // exec_traj_service.call(msg_other);
    
    if(msg_mine.response.success)
    {
    ROS_INFO("%s", "Service successfully called. Executing trajectory: ");
    }
    else
    {
    ROS_ERROR("Failed to call service /moony_navigate");
    return 1;
    }


    return 0;
}

// #include <chores/DoDishesAction.h> // Note: "Action" is appended
// #include <actionlib/client/simple_action_client.h>

// typedef actionlib::SimpleActionClient<chores::DoDishesAction> Client;

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "do_dishes_client");
//   Client client("do_dishes", true); // true -> don't need ros::spin()
//   client.waitForServer();
//   chores::DoDishesGoal goal;
//   // Fill in goal here
//   client.sendGoal(goal);
//   client.waitForResult(ros::Duration(5.0));
//   if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     printf("Yay! The dishes are now clean");
//   printf("Current State: %s\n", client.getState().toString().c_str());
//   return 0;
// }