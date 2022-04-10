#include "ros/ros.h"
#include "std_srvs/Empty.h"
// #include "iri_wam_reproduce_trajectory/ExecTraj.h" // could not find this header???
    // but it worked, even though ExecTraj.srv was inside the srv folder
#include <ros/package.h> // for searching, ros::package::getPath()

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_client");
    ros::NodeHandle nh;
    
    ros::ServiceClient exec_traj_service = nh.serviceClient<std_srvs::Empty>("/my_service");
    
    // This ros::package::getPath works in the same way as $(find name_of_package) in the launch files.
    // trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/get_food.txt";

    std_srvs::Empty srv;
    if (exec_traj_service.call(srv))
    {
    ROS_INFO("%s", "Service successfully called. Executing trajectory.");
    }
    else
    {
    ROS_ERROR("Failed to call service /my_service");
    return 1;
    }

    return 0;
}