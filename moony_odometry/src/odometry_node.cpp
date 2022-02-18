#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

// bad global variables
double vx = 0;
double vy = 0;
double vth = 0;
double dt;

size_t cycleTracker = 0;
size_t publishHz = 100;

void IMUCallBack(const sensor_msgs::Imu &);

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  // subscribing to the IMU data
  ros::Subscriber odom_sub = n.subscribe("moony/imu", 1000, IMUCallBack);

  // where to publish robot odometry / pose
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("moony/odom", 100);
  tf::TransformBroadcaster odom_broadcaster;

  // initialize starting position
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(publishHz); // Hz
  
  while (n.ok()) {

    ros::spinOnce(); // check for incoming messages
    current_time = ros::Time::now();

    // compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    auto child = "base_footprint"; // old C style array? ends with a null character

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = child;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    /* 
    * next, we'll publish the odometry message over ROS
    */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = child;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // publish the message
    odom_pub.publish(odom);
    if (++cycleTracker % publishHz == publishHz - 1)
        ROS_INFO("\nx: %.2f, y: %.2f, theta: %.2f", x, y, th);
    
    last_time = current_time;
    r.sleep();
  }
}

void IMUCallBack(const sensor_msgs::Imu &msg) {
  // physics equation: a = (v2 - v1) / t
  // thus: v2 = a * t + v1
  if (cycleTracker % publishHz == publishHz - 1)
        ROS_INFO("\n\tDATA vx: %.2f, vy: %.2f, vtheta: %.2f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.angular_velocity.z);
  vx += msg.linear_acceleration.x * dt;
  vy += msg.linear_acceleration.y * dt;
  vth = msg.angular_velocity.z;
}
