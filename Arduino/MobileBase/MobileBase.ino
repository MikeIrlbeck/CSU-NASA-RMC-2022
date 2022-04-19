
#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <Sabertooth.h>
// #include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

Sabertooth rightWheels(131, Serial2); //robot right side
Sabertooth leftWheels(130, Serial2); //robot left side
Sabertooth beltTilt(129, Serial2); //beltTilt to belt tilt
Sabertooth plunge(128, Serial2); //plunge to plunge
Sabertooth dump(132, Serial3); //dump to DUMPY!
ros::NodeHandle nh;

void callback(const geometry_msgs::Twist& twist)
{ 

  double linearX = twist.linear.x;
  double angularZ = twist.angular.z;

  // this is where a PID would be useful
  
  rightWheels.motor(1, s0);
  rightWheels.motor(2, s1);
  leftWheels.motor(1, s2);
  leftWheels.motor(2, s3);
  // beltTilt.motor(1, s4);
  // beltTilt.motor(2, s5);
  // plunge.motor(1, s6);
  // plunge.motor(2, s7);
  // dump.motor(s8);
  
  if (s10==1) digitalWrite(3, HIGH);
  else digitalWrite(3, LOW);
  analogWrite(2, s9); // Trigger turns belt
}

ros::Subscriber <geometry_msgs::Twist> subCmdVel("moony/cmd_vel",  callback); //subscribes to the joy topic


void setup()
{
    Serial1.begin(57600); //Baud for ROS
    Serial2.begin(9600);  //Baud for Sabertooths
    Serial3.begin(9600);
    
      //sets the drive and turn states to 0 so the motors start at 0
    rightWheels.drive(0);
    rightWheels.turn(0);
    leftWheels.drive(0);
    leftWheels.turn(0);
    // beltTilt.drive(0);
    // beltTilt.turn(0);
    // plunge.drive(0);
    // plunge.turn(0);
    // dump.motor(0);

    // pinMode(2, OUTPUT); // PWM for belt speed
    // pinMode(3, OUTPUT); // Digital for belt F/R
    
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
}
