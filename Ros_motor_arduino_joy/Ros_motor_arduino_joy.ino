
#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <Sabertooth.h>
#include <SabertoothSimplified.h>
#include <SoftwareSerial.h>
#include <sensor_msgs/Joy.h>

SoftwareSerial SWSerial(NOT_A_PIN, 6);
SoftwareSerial SWSerial1(NOT_A_PIN, 5);
Sabertooth ST(128, SWSerial);
Sabertooth ST1(128, SWSerial1);
SoftwareSerial portROS(0, 1);
int deadband = 15;
ros::NodeHandle nh;

void callback(const sensor_msgs::Joy& joy)
{
  float joy1 = joy.axes[1];
  //float move2 = cmd_vel.buttons;
  float move1 = joy1 * 125;
    
  //ST.motor(1, move1);
  //ST1.motor(1, move1);
  //ST.motor(2, move2);
  //ST1.motor(2,move2);
  ST.drive(move1);
  ST1.drive(move1);
  //ST.turn(move2);
  //ST1.turn(move2);
}

ros::Subscriber <sensor_msgs::Joy> sub("joy",  callback);


void setup()
{
    SWSerial.begin(9600);
    SWSerial1.begin(9600);
    portROS.begin(57600);
  //sets the drive and turn states to 0 so the motors start at 0
    ST.drive(0);
    ST.turn(0);
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
}
