
#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>
#include <Sabertooth.h>
#include <sensor_msgs/Joy.h>

Sabertooth ST0(131, Serial2); //robot right side
Sabertooth ST1(130, Serial2); //robot left side
Sabertooth ST2(129, Serial2); //ST2 to belt tilt
Sabertooth ST3(128, Serial2); //ST3 to plunge
Sabertooth ST4(132, Serial3); //ST4 to DUMPY!
ros::NodeHandle nh;

void callback(const sensor_msgs::Joy& joy)
{ 
  // these will be mapped to int values between -127 and 127
  // -127 is reverse 127 is forward
  int s0 = 0;
  int s1 = 0;
  int s2 = 0;
  int s3 = 0;
  int s4 = 0;
  int s5 = 0;
  int s6 = 0;
  int s7 = 0;
  int s8 = 0;
  int s9 = 0;
  int s10 = 0;
  
  ST0.motor(1, s0);
  ST0.motor(2, s1);
  ST1.motor(1, s2);
  ST1.motor(2, s3);
  ST2.motor(1, s4);
  ST2.motor(2, s5);
  ST3.motor(1, s6);
  ST3.motor(2, s7);
  ST4.motor(s8);
  
  if (s10==1) digitalWrite(3, HIGH);
  else digitalWrite(3, LOW);
  analogWrite(2, s9); // Trigger turns belt
}

ros::Subscriber <sensor_msgs::Joy> sub("joy",  callback); //subscribes to the joy topic


void setup()
{
    Serial1.begin(57600); //Baud for ROS
    Serial2.begin(9600);  //Baud for Sabertooths
    Serial3.begin(9600);
    
      //sets the drive and turn states to 0 so the motors start at 0
    ST1.drive(0);
    ST1.turn(0);
    ST2.drive(0);
    ST2.turn(0);
    ST3.drive(0);
    ST3.turn(0);
    ST4.motor(0);

    
    pinMode(2, OUTPUT); // PWM for belt speed
    pinMode(3, OUTPUT); // Digital for belt F/R
    
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
}
