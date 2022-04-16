
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
  int p = 80;
  int joy0 = joy.axes[0] * p; //LS LR
  int joy1 = joy.axes[1] * p; //LS UD
  int joy2 = -8 * (joy.axes[2] - 1); //LT
  int joy3 = joy.axes[3] * p; //RS LR
  int joy4 = joy.axes[4] * p; //RS UD
  int joy5 = -127.5 * (joy.axes[5] - 1); //RT
  int button1 = joy.buttons[1]; //B button
  int button0 = joy.buttons[0];
  int button2 = joy.buttons[2];
  int button3 = joy.buttons[3];
  
  if (button2==1){
    ST0.motor(1, -joy1);
    ST0.motor(2, joy1);
    ST1.motor(1, joy1);
    ST1.motor(2, -joy1);
  }
  else{
    ST0.motor(1, -joy0);
    ST0.motor(2, joy0);
    ST1.motor(1, -joy0);
    ST1.motor(2, joy0);
  }
  
  
  ST2.motor(1, -joy3);
  ST3.motor(2, joy3);

  if (button3==1){
    ST3.motor(1, joy4);
    ST2.motor(2, joy4);
  }
  else {
    ST3.motor(1, joy4);
    ST2.motor(2, -joy4);
  }
  
  if (button0==1) ST4.motor(-joy2);
  else ST4.motor(joy2); //BIG FAT DUMPY
  // If button B is pressed Reverse else forward
  
  if (button1==1) digitalWrite(3, HIGH);
  else digitalWrite(3, LOW);
  
  analogWrite(2, joy5); // Trigger turns belt
}

ros::Subscriber <sensor_msgs::Joy> sub("joy",  callback);


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
