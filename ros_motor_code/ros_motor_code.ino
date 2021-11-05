#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Sabertooth.h>
#include <SabertoothSimplified.h>

Sabertooth ST(128);

ros::NodeHandle nh;


void motor_cb( const std_msgs::UInt16& cmd_msg){
    ST.drive(cmd_msg.data); //set motor speed, should be from -127 to 127
    digitalWrite(13, HIGH-digitalRead(13)); //toggle led
}


ros::Subscriber<std_msgs::UInt16> sub("ST", motor_cb);

void setup() {
    SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
    ST.autobaud();

  //sets the drive and turn states to 0 so the motors start at 0
    ST.drive(0);
    ST.turn(0);

    pinMode(13, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
