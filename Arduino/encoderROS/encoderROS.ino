#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int32.h>
#define channelB 4
#define channelA 5

ros::NodeHandle nh;
std_msgs::Int32 int_msg;
ros::Publisher wheel1("wheel1", &int_msg);


int counter = 0; 
int currentStateB;
int previousStateB; 
 
void setup() {
  pinMode (channelB,INPUT);
  pinMode (channelA,INPUT);
  
  nh.initNode();
  nh.advertise(wheel1);
  previousStateB = digitalRead(channelB);
}

void loop() {
  counter = count(counter);
  nh.spinOnce();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int count(int counter){
  currentStateB = digitalRead(channelB);
  if (currentStateB != previousStateB){ 
    if (digitalRead(channelA) != currentStateB) {
      counter --;
    }
    else {
      counter ++;
    }
  int_msg.data = counter;
  wheel1.publish( &int_msg );
  }
  previousStateB = currentStateB;
  return counter;
}
