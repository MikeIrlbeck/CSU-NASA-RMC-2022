//#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int32.h>
#define channelRB 4
#define channelRA 5
#define channelLB 11
#define channelLA 12

ros::NodeHandle nh;
std_msgs::Int32 int_msg;
ros::Publisher PlungeR("PlungeR", &int_msg);
ros::Publisher PlungeL("PlungeL", &int_msg);

int counterR = 0;
int counterL = 0; 
int currentStateRB;
int previousStateRB;
int currentStateLB;
int previousStateLB; 
 
void setup() {
  pinMode (channelRB,INPUT);
  pinMode (channelRA,INPUT);
  pinMode (channelLB,INPUT);
  pinMode (channelLB,INPUT);
  
  nh.initNode();
  nh.advertise(PlungeR);
  nh.advertise(PlungeL);
  previousStateRB = digitalRead(channelRB);
  previousStateLB = digitalRead(channelLB);
}

void loop() {
  counterR = count(counterR, 'R');
  counterL = count(counterL, 'L');
  nh.spinOnce();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int count(int counter, char side){
  
  if(side == 'R'){
    currentStateRB = digitalRead(channelRB);
    if (currentStateRB != previousStateRB){ 
      if (digitalRead(channelRA) != currentStateRB) {
        counter --;
      }
      else {
        counter ++;
      }
    int_msg.data = counter;
    PlungeR.publish( &int_msg );
    }
    previousStateRB = currentStateRB;
  }
  else{
    currentStateLB = digitalRead(channelLB);
    if (currentStateLB != previousStateLB){ 
      if (digitalRead(channelLA) != currentStateLB) {
        counter --;
      }
      else {
        counter ++;
      }
    int_msg.data = counter;
    PlungeL.publish( &int_msg );
    }
    previousStateLB = currentStateLB;
  }
  return counter;
}
