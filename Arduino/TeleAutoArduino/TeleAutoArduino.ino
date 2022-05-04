// Mikes addition to Ryan's original code to more easily integrate with autonomy.
// 5/3/22
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <Sabertooth.h>
#define Teensy41

//for encoders
#define channelRB 4
#define channelRA 5
#define channelLB 11
#define channelLA 12

ros::NodeHandle nh;
std_msgs::Int32 int_msg;
std_msgs::Float64 tiltL_msg;
std_msgs::Float64 tiltR_msg;
ros::Publisher PlungeR("PlungeR", &int_msg);
ros::Publisher PlungeL("PlungeL", &int_msg);
ros::Publisher TiltL("moony/left_linear_actuator_pos_feedback", &tiltL_msg);
ros::Publisher TiltR("moony/right_linear_actuator_pos_feedback", &tiltR_msg);

Sabertooth driveRightWheels(131, Serial2); //robot right side
Sabertooth driveLeftWheels(130, Serial2); //robot left side
Sabertooth left1LinearActuator2Plunge(129, Serial2); 
Sabertooth right2LinearActuator1Plunge(128, Serial2); 
Sabertooth ST4(132, Serial3); //ST4 to DUMPY!

int counterR = 0;
int counterL = 0; 
int currentStateRB;
int previousStateRB;
int currentStateLB;
int previousStateLB;

float leftLinearActuatorEffCmd = 0;
float rightLinearActuatorEffCmd = 0;

int leftStickLeftRight; //LS LR
int leftStickUpDown; //LS UD
int leftTrigger; //LT
int rightStickLeftRight; //RS LR
int rightStickUpDown; //RS UD
int rightTrigger; //rightTrigger
int buttonB; //B button
int buttonA;
int buttonX;
int buttonY;
int leftBumper;
int buttonBack;
int buttonCrossUpDown;
int buttonCrossLeftRight;
bool teleop = true;

void xboxCallback(const sensor_msgs::Joy& joy)
{ 
  int p = 80;
  leftStickLeftRight = joy.axes[0] * p; //Left/Right Axis stick left
  leftStickUpDown = joy.axes[1] * p; //Up/Down Axis stick left
  leftTrigger = -8 * (joy.axes[2] - 1); //Left/Right Axis stick right
  rightStickLeftRight = joy.axes[3] * p; //Up/Down Axis stick right
  rightStickUpDown = joy.axes[4] * p; //rightTrigger, Ryans comment: RS UD
  rightTrigger = -127.5 * (joy.axes[5] - 1); //LT Ryan's comment rightTrigger
  buttonB = joy.buttons[1]; 
  buttonA = joy.buttons[0]; 
  buttonX = joy.buttons[2]; 
  buttonY = joy.buttons[3]; 
  leftBumper = joy.buttons[4]; 
  buttonBack = joy.buttons[6]; 
  buttonCrossLeftRight = joy.axes[6];
  buttonCrossUpDown = joy.axes[7]; 

  if (buttonCrossLeftRight == 1.0) {
    teleop = true;
  }
  else if (buttonCrossLeftRight == -1.0) {
    teleop = false;
  }
  // else remain in the same state
}

void leftActuatorCallback(const std_msgs::Float64& msg) {
  leftLinearActuatorEffCmd = constrain(msg.data, -80, 80);
}

void rightActuatorCallback(const std_msgs::Float64& msg) {
  rightLinearActuatorEffCmd = constrain(msg.data, -80, 80);
}

ros::Subscriber <sensor_msgs::Joy> sub("joy",  xboxCallback);
ros::Subscriber <std_msgs::Float64> subLeftActuator("moony/left_linear_actuator_eff_cmd",  leftActuatorCallback);
ros::Subscriber <std_msgs::Float64> subRightActuator("moony/right_linear_actuator_eff_cmd",  rightActuatorCallback);


void setup()
{
    pinMode (channelRB,INPUT);
    pinMode (channelRA,INPUT);
    pinMode (channelLB,INPUT);
    pinMode (channelLB,INPUT);
    
    Serial1.begin(500000); //Baud for ROS
    Serial2.begin(9600);  //Baud for Sabertooths
    Serial3.begin(9600);
    
      //sets the drive and turn states to 0 so the motors starightTrigger at 0
    driveLeftWheels.drive(0);
    driveLeftWheels.turn(0);
    left1LinearActuator2Plunge.drive(0);
    left1LinearActuator2Plunge.turn(0);
    right2LinearActuator1Plunge.drive(0);
    right2LinearActuator1Plunge.turn(0);
    ST4.motor(0);

    
    pinMode(2, OUTPUT); // PWM for belt speed
    pinMode(3, OUTPUT); // Digital for belt F/R
    
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(subLeftActuator);
    nh.subscribe(subRightActuator);
    nh.advertise(PlungeR);
    nh.advertise(PlungeL);
    nh.advertise(TiltL);
    nh.advertise(TiltR);
    previousStateRB = digitalRead(channelRB);
    previousStateLB = digitalRead(channelLB);
}

void loop()
{
  //count(counterR, 'R');
  //count(counterL, 'L');

  if (teleop) {
    teleOp(); // if teleop mode is selected
  }
  else {
    autonomyOp();
  }


  int_msg.data= int(rightLinearActuatorEffCmd);
  PlungeL.publish( &int_msg);
  tiltL_msg.data = int(analogRead(A13)/10.0);
  TiltL.publish( &tiltL_msg );
  tiltR_msg.data = int(analogRead(A9)/10.0);
  TiltR.publish( &tiltR_msg );
  delay(7);
  nh.spinOnce();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
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

void teleOp() {
    if (buttonX==1){ // to turn hold x
    driveRightWheels.motor(1, leftStickLeftRight);
    driveRightWheels.motor(2, leftStickLeftRight);
    
    driveLeftWheels.motor(1, -leftStickLeftRight);
    driveLeftWheels.motor(2, leftStickLeftRight); 
  }
  else{
    driveRightWheels.motor(1, leftStickUpDown); 
    driveRightWheels.motor(2, leftStickUpDown);
    driveLeftWheels.motor(1, leftStickUpDown);
    driveLeftWheels.motor(2, -leftStickUpDown); // the negative means this motor was wired oppositely
  }

  // linear acuator
  if (buttonBack==1){ 
    left1LinearActuator2Plunge.motor(1, rightStickLeftRight);  
    right2LinearActuator1Plunge.motor(2, rightStickLeftRight);
  }
  else {
    moveLinearActuators(rightStickLeftRight);
  }

  // plunge
  if (leftBumper==1){
    left1LinearActuator2Plunge.motor(2, rightStickUpDown); 
    right2LinearActuator1Plunge.motor(1, rightStickUpDown);
  }
  else {
    left1LinearActuator2Plunge.motor(2, -rightStickUpDown); 
    right2LinearActuator1Plunge.motor(1, rightStickUpDown);

  }
  
  if (buttonA==1) {
    ST4.motor(-leftTrigger);
  }
  else ST4.motor(leftTrigger); //BIG FAT DUMPY
  // If button B is pressed Reverse else forward
  
  if (buttonB==1) digitalWrite(3, HIGH);
  else digitalWrite(3, LOW);
  
  analogWrite(2, rightTrigger); // Trigger turns belt
  if (buttonCrossUpDown==-1){
    counterR = 0;
    counterL = 0;
  }
}

void moveLinearActuators(int val) {
  moveLinearActuators(val, val);
}
void moveLinearActuators(int left, int right) {
  left1LinearActuator2Plunge.motor(1, -left); 
  right2LinearActuator1Plunge.motor(2, right);
}
void autonomyOp() {
  moveLinearActuators(leftLinearActuatorEffCmd, rightLinearActuatorEffCmd);
}
