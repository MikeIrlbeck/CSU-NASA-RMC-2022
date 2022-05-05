// Mikes addition to Ryan's original code to more easily integrate with autonomy.
// 5/3/22
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <math.h>

#include <Sabertooth.h>
#define Teensy41

//for encoders
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 4
#define ENC_IN_RIGHT_A 33
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 34
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -2147483648;
const int encoder_maximum = 2147483648;
// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 




//initialization
ros::NodeHandle nh;
std_msgs::Float64 tiltL_msg;
std_msgs::Float64 tiltR_msg;
std_msgs::Int16 encoderL_Aticks;
ros::Publisher leftPubA("ENC_IN_LEFT_A", &encoderL_Aticks);
std_msgs::Int16 encoderL_Bticks;
ros::Publisher leftPubB("ENC_IN_LEFT_B", &encoderL_Bticks);
std_msgs::Int16 encoderR_Aticks;
ros::Publisher rightPubA("ENC_IN_RIGHT_A", &encoderR_Aticks);
std_msgs::Int16 encoderR_Bticks;
ros::Publisher rightPubB("ENC_IN_RIGHT_B", &encoderR_Bticks);

ros::Publisher TiltL("moony/left_linear_actuator_pos_feedback", &tiltL_msg);
ros::Publisher TiltR("moony/right_linear_actuator_pos_feedback", &tiltR_msg);

Sabertooth driveRightWheels(131, Serial2); //robot right side
Sabertooth driveLeftWheels(130, Serial2); //robot left side
Sabertooth left1LinearActuator2Plunge(129, Serial2); 
Sabertooth right2LinearActuator1Plunge(128, Serial2); 
Sabertooth ST4(132, Serial3); //ST4 to DUMPY!



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

#define WINDOW_SIZE 10
int sumIndex1 = 0;
int sumIndex2 = 0;
int averageSum1 = 0;
int averageSum2 = 0;
int averageReadings1[WINDOW_SIZE]; ///set window size
int averageReadings2[WINDOW_SIZE];

// test
// Rotary Encoder Inputs
 #define channelB 4
 #define channelA 5

 int counter1 = 0; 
 int currentStateB;
 int previousStateB; 

 std_msgs::Int16 encoderPosition;
ros::Publisher encoderTest("RightEncoderPos", &encoderPosition);

void xboxCallback(const sensor_msgs::Joy& joy)
{ 
  int p = 80;
  leftStickLeftRight = joy.axes[0] * p; //Left/Right Axis stick left
  leftStickUpDown = joy.axes[1] * p; //Up/Down Axis stick left
  leftTrigger = -8 * (joy.axes[2] - 1); //Left/Right Axis stick right
  rightStickLeftRight = joy.axes[3] * p; //Up/Down Axis stick right
  rightStickUpDown = joy.axes[4] * 20; //rightTrigger, Ryans comment: RS UD
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

    //interrupt for encoders
    pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
    pinMode(ENC_IN_LEFT_B , INPUT);
    pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
    pinMode(ENC_IN_RIGHT_B , INPUT);
    // Every time the pin goes high, this is a tick
    
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(subLeftActuator);
    nh.subscribe(subRightActuator);
    nh.advertise(rightPubA);
    nh.advertise(rightPubB);
    nh.advertise(leftPubA);
    nh.advertise(leftPubB);
    nh.advertise(TiltL);
    nh.advertise(TiltR);

    // Set encoder pins as inputs  
   pinMode (channelB,INPUT);
   pinMode (channelA,INPUT);

   previousStateB = digitalRead(channelB);
   nh.advertise(encoderTest);
   encoderPosition.data = 0;

}

void loop()
{
  currentMillis = millis();
  
  
  if (teleop) {
    teleOp(); 
  }
  else {
    autonomyOp();
  }

  counter1 = count(counter1);


  previousMillis = currentMillis; 
  //encoderL_Aticks.data = digitalRead(ENC_IN_LEFT_A);
  //rightPubA.publish(&encoderL_Aticks );
  rightPubB.publish( digitalRead(ENC_IN_LEFT_B));
  encoderL_Aticks.data = digitalRead(ENC_IN_LEFT_A);
  leftPubA.publish( &encoderL_Aticks);
  leftPubB.publish( digitalRead(ENC_IN_RIGHT_B));
  
  tiltL_msg.data = movingAverage(int(analogRead(A13)), 'L')/10;
  TiltL.publish( &tiltL_msg );
  tiltR_msg.data = movingAverage(int(analogRead(A9)), 'R')/10;
  TiltR.publish( &tiltR_msg );

// test
  encoderTest.publish( &encoderPosition );

  nh.spinOnce();
  
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int count(int counter){
  currentStateB = digitalRead(channelB);
  if (currentStateB != previousStateB){ 
    if (digitalRead(channelA) != currentStateB) {
      counter --;
    }
    else {
      counter ++;
    }
   Serial3.println(counter);
   encoderPosition.data = counter;
  }
  previousStateB = currentStateB;
  return counter;
 }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
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
    moveLinearActuators(rightStickLeftRight, rightStickLeftRight);
//    left1LinearActuator2Plunge.motor(1, rightStickLeftRight);  
//    right2LinearActuator1Plunge.motor(2, rightStickLeftRight);
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
  }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void moveLinearActuators(int val) {
  moveLinearActuators(val, val);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void moveLinearActuators(int left, int right) {
  left1LinearActuator2Plunge.motor(1, -left); 
  right2LinearActuator1Plunge.motor(2, right);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void autonomyOp() {
  moveLinearActuators(leftLinearActuatorEffCmd, rightLinearActuatorEffCmd);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int movingAverage(int value, char side){
  if(side == 'R'){
    averageSum1 = averageSum1 - averageReadings1[sumIndex1];
    averageReadings1[sumIndex1] = value;
    averageSum1 = averageSum1 + value;
    sumIndex1 = (sumIndex1 + 1) % WINDOW_SIZE;
    return (averageSum1 / WINDOW_SIZE);
  }
  else{
    averageSum2 = averageSum2 - averageReadings2[sumIndex2];
    averageReadings2[sumIndex2] = value;
    averageSum2 = averageSum2 + value;
    sumIndex2 = (sumIndex2 + 1) % WINDOW_SIZE;
    return (averageSum2 / WINDOW_SIZE);
  }
}
