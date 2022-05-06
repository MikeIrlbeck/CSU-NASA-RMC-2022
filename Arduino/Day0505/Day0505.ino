
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
 
int beltSpinPin = 3;



//initialization
ros::NodeHandle nh;
std_msgs::Float64 tiltL_msg;
std_msgs::Float64 tiltR_msg;
//std_msgs::Int16 encoderL_Aticks;
//ros::Publisher leftPubA("ENC_IN_LEFT_A", &encoderL_Aticks);
//std_msgs::Int16 encoderL_Bticks;
//ros::Publisher leftPubB("ENC_IN_LEFT_B", &encoderL_Bticks);
//std_msgs::Int16 encoderR_Aticks;
//ros::Publisher rightPubA("ENC_IN_RIGHT_A", &encoderR_Aticks);
//std_msgs::Int16 encoderR_Bticks;
//ros::Publisher rightPubB("ENC_IN_RIGHT_B", &encoderR_Bticks);

ros::Publisher TiltL("moony/left_linear_actuator_pos_feedback", &tiltL_msg);
ros::Publisher TiltR("moony/right_linear_actuator_pos_feedback", &tiltR_msg);

std_msgs::Float64 leftTilt;
ros::Publisher autonomyLeftLinearActuator("/leftLinearActuator_position_controller/command", &leftTilt);
std_msgs::Float64 rightTilt;
ros::Publisher autonomyRightLinearActuator("/rightLinearActuator_position_controller/command", &rightTilt);


Sabertooth driveRightWheels(131, Serial2); //robot right side
Sabertooth driveLeftWheels(130, Serial2); //robot left side
Sabertooth left1LinearActuator2Plunge(129, Serial2); 
Sabertooth right2LinearActuator1Plunge(133, Serial2); 
Sabertooth dumpBucket(132, Serial3); //ST4 to DUMPY!



float leftLinearActuatorEffCmd = 0;
float rightLinearActuatorEffCmd = 0;

int leftStickLeftRight; //LS LR
int leftStickUpDown; //LS UD
int leftStickUpDownDump;
int leftTrigger; //LT
int rightStickLeftRight; //RS LR
int rightStickUpDown; //RS UD
int rightTrigger; //rightTrigger
int buttonB; //B button
int buttonA;
int buttonX;
int buttonY;
int leftBumper;
int rightBumper;
int buttonBack;
int buttonStart;
int buttonCrossUpDown;
int buttonCrossLeftRight;
bool teleop = true;
bool transportationTilt = false;
bool mineTilt = false;
bool dumpTilt = false;

//for floating average
#define WINDOW_SIZE 30
int sumIndex1 = 0;
int sumIndex2 = 0;
int averageSum1 = 0;
int averageSum2 = 0;
int averageReadings1[WINDOW_SIZE]; ///set window size
int averageReadings2[WINDOW_SIZE];

void setMineTilt() {
  mineTilt = true;
  transportationTilt = false;
  dumpTilt = false;
  teleop = false;
}
void setTransportationTilt() {
  mineTilt = false;
  transportationTilt = true;
  dumpTilt = false;
  teleop = false;
}
void setDumpTilt() {
  mineTilt = false;
  transportationTilt = false;
  dumpTilt = true;
  teleop = false;
}
void setNoTiltAutonomy() {
  mineTilt = false;
  transportationTilt = false;
  dumpTilt = false;
  teleop = true;
}
void xboxCallback(const sensor_msgs::Joy& joy)
{ 
  static int p = 50; // plunge multiplier
  leftStickLeftRight = joy.axes[0] * p; //Left/Right Axis stick left
  leftStickUpDown = joy.axes[1] * p; //Up/Down Axis stick left
  leftStickUpDownDump = joy.axes[1]; //Up/Down Axis stick left

  leftTrigger = -8 * (joy.axes[2] - 1); //Left/Right Axis stick right
  rightStickLeftRight = joy.axes[3] * p; //Up/Down Axis stick right
  rightStickUpDown = joy.axes[4] * 20; //rightTrigger, Ryans comment: RS UD
  rightTrigger = -127.5 * (joy.axes[5] - 1); //LT Ryan's comment rightTrigger
  buttonB = joy.buttons[1]; 
  buttonA = joy.buttons[0]; 
  buttonX = joy.buttons[2]; 
  buttonY = joy.buttons[3]; 
  leftBumper = joy.buttons[4]; 
  rightBumper = joy.buttons[5]; 
  buttonBack = joy.buttons[6]; 
  buttonStart = joy.buttons[7];
  buttonCrossLeftRight = joy.axes[6];
  buttonCrossUpDown = joy.axes[7]; 

  if (buttonX == 1) {
    teleop = true;
  }
  else {
    if (buttonCrossLeftRight == 1.0) {
      setTransportationTilt();
    }
    else if (buttonCrossLeftRight == -1.0) {
      setMineTilt();
    }
    else if (buttonCrossUpDown == 1) {
      setDumpTilt();
    }
  }
  // else remain in the same state
}

float deadBand = 10;
void leftActuatorCallback(const std_msgs::Float64& msg) {
  leftLinearActuatorEffCmd = constrain(msg.data, -80, 80);
  if (leftLinearActuatorEffCmd < deadBand && leftLinearActuatorEffCmd > -deadBand) {
    leftLinearActuatorEffCmd = 0;
  }
}

void rightActuatorCallback(const std_msgs::Float64& msg) {
  rightLinearActuatorEffCmd = constrain(msg.data, -80, 80);
  if (rightLinearActuatorEffCmd < deadBand && rightLinearActuatorEffCmd > -deadBand) {
    rightLinearActuatorEffCmd = 0;
  }
}

ros::Subscriber <sensor_msgs::Joy> sub("joy",  xboxCallback);
ros::Subscriber <std_msgs::Float64> subLeftActuator("moony/left_linear_actuator_eff_cmd",  leftActuatorCallback);
ros::Subscriber <std_msgs::Float64> subRightActuator("moony/right_linear_actuator_eff_cmd",  rightActuatorCallback);


void stopAllMotors();
void teleOp();
void plunge(int val);
void plunge(int left, int right);
void moveLinearActuators(int val);
void moveLinearActuators(int left, int right);
void autonomyOp();
int movingAverage(int value, char side);

void setup()
{
    Serial1.begin(500000); //Baud for ROS
    Serial2.begin(9600);  //Baud for Sabertooths
    Serial3.begin(9600);
    
      //sets the drive and turn states to 0 so the motors starightTrigger at 0
    stopAllMotors();


    
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
//    nh.advertise(rightPubA);
//    nh.advertise(rightPubB);
//    nh.advertise(leftPubA);
//    nh.advertise(leftPubB);
    nh.advertise(TiltL);
    nh.advertise(TiltR);

    nh.advertise(autonomyLeftLinearActuator);
    nh.advertise(autonomyRightLinearActuator);


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


//  int_msg.data= int(rightLinearActuatorEffCmd);
//  PlungeL.publish( &int_msg);
//  tiltL_msg.data = int(analogRead(A13)/10.0);
//  TiltL.publish( &tiltL_msg );
//  tiltR_msg.data = int(analogRead(A9)/10.0);
//  TiltR.publish( &tiltR_msg );

  tiltL_msg.data = movingAverage(int(analogRead(A13)), 'L')/10;
  TiltL.publish( &tiltL_msg );
  tiltR_msg.data = movingAverage(int(analogRead(A9)), 'R')/10;
  TiltR.publish( &tiltR_msg );

  delay(7);
  nh.spinOnce();
}

////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void stopAllMotors() {
    driveLeftWheels.drive(0);
    driveLeftWheels.turn(0);
    left1LinearActuator2Plunge.drive(0);
    left1LinearActuator2Plunge.turn(0);
    right2LinearActuator1Plunge.drive(0);
    right2LinearActuator1Plunge.turn(0);
    dumpBucket.motor(0);

    teleop = true;
    setNoTiltAutonomy();

}
void teleOp() {
  if (buttonX == 1) {
    stopAllMotors();
  }
  else {
    // plunge
    if (buttonY==1){ 
      if (leftBumper==1){
        // tilt CCW
        plunge(-20, 20); 
      }
      else if (rightBumper==1){
        // tilt CW
        plunge(20, -20); 
      }
      else {
        plunge(leftStickUpDown); 
      }
    }
  
    // tilt
    else if (buttonA == 1) {
    
      if (leftBumper==1){
        // tilt CCW
        moveLinearActuators(-20, 20); 
      }
      else if (rightBumper==1){
        // tilt CW
        moveLinearActuators(20, -20); 
      }
      else {
        moveLinearActuators(leftStickUpDown);
      }
    }
    else if (buttonB == 1) {
       dumpBucket.motor(-leftStickUpDownDump * 15);
       //constrain(-leftStickUpDown/10, -8, 8));
    }
    else {
      // drive forward and back
      driveRightWheels.motor(1, -leftStickUpDown - rightStickLeftRight); 
      driveRightWheels.motor(2, leftStickUpDown + rightStickLeftRight);
      driveLeftWheels.motor(1, leftStickUpDown - rightStickLeftRight);
      driveLeftWheels.motor(2, -leftStickUpDown + rightStickLeftRight); // the negative means this motor was wired oppositely
    
      // turning
  //    driveRightWheels.motor(1, -rightStickLeftRight);
  //    driveRightWheels.motor(2, rightStickLeftRight);
  //    
  //    driveLeftWheels.motor(1, -rightStickLeftRight);
  //    driveLeftWheels.motor(2, rightStickLeftRight); 
    }
  
    digitalWrite(beltSpinPin, LOW);
    if(buttonStart == 1) {
      digitalWrite(beltSpinPin, HIGH); // reverse 
    }
    analogWrite(2, rightTrigger); // Trigger turns belt
  }
}


void plunge(int val) {
  plunge(val, val);
}
void plunge(int left, int right) {
  left1LinearActuator2Plunge.motor(2, left); 
  right2LinearActuator1Plunge.motor(1, right);
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

  float tiltPos = 30;
  if (mineTilt) {
    tiltPos = 38;
  }
  else if (transportationTilt) {
    tiltPos = 20;
  }
  else if (dumpTilt) {
    tiltPos = 80;
  }
  rightTilt.data = tiltPos;
  leftTilt.data = tiltPos;
  autonomyLeftLinearActuator.publish( &leftTilt );
  autonomyRightLinearActuator.publish( &rightTilt );

  moveLinearActuators(leftLinearActuatorEffCmd, rightLinearActuatorEffCmd);
}
// mining orientation 38
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
