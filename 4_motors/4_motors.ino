//Code for simutaniously running 4 motors on 2 motor drivers. Single arduino.

#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial(2,3);   // RX on pin 2 (to S2), TX on pin 3 (to S1)              Drive Motors
SoftwareSerial SWSerial1(4, 3); // RX on pin 4 (to S2), TX on pin 3 (to S1)             Drive Motors


Sabertooth ST(128, SWSerial);
Sabertooth ST1(128, SWSerial1);


//const int b2 = 2;
double forward;
double right;

void setup() {
  delay(2000);

  //pinMode (b1, INPUT);
  //pinMode (b2, INPUT);
  
  SabertoothTXPinSerial.begin(9600);

  SWSerial.begin(9600);    // Serial for the motor controller
  SWSerial.begin(9600);

  delay(100);
  
  ST.drive(0); // Set all motors to zero initially
  //ST.turn(0);
  ST1.drive(0);
  //ST1.turn(0);

}

void loop(){
  int b1 = Serial.read()-'0';
  forward = map(b1, 0, 1, 0, 125);
  //right = map(b2, 0, 1, 0, 125);
  


  ST.drive(b1);
  //ST.turn(b2);
  ST1.drive(b1);
  //ST1.turn(b2);

  delay(20);

}
