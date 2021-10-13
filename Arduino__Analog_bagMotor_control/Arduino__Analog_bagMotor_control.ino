// Jolty Sample for Packet Serial
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Sabertooth.h>
#include <SabertoothSimplified.h>

const int X_pin = 1; // A1 pin connected to X output
const int Y_pin = 0; // A0 pin connected to Y output

Sabertooth ST(128); // The Sabertooth is on address 128.
                                     
void setup()
{
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud();

  //sets the drive and turn states to 0 so the motors start at 0
  ST.drive(0);
  ST.turn(0);

}


void loop()
{

  // makes 2 integer values that read the x and y pins position
  int xVal = analogRead(X_pin);
  int yVal = analogRead(Y_pin);

  
  //maps input values of analog stick, which range from 0 to 1023, to values that the motor driver reads, which is -127 to 127
  int rl = map(xVal, 0, 1023, -127, 127);
  int bf = map(yVal, 0, 1023, -127, 127);

  //has the motor drivers read the mapped values sent from rl and bf
  ST.turn(rl);
  ST.drive(bf);
}
