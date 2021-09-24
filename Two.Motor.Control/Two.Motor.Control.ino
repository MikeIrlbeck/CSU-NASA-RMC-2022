// Jolty Sample for Packet Serial
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Sabertooth.h>

Sabertooth ST(128); // The Sabertooth is on address 128. We'll name its object ST.
                    // If you've set up your Sabertooth on a different address, of course change
                    // that here. For how to configure address, etc. see the DIP Switch Wizard for
                    //   Sabertooth - http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                    //   SyRen      - http://www.dimensionengineering.com/datasheets/SyrenDIPWizard/start.htm
                    // Be sure to select Packetized Serial Mode for use with this library.
                    //
                    // On that note, you can use this library for SyRen just as easily.
                    // The diff-drive commands (drive, turn) do not work on a SyRen, of course, but it will respond correctly
                    // if you command motor 1 to do something (ST.motor(1, ...)), just like a Sabertooth.
                    //
                    // In this sample, hardware serial TX connects to S1.
                    // See the SoftwareSerial example in 3.Advanced for how to use other pins.
                                        
// Define pins
int button1 = 2;
int button2 = 4;


void setup()
{
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud(); // Send the autobaud command to the Sabertooth controller(s).
                 // NOTE: *Not all* Sabertooth controllers need this command.
                 //       It doesn't hurt anything, but V2 controllers use an
                 //       EEPROM setting (changeable with the function setBaudRate) to set
                 //       the baud rate instead of detecting with autobaud.
                 //
                 //       If you have a 2x12, 2x25 V2, 2x60 or SyRen 50, you can remove
                 //       the autobaud line and save yourself two seconds of startup delay.
    pinMode(button1, INPUT);
    pinMode(button2, INPUT);
}


void loop()
{
  if(digitalRead(button1) == HIGH) {
  throttle(1, 0, 127, 10);  // Go forward at full power.
  throttle(1, 127, 0, 10);    // Stop.
  }
  else if(digitalRead(button2) == HIGH) {
  throttle(2, 0, 127,10);  // Go forward at full power.
  throttle(2, 127, 0, 10);    // Stop.
  }
  
}

void throttle(int Motor, int Begin, int End, int Time) {
    for(int i = Begin; i <= End; i++) {
        ST.motor(Motor,i);
        delay(Time); // in ms
    }
}
