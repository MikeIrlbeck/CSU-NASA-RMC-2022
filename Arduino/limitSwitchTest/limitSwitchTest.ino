/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: Calculate the accumulated ticks for each wheel using the 
 * built-in encoder (forward = positive; reverse = negative) 
 */

int plungeLimitpin = 19;
int dumpTopLimitpin = 18;
int dumpBottomLimitpin = 17;

// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
  Serial.println("Begin");
  pinMode(dumpBottomLimitpin, INPUT_PULLDOWN);  // dump bottom
  pinMode(dumpTopLimitpin, INPUT_PULLDOWN); // dump top
  pinMode(plungeLimitpin, INPUT_PULLDOWN); // plunge limit switch
    }

void loop() {
  int bottom = digitalRead(dumpBottomLimitpin);
  int top = digitalRead(dumpTopLimitpin);
  int plunge = digitalRead(plungeLimitpin);

  Serial.println("Bottom: ");
  Serial.println(top);
  Serial.println("\tTop: ");
  Serial.println(bottom);  
  Serial.println("\t\tPlunge: ");
  Serial.println(plunge);

  delay(10);
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
  }
}
