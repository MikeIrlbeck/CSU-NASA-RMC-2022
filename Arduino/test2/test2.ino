 /*
  Rotary Encoder Demo
  rot-encode-demo.ino
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
 // Rotary Encoder Inputs
 #define channelB 4
 #define channelA 5

 int counter1 = 0; 
 int currentStateB;
 int previousStateB; 


 void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (channelB,INPUT);
   pinMode (channelA,INPUT);
   
   
   // Setup Serial Monitor
   Serial.begin (9600);
   
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateB = digitalRead(channelB);

 } 

 void loop() {
  counter1 = count(counter1);
 }

 int count(int counter){
  currentStateB = digitalRead(channelB);
  if (currentStateB != previousStateB){ 
    if (digitalRead(channelA) != currentStateB) {
      counter --;
    }
    else {
      counter ++;
    }
   Serial.println(counter);
  }
  previousStateB = currentStateB;
  return counter;
 }
