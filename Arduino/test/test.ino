 /*
  Rotary Encoder Demo
  rot-encode-demo.ino
  Demonstrates operation of Rotary Encoder
  Displays results on Serial Monitor
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
 // Rotary Encoder Inputs
 #define inputCLK1 4
 #define inputDT1 5

 int counter = 0; 
 int currentStateCLK1;
 int previousStateCLK1; 


 void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (inputCLK1,INPUT);
   pinMode (inputDT1,INPUT);
   
   
   // Setup Serial Monitor
   Serial.begin (9600);
   
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK1 = digitalRead(inputCLK1);

 } 

 void loop() {
  counter = count(counter);
 }

 int count(int counter){
  currentStateCLK1 = digitalRead(inputCLK1);
  if (currentStateCLK1 != previousStateCLK1){ 
    if (digitalRead(inputDT1) != currentStateCLK1) {
      counter --;
    }
    else {
      counter ++;
    }
   Serial.println(counter);
  }
  previousStateCLK1 = currentStateCLK1;
  return counter;
 }
