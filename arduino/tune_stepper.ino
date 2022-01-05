/*     Simple Stepper Motor Control Exaple Code
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
// defines pins numbers
const int stepPin = 3; 
const int dirPin = 4; 
const int delay_st = 620; 
int delay_s; 

 int potpin1 = 0;
int val1;

void setup() {
  Serial.begin(115200);
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void loop() {
     val1 = analogRead(potpin1);
   val1 = map(val1, 0, 1023, -200, 200);
  delay_s = val1+ delay_st;
  
   Serial.println(delay_s);
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 50; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(delay_s); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(delay_s); 
  }
//  delay(500); // One second delay
//  
//  digitalWrite(dirPin,LOW); //Changes the rotations direction
//  // Makes 400 pulses for making two full cycle rotation
//  for(int x = 0; x < 400; x++) {
//    digitalWrite(stepPin,HIGH);
//    delayMicroseconds(delay_s);
//    digitalWrite(stepPin,LOW);
//    delayMicroseconds(delay_s);
//  }
//  delay(500);
}