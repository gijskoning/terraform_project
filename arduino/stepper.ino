/*     Simple Stepper Motor Control Exaple Code
 *
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *
 */
// defines pins numbers
const int stepPin = 3;
const int dirPin = 4;
const int delay_st = 600;
const int speed = 1;

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
}
void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  delayMicroseconds(500);
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(delay_st);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(delay_st);
    delayMicroseconds(delay_st*speed);
  }
  delay(1000); // One second delay

  digitalWrite(dirPin,LOW); //Changes the rotations direction
  delayMicroseconds(500);
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(delay_st);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(delay_st);
  }
  delay(1000);
}