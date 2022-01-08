/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  340  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2510 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// stepper
const int delay_stepper = 600;

// our servo # counter
uint8_t servonum = 0;
int potpin1 = 0;
int potpin2 = 1;
int potpin3 = 2;
int val1;
int val2;
int val3;

const int left_servo = 1;
const int right_servo = 15;
const int servo_resolution = 200;

bool execute_servo = true; // when false doesnt execute servos
bool servo_enabled[4] = {true,true,true,true};
int vals[3];
// For serial communication with pc
#define INPUT_SIZE 30
char input[INPUT_SIZE + 1];

void setup() {
  Serial.begin(115200);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency.
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}
void write_servo(int servo_id, int val){
  if (execute_servo and servo_enabled[servo_id]){
    pwm.writeMicroseconds(servo_id, val);
  }
}
void read_and_set_servo(){
    int servos[3] = {0,0,0};
    char * pch;
    byte size = Serial.readBytes(input, INPUT_SIZE);
    input[size] = 0;
    char* command = strtok(input, ",");
    Serial.print("command");Serial.println(command);
    //   Sent commands using 0:0    Which is equal to id:value
    while (command != 0)
    {

      // Split the command in two values
      char* separator = strchr(command, ':');
      if (separator != 0)
      {
          // Actually split the string in 2: replace ':' with 0
          *separator = 0;
          int joint_id = atoi(command);
          if (servos[joint_id] == 1){
            continue;
          }

          servos[joint_id] = 1;
          ++separator;
          int pos = atoi(separator);

          val1 = map(pos, 0, servo_resolution, SERVOMIN, SERVOMAX);
//          write_servo(joint_id, val1);


//        First joint has two servos. The second one gets a reversed signal.
          if (joint_id == 0){
             write_servo(left_servo, val1); // left servo
             int right_val = map(servo_resolution-pos, 0, servo_resolution, SERVOMIN, SERVOMAX);
             write_servo(right_servo, right_val); // right servo
          }
          else{
            if (joint_id == 2){ // revert third joint angle
              int reversed_pos = servo_resolution -pos;
              int temp_val = map(reversed_pos, 0, servo_resolution, SERVOMIN, SERVOMAX);
              write_servo(joint_id+1, temp_val);

              Serial.print("reversed_pos");Serial.println(reversed_pos);
              Serial.print("temp_val");Serial.println(temp_val);
              Serial.print("pos");Serial.println(pos);
              Serial.print("jointid");Serial.println(joint_id);
            }
            else{
             write_servo(joint_id+1, val1);
           }
          }
          // Do something with servoId and position
      }
      // Find the next command in input string
      command = strtok(0, ",");
    }
    serialFlush();
}

int start = 1;
bool tune_start = false;
int check_ending = 0;
void loop() {
 if (!tune_start and Serial.available()){

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
  read_and_set_servo();
 }

  int pot1 = analogRead(potpin1);
  int pot2 = analogRead(potpin2);
//  val3 = analogRead(potpin3);
//
//
if (tune_start){

    check_ending = 1023 - check_ending; // be carefull using this switch actions

    int q1 = map(pot1, 0, 1023, 0, servo_resolution);
    Serial.print("reversed q1: ");Serial.println(servo_resolution-q1);
    Serial.print("normal q1: ");Serial.println(q1);
    int start_pos1 = map(q1, 0, servo_resolution, SERVOMIN, SERVOMAX);
    int start_pos2 = map(1023-pot1, 0, 1023, SERVOMIN, SERVOMAX);
//   Serial.println(start_pos1);
    //int start_pos3 = map(check_ending, 0, 1023, SERVOMIN, SERVOMAX); // for simulating boundary signals
    int start_pos3 = map(pot2, 0, 1023, SERVOMIN, SERVOMAX);

    int start_pos4 = map(1023-pot2, 0, 1023, SERVOMIN, SERVOMAX);
   // write_servo(left_servo, start_pos1);
   // write_servo(right_servo, start_pos2);
    write_servo(3, 340);
//    write_servo(3, start_pos4);
//
//   Serial.print("pot1: ");Serial.println(pot1);
//   Serial.print("pot2: ");Serial.println(pot2);

   Serial.print("start_pos1: ");Serial.println(start_pos1);
   Serial.print("start_pos3: ");Serial.println(start_pos3);
   delay(500);
 }
//
}