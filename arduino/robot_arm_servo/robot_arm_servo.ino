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
#include <Ramp.h>  // requires  RAMP by Sylvain



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
#define SERVOMAX  2505 // This is the 'maximum' pulse length count (out of 4096)
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
int val4;
int val5;

const int action_delay = 600;



const int servo_resolution = 200;

bool execute_servo = true; // when false doesnt execute servos
const int joints = 5;
const int servos = 6;
bool servo_enabled[servos] = {true,true,true,true,false,false}; // keep in mind first 2 servos are joint 1
bool servo_reversed[servos] = {false,true,false,true,false,false};
int servo_pins[servos] = {2,0,4,6,8,10};
int last_servo_val[servos] = {-1,-1,-1,-1,-1,-1};
int last_joint_val[joints] = {-1,-1,-1,-1,-1};
const int reset_time_sec = 3;
int reset_q[joints] = {140,110,80,90,90}; // constant value
bool active = false;
bool resetting= false;


rampInt joints_ramp_objs[joints];
int vals[joints];
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
  turn_off_joints();
  for (int i = 0; i < joints; i++) {
    joints_ramp_objs[i] = rampInt();
    joints_ramp_objs[i].go(reset_q[i],0);// set ramp to reset point
  }
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
  if(servo_id == 0){
    Serial.print("servo_id");Serial.print(servo_id);Serial.print("val");Serial.println(val);
  }
  // Do not send the same value
  if (last_servo_val[servo_id] == val){
    return;
  }
  last_servo_val[servo_id] = val;

  if (servo_reversed[servo_id]){
    val = servo_resolution-val;
  }
  int pulse = map(val, 0, servo_resolution, SERVOMIN, SERVOMAX);
  Serial.print("servo_id");Serial.print(servo_id);Serial.print("val");Serial.println(val);
  if (execute_servo and servo_enabled[servo_id]){
    pwm.writeMicroseconds(servo_pins[servo_id], pulse);
  }
}
void set_joints(int vals[joints]){
  for (int joint_id = 0; joint_id < joints; joint_id++) {
    
    int val = vals[joint_id];
    
    // By setting the servo values in a range between the current position and the goal.
    if (val == -1){

      continue;
    }
    if (val != last_joint_val[joint_id]){
        Serial.print("joint_id");Serial.print(joint_id);
        Serial.print("val_joint");Serial.print(val);
        // todo put speed in variable
        int length_move = abs(joints_ramp_objs[joint_id].getValue() - val)*1000*8/servo_resolution;
        joints_ramp_objs[joint_id].go(val, length_move);       
        active = true;
    }
    last_joint_val[joint_id] = val;
  }
}
void move_servos(){
  for (int joint_id = 0; joint_id < joints; joint_id++) {
//    rampInt joint_ramp = joints_ramp_objs[joint_id];
    int val = joints_ramp_objs[joint_id].update();
    
//    Serial.print("Move joint: ");Serial.println(joint_id);
//    Serial.print("Value start at: ");
//    Serial.println(joint_ramp.getValue());
    //First joint has two servos. The second one gets a reversed signal.
    if (joint_id == 0){
       write_servo(0, val); // left servo
       write_servo(1, val); // right servo
       delay(action);
    }
    else{
       write_servo(joint_id+1, val);
       delay(action_delay);
    }
  }
}
void turn_off_joints(){
  for (int servo_id = 0; servo_id < joints+1; servo_id++) {
    pwm.writeMicroseconds(servo_id, 0);
  }
}
void read_command(int* joint_vals){
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
          // avoid sending multiple commands to same servo
          if (joint_vals[joint_id] != -1){
            continue;
          }
          ++separator;
          joint_vals[joint_id] = atoi(separator);
      }
      // Find the next command in input string
      command = strtok(0, ",");
    }
    serialFlush();
}

bool tune_start = false;
int check_ending = 0;
unsigned long last_command_milli = millis() - reset_time_sec*1000;


void loop() {
  Serial.println("console prints!");
  delay(25); // base delay per loop
  if (!tune_start){
    if(Serial.available()){
      int joint_vals[joints] = {-1,-1,-1,-1,-1};
      read_command(joint_vals);
      Serial.print("joint_vals: ");Serial.println(joint_vals[0]);
      set_joints(joint_vals);
      last_command_milli = millis();
    }
    else{
      if (!resetting and millis() - last_command_milli > reset_time_sec*1000){
        Serial.println("reset: ");
        set_joints(reset_q);        
        resetting=true;        
      }
    }
    if (active){
      move_servos();
    }
    if(resetting){
      if (millis() - last_command_milli > reset_time_sec*1000*3){
          turn_off_joints();
          for(int i=0;i<6;++i){
            last_servo_val[i] = -1;
          }
          active=false;
       }
    }
  }
  int pot1 = analogRead(potpin1);
  int pot2 = analogRead(potpin2);
//  val3 = analogRead(potpin3);
//
//
if (tune_start){

    check_ending = 1023 - check_ending; // BE CAREFULL using this switch actions

    int q1 = map(pot1, 0, 1023, 0, servo_resolution);
    int q2 = map(pot2, 0, 1023, 0, servo_resolution);
//   Serial.println(start_pos1);
    //int start_pos3 = map(check_ending, 0, 1023, SERVOMIN, SERVOMAX); // for simulating boundary signals

    write_servo(0, q1);
//    write_servo(3, start_pos4);
//
//   Serial.print("pot1: ");Serial.println(pot1);
//   Serial.print("pot2: ");Serial.println(pot2);

   Serial.print("q1: ");Serial.println(q1);
   Serial.print("q2: ");Serial.println(q2);
   delay(500);
 }

}
