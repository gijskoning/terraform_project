const int ledPin =  LED_BUILTIN;// the number of the LED pin
// Variables will change:
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
#define INPUT_SIZE 30
char input[INPUT_SIZE + 1];

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(ledPin, OUTPUT);
}
int i;
int position;
void read_and_set_servo(){
    char * pch;
    byte size = Serial.readBytes(input, INPUT_SIZE);
    input[size] = 0;
    char* command = strtok(input, ",");
    //   Sent commands using 0:0    Which is equal to id:value
    while (command != 0)
    {
      // Split the command in two values
      char* separator = strchr(command, ':');
      if (separator != 0)
      {
          // Actually split the string in 2: replace ':' with 0
          *separator = 0;
          int servoId = atoi(command);
          ++separator;
          position = atoi(separator);

          if (position == 1) {
              ledState = HIGH;
          }
          if (position == 2) {
              ledState = LOW;
          }
          digitalWrite(ledPin, ledState);
          // Do something with servoId and position
      }
      // Find the next command in input string
      command = strtok(0, "&");
    }
}

void loop() {
//   while (!Serial.available());
  if (Serial.available()){
       read_and_set_servo();
  }
}