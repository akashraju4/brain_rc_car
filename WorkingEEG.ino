/*                                                                                                                                          /*
 
 
 Puzzlebox - Brainstorms - RC Car - Arduino Control
 
 Copyright Puzzlebox Productions, LLC (2011)
 
 Modified by Purdue Undergraduate Neuroscience Society Spring 2017
 
 */

int DEBUG = 2;

int ledPin = A0;         // select the pin for the LED

char serialDelimiter = '!'; // character to specify the boundary between separate, 
                            // independent regions in data stream
char axisDelimiter = ',';   // character to specify the boundary between throttle and steering

int delayValue = 5000;   // loop delay in milliseconds
//int delayMultiplier = (float)62500 / (float)976.5625;
//int delayMultiplier = (float)31250 / (float)488.28125;
int delayMultiplier = 1;

int sensorPin = A0;      // select the input pin for the potentiometer
int sensorValue = 0;     // variable to store the value coming from the sensor
int testSensorPin = A1;  // input pin for simulation testing output PWM
int testSensorValue = 0; // variable to test the value coming from the output pin

int outputValueThrottle = 0; // variable to store the throttle output PWM value
int outputPinThrottle = 9;   // PWM output pin
float outputVoltageThrottle = 0.0;

int outputValueSteering = 0; // variable to store the steering output PWM value
int outputPinSteering = 10;  // PWM output pin
float outputVoltageSteering = 0.0;

//int neutralValueThrottle = 351; // neutral 1.64v (Arduino 353-356)
int neutralValueThrottle = 342; // neutral 1.64v (Arduino 353-356)
int forwardValue = 415;         // forward 1.91v (Arduino 415-418)
int reverseValue = 312;         // reverse 1.47v (Arduino 311-314)
int neutralValueSteering = 342; // neutral 1.64v
int leftValue = 420;            // left    1.20v
int rightValue = 310;           // right   2.05v

char inputByte = '0';
char inputSign = '+';
char inputValueString[4] = "000";
int inputValue = 0;

float ratio;

void setup() {
  
  // NOTE: Timer defaults are 0x04 - 976.5625, 488.28125, 488.28125 respectively
  // Set Pins 5 & 6 (Timer0) frequency to 976.5625Hz or 62.5Khz
  TCCR0B = TCCR0B & 0b11111000 | 0x04; // 976.5625Hz
  //TCCR0B = TCCR0B & 0b11111000 | 0x01; // 62.5Khz
  // Set Pins 9 & 10 (Timer1) frequency to 488.28125Hz or 31.25Khz
  //TCCR1B = TCCR1B & 0b11111000 | 0x04; // 488.28125Hz
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // 31.25Khz
  // Set Pins 11 & 3 (Timer2) frequency to 488.28125Hz or 31.25Khz
  TCCR2B = TCCR2B & 0b11111000 | 0x04; // 488.28125Hz
  //TCCR2B = TCCR2B & 0b11111000 | 0x01; // 31.25Khz
  
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  
  // turn the ledPin off
  digitalWrite(ledPin, LOW);

  // intialize the Serial Monitor
  if (DEBUG >= 1)
    Serial.begin(9600);

  // set default throttle value to neutral
  if (DEBUG > 1) {
    Serial.println("\nSetting Throttle Neutral");
    Serial.println("Setting Steering Neutral");
  }
  
  setPotentiometerValues(neutralValueThrottle, neutralValueSteering);

} // setup


void displayOutputValues() {
  // print debug information to console with throttle and steering values
  if (DEBUG > 1) {
    Serial.print("Throttle value: ");
    Serial.print("[PWM:");
    Serial.print(outputValueThrottle);
    Serial.print("] [");
    Serial.print(outputVoltageThrottle);
    Serial.println("V]");
    
    Serial.print("Steering value: ");
    Serial.print("[PWM:");
    Serial.print(outputValueSteering);
    Serial.print("] [");
    Serial.print(outputVoltageSteering);
    Serial.println("V]\n");
  }

} // displayOutputValues


void setPotentiometerValues(int throttle, int steering) {
  // set default throttle value to neutral
  outputValueThrottle = map(throttle, 0, 1023, 0, 255);
  outputVoltageThrottle = ((float)throttle / (float)1023 * 5);
  
  // set default steering value to neutral
  outputValueSteering = map(steering, 0, 1023, 0, 255);
  outputVoltageSteering = ((float)steering / (float)1023 * 5);

  displayOutputValues();
  
  analogWrite(outputPinThrottle, outputValueThrottle);
  analogWrite(outputPinSteering, outputValueSteering);
  
} // setPotentiometerValues


int readSerialInput(char delimiter) {
  // Drive RC Car by Serial input
  while(Serial.available() == 0); // pause until a byte is received by serial Rx
  inputByte = Serial.read(); // reads !
  
  while(inputByte != delimiter) {
    while(Serial.available() == 0);
    inputByte = Serial.read(); 
  }
  
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputSign = inputByte;
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputValueString[0] = inputByte - 48; // ASCII conversion
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputValueString[1] = inputByte - 48;
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputValueString[2] = inputByte - 48;
  
  inputValue = int(inputValueString[0]) * 100 + \
               int(inputValueString[1]) * 10 + \
               int(inputValueString[2]);
  
  if (inputSign == '-')
    inputValue = -inputValue;
  
  return(inputValue);
  
} // readSerialInput


void updateRemoteControl() {
  // read control settings from serial input and set corresponding potentiometer values
  inputValue = readSerialInput(serialDelimiter);

  if(inputValue >= 15){
    digitalWrite(ledPin, HIGH);
  }
  
  else {
    digitalWrite(ledPin, LOW);
  }
  
  
} // updateRemoteControl


int calculateThrottlePower(int value) {
  // take a power input setting of 0 to 100 for forward or -100 to 0 for reverse
  if (value >=0) {
    value = forwardValue;
  } else {
    value = reverseValue;
  }

  return(value);
  
} // calculateThrottlePower


int calculateSteeringPower(int value) {
  // take a power input setting of 0 to 100 for left or -100 to 0 for right
  if (value >=0) {
    value = leftValue;
  } else {
    value = rightValue;
  }

  return(value);

} // calculateSteeringPower





void loop() {

  updateRemoteControl();
  // stop the program for <delayValue> milliseconds:
  if (DEBUG >= 1) {
    delay(500);
  }
  digitalWrite(ledPin, LOW);
  // print an empty line between loops
  if (DEBUG > 1) {
    Serial.println();
  }

} // loop
