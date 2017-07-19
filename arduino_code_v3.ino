/*                                                                                                                                          /*
 
 Puzzlebox - Brainstorms - RC Car - Arduino Control
 
 Copyright Puzzlebox Productions, LLC (2011)
 
 Modified by Purdue Undergraduate Neuroscience Society Spring 2017
 */
int ledPin = 12;         // Right
int ledPin2 = 13;        // Left
int ledPin3 = 2;         // LED pin for forward

char serialDelimiter = '!'; // character to specify the boundary between separate, 
                            // independent regions in data stream
char axisDelimiter = ',';   // character to specify the boundary between throttle and steering

int leftEMGPin = A0;      // select the input pin for the left EMG
int rightEMGPin = A1;  // input pin for the right EMG

int outputValueThrottle = 0; // variable to store the throttle output PWM value
int outputPinThrottle = 9;   // PWM output pin
float outputVoltageThrottle = 0.0;

int neutralValueThrottle = 342; // neutral 1.64v (Arduino 353-356)

int neutralValueSteering = 150; // neutral voltage
int steerVoltage = 1000; // drive voltage

char inputByte = '0';
char inputSign = '+';
char inputValueString[4] = "000";
int inputValue = 0;

float ratio;

int throttlePower = neutralValueThrottle;

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
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);

  // intialize the Serial Monitor
 
  setPotentiometerValues(neutralValueThrottle, 3);
} // setup


void setPotentiometerValues(int throttle, int steer) {
  // Controls steering
  if (steer == 1) {
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledPin2, LOW);
  }
  else if (steer == 2) {
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin, LOW);
  }  
  else
  {
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin, HIGH);
  }
} // setPotentiometerValues


void updateRemoteControl() {
  // read control settings from serial input and set corresponding potentiometer values
  inputValue = readSerialInput(serialDelimiter);

  // determines if car should move forward or stop based on EEG reading
  if(inputValue >= 0){
    digitalWrite(ledPin3, HIGH);
  }
  else {
    outputValueThrottle = neutralValueThrottle; // Not used
    digitalWrite(ledPin3, LOW);
  }  

  // determines how car should turn based on EMG readings
  bool leftBool = false;
  bool rightBool = false;
  int leftEMGValue = readLeftAnalogSensors();
  int rightEMGValue = readRightAnalogSensors();
  int steer = calculateSteeringPower(leftEMGValue, rightEMGValue);
  
  setPotentiometerValues(outputValueThrottle, steer);
  
} // updateRemoteControl


int calculateSteeringPower(int leftEMGValue, int rightEMGValue) {
  bool leftBool;
  bool rightBool;
  if (leftEMGValue > neutralValueSteering)
  {
    leftBool = true;
  }
  if (rightEMGValue > neutralValueSteering)
  {
    rightBool = true;
  }
  if (leftBool && rightBool)
  {
    if (leftEMGValue > rightEMGValue){
      rightBool = false;
    }
    else if (rightEMGValue > leftEMGValue){
      leftBool = false;
     }
    else{
      leftBool = false;
      rightBool = false;
    }
  }
  if (leftBool){
    return (1);
  }
  else if (rightBool){
    return (2);
  }
  else{
    return (3);
  }
} // calculateSteeringPower


int readLeftAnalogSensors() {
  // read the value from the sensor:
  int leftEMGValue = analogRead(leftEMGPin);
  return (leftEMGValue);  
} // readAnalogSensor


int readRightAnalogSensors(){
  int rightEMGValue = analogRead(rightEMGPin);
  return (rightEMGValue);
}


int readSerialInput(char delimiter) {
  // Drive RC Car by Serial input
  while(Serial.available() == 0); // pause until a byte is received by serial Rx
  inputByte = Serial.read();
  
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


void loop() {
  
  updateRemoteControl();
  
  // stop the program for 300 milliseconds:
  
  outputValueThrottle = neutralValueThrottle;
  digitalWrite(ledPin3, HIGH);
  
  // print an empty line between loops
  
} // loop

