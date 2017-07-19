/*                                                                                                                                          /*
 
 Puzzlebox - Brainstorms - RC Car - Arduino Control
 
 Copyright Puzzlebox Productions, LLC (2011)
 
 Modified by Purdue Undergraduate Neuroscience Society Spring 2017

int DEBUG = 2;

// Initializes INPUT PIN locations
int rightEMGPin = A1;
int leftEMGPin = A0;

// Initializes OUTPUT PIN locations
int rightPIN = 13;
int leftPIN = 12;
int forwardPIN = 2;

// Initializes turning variables
int leftOutput = 0; 
int leftEMGValue = 0;
bool leftBool = false;
int rightOutput = 0; 
int rightEMGValue = 0;
bool rightBool = false;
int neutralValueSteering = 200; // neutral voltage
int steerValue = 1000;          // arbitrary number above 200

// Initializes EEG input variables
char inputByte = '0';
char inputSign = '+';
char inputEEGString[4] = "000";
int inputEEG = 0;
char serialDelimiter = '!'; 

void setup() {
  
  // Declare right, left, forward pins as output
  pinMode(rightPIN, OUTPUT);
  pinMode(leftPIN, OUTPUT);
  pinMode(forwardPIN, OUTPUT);
  
  // Start output pins off
  digitalWrite(rightPIN, LOW);
  digitalWrite(leftPIN, LOW);
  digitalWrite(forwardPIN, HIGH);

  // Intialize the serial monitor
  if (DEBUG >= 1) {
    Serial.begin(9600);
  }

  // Starts car moving straight
  steeringControl(neutralValueSteering, neutralValueSteering);

} // setup


void driveControl() {
  // Input EEG reading
  inputEEG = readSerialInput(serialDelimiter);

  // Determines if car should move forward or stop based on EEG
  if(inputEEG >= 15){
    digitalWrite(forwardPIN, LOW);     // Forward
  }
  else {
    digitalWrite(forwardPIN, HIGH);      // Stop
  }
  
} // driveControl


int readSerialInput(char delimiter) {
  // Convert EEG reading into integer
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
  inputEEGString[0] = inputByte - 48; // ASCII conversion
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputEEGString[1] = inputByte - 48;
  while(Serial.available() == 0);
  inputByte = Serial.read();
  inputEEGString[2] = inputByte - 48;
  
  inputEEG = int(inputEEGString[0]) * 100 + \
               int(inputEEGString[1]) * 10 + \
               int(inputEEGString[2]);
  
  if (inputSign == '-')
    inputEEG = -inputEEG;
  
  return(inputEEG);
  
} // readSerialInput


void readEMGSensors() {
  // Reads EMG sensors
  leftEMGValue = analogRead(leftEMGPin);
  rightEMGValue = analogRead(rightEMGPin);
  
} // readAnalogSensor


void calculateSteeringDirection() {
  // Determines which direction car should turn
  if (leftEMGValue > neutralValueSteering)
  {
    leftBool = true;
    leftOutput = steerValue;
  }
  
  if (rightEMGValue > neutralValueSteering)
  {
    rightBool = true;
    rightOutput = steerValue;
  }
  if (leftBool && rightBool)
  {
    leftBool = false;
    rightBool = false;
    leftOutput = neutralValueSteering;
    rightOutput = neutralValueSteering;
  }
  
} // calculateSteeringDirection


void steeringControl(int leftOutput, int rightOutput) {
  // Turns the car
  if (rightOutput == leftOutput) {    // Straight
    digitalWrite(rightPIN, LOW);
    digitalWrite(leftPIN, LOW);
  }
  if (rightOutput > leftOutput) {     // Right
    digitalWrite(leftPIN, LOW);
    digitalWrite(rightPIN, HIGH);
  }
  if (rightOutput < leftOutput) {     // Left
    digitalWrite(leftPIN, HIGH);
    digitalWrite(rightPIN, LOW);
  }
  
} // steeringControl


void loop() {
  // Continuous drive/brake control
  driveControl();

  // Continuous turn control
  leftBool = false;
  leftOutput = neutralValueSteering;
  rightBool = false;
  rightOutput = neutralValueSteering;
  
  readEMGSensors();
  calculateSteeringDirection();
  steeringControl(leftOutput, rightOutput);
  
  // Stop car after 300 milliseconds -- trial and error solution
  delay(300);
  //digitalWrite(forwardPIN, HIGH);

} // loop
