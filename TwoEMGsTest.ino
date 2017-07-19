int LED1 = 1;
int LED2 = 2;
int counter1 = 0;
int counter2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue1 = analogRead(A0);
  int sensorValue2 = analogRead(A1);
  delay(1);

  // First EMG
  if (sensorValue1 >= 700) {
    counter1 = counter1 + 1;
  } else {
    counter1 = 0;
  }

  if (counter1 > 100) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }

  // Second EMG
    if (sensorValue2 >= 700) {
    counter2 = counter2 + 1;
  } else {
    counter2 = 0;
  }

  if (counter2 > 100) {
    digitalWrite(LED2, HIGH);
  } else {
    digitalWrite(LED2, LOW);
  }

  Serial.println("%d    %d", sensorValue1, sensorValue2);
}
