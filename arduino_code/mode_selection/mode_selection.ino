const int pin1 = 12;
const int pin2 = 8;
const int pin3 = 4;
const int analog1 = A0;
const int analog2 = A1;
const int slct1 = 7;
const int slct2 = 2;

void setup() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(analog1, INPUT);
  pinMode(analog2, INPUT);
  pinMode(slct1, INPUT);
  pinMode(slct2, INPUT);
}

void loop() {
  // Read the state of the selector pins
  int state1 = digitalRead(slct1);
  int state2 = digitalRead(slct2);

  if (state1 == HIGH && state2 == HIGH) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
  } 
  else if (state1 == HIGH && state2 == LOW) {
    digitalWrite(pin2, HIGH);
    digitalWrite(pin1, LOW);
    digitalWrite(pin3, LOW);
  }
  else if (state1 == LOW && state2 == HIGH) {
    digitalWrite(pin3, HIGH);
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, LOW);
  }
  else if (state1 == LOW && state2 == LOW) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
  }
}