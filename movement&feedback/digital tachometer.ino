const int LEFT_HALLSENSOR_PIN=2;
const int RIGHT_HALLSENSOR_PIN=3;
const int TIME=100;
volatile long LEFT_PULSE_COUNT;
volatile long RIGHT_PULSE_COUNT;
const float TRANSITION_PER_REVOLUTION=30000/TIME;

void LEFT_COUNTER()
{
  LEFT_PULSE_COUNT++;
}

void RIGHT_COUNTER()
{
  RIGHT_PULSE_COUNT++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEFT_HALLSENSOR_PIN,INPUT_PULLUP);
  pinMode(RIGHT_HALLSENSOR_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_HALLSENSOR_PIN),LEFT_COUNTER,CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_HALLSENSOR_PIN),RIGHT_COUNTER,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

  noInterrupts();
  LEFT_PULSE_COUNT=0;
  RIGHT_PULSE_COUNT=0;
  interrupts();

  delay(TIME);

  noInterrupts();
  long LEFT_PULSE=LEFT_PULSE_COUNT;
  long RIGHT_PULSE=RIGHT_PULSE_COUNT;
  interrupts();

  int LEFT_RPM=((float)LEFT_PULSE)*(TRANSITION_PER_REVOLUTION);
  int RIGHT_RPM=((float)RIGHT_PULSE)*(TRANSITION_PER_REVOLUTION);

  char DATA_SENT[12];
  sprintf(DATA_SENT, "%04d|%04d", LEFT_RPM, RIGHT_RPM);
  Serial.println(DATA_SENT);


}
