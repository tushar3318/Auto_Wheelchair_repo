// Input pins
const int pin1 = 2; // PWM input pin 1
const int pin2 = 3; // PWM input pin 2

// Output pins
const int pin3 = 5; // PWM output pin 1
const int pin4 = 6; // PWM output pin 2

// PWM threshold
const float voltageThreshold = 3.0;
const int pwmValue = 40;

void setup() {
  // Configure input pins
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);

  // Configure output pins
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  // Initialize outputs to 0
  analogWrite(pin3, 0);
  analogWrite(pin4, 0);
}

void loop() {
  // Read analog values from the input pins
  int signal1 = analogRead(pin1); 
  int signal2 = analogRead(pin2); 

  // Convert analog values to voltage
  float voltage1 = signal1 * (5.0 / 1023.0); // Assuming 5V reference
  float voltage2 = signal2 * (5.0 / 1023.0); // Assuming 5V reference

  // Determine states based on the voltage threshold
  bool state1 = (voltage1 > voltageThreshold);
  bool state2 = (voltage2 > voltageThreshold);

  // Check the state combinations and act accordingly
  if (state1 && state2) { // GO
    analogWrite(pin3, pwmValue);
    analogWrite(pin4, pwmValue);
  } else if (!state1 && !state2) { // STOP
    analogWrite(pin3, 0);
    analogWrite(pin4, 0);
  } else if (state1 && !state2) { // RIGHT
    analogWrite(pin3, pwmValue);
    analogWrite(pin4, 0);
  } else if (!state1 && state2) { // LEFT
    analogWrite(pin3, 0);
    analogWrite(pin4, pwmValue);
  } else { // No valid command
    analogWrite(pin3, 0);
    analogWrite(pin4, 0);
  }
}