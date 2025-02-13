// Define analog input pins
const int analogInput1 = A0; // First analog input pin
const int analogInput2 = A1; // Second analog input pin

// Define PWM output pins
const int pwmOutput1 = 9;    // First PWM output pin
const int pwmOutput2 = 10;   // Second PWM output pin

// Scaling factors for magnitude adjustment
float scaleFactor1 = 0.5;    // Scale factor for first input (50% of original)
float scaleFactor2 = 1.5;    // Scale factor for second input (150% of original)

void setup() {
  // Set PWM pins as output
  pinMode(pwmOutput1, OUTPUT);
  pinMode(pwmOutput2, OUTPUT);
}

void loop() {
  // Read analog inputs
  int input1 = analogRead(analogInput1);
  int input2 = analogRead(analogInput2);

  // Adjust the magnitude using scale factors
  int output1 = input1 * scaleFactor1;
  int output2 = input2 * scaleFactor2;

  // Map analog values to PWM range (0-255)
  output1 = constrain(map(output1, 0, 1023, 0, 255), 0, 255);
  output2 = constrain(map(output2, 0, 1023, 0, 255), 0, 255);

  // Write adjusted values to PWM pins
  analogWrite(pwmOutput1, output1);
  analogWrite(pwmOutput2, output2);

  // Small delay for stability
  delay(10);
}