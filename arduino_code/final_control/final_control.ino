// Pin Definitions
const int leftMotorPin = 9;   // PWM pin for left motor
const int rightMotorPin = 10; // PWM pin for right motor
const int selectPin = 2;      // Pin to select control mode
const int x_pin = A0;         // Joystick X-axis input
const int y_pin = A1;         // Joystick Y-axis input

// Joystick Motion Ranges
const int x_start_fwd = 540;
const int x_end_fwd = 1023;
const int x_start_back = 490;
const int x_end_back = 0;
const int y_start_right = 540;
const int y_end_right = 1023;
const int y_start_left = 490;
const int y_end_left = 0;

void setup() {
  // Setup pins
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(selectPin, INPUT);
  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);

  Serial.begin(9600); // Initialize Serial communication
}

void loop() {
  // Read the control mode from the selection pin
  int controlMode = digitalRead(selectPin);

  if (controlMode == HIGH) {
    // Joystick Control
    joystickControl();
  } else {
    // cmd_velocity Control
    cmdVelocityControl();
  }
}

void joystickControl() {
  // Read joystick data
  double x_data = analogRead(x_pin);
  double y_data = analogRead(y_pin);

  // Map joystick values to motor speeds
  int speed_fwd = map(x_data, x_start_fwd, x_end_fwd, 0, 80);
  int speed_back = map(x_data, x_start_back, x_end_back, 0, 80);
  int turn_right = map(y_data, y_start_right, y_end_right, 0, 80);
  int turn_left = map(y_data, y_start_left, y_end_left, 0, 80);

  // Determine movement based on joystick position
  if (y_data > y_start_right && x_data < x_start_back && x_data > x_start_fwd) {
    // Right turn
    analogWrite(leftMotorPin, speed_fwd + turn_right);
    analogWrite(rightMotorPin, speed_fwd - turn_right);
  } else if (y_data < y_start_left && x_data < x_start_back && x_data > x_start_fwd) {
    // Left turn
    analogWrite(leftMotorPin, speed_fwd - turn_left);
    analogWrite(rightMotorPin, speed_fwd + turn_left);
  } else if (x_data > x_start_fwd) {
    // Forward motion
    analogWrite(leftMotorPin, speed_fwd);
    analogWrite(rightMotorPin, speed_fwd);
  } else if (x_data < x_start_back) {
    // Backward motion
    analogWrite(leftMotorPin, speed_back);
    analogWrite(rightMotorPin, speed_back);
  } else {
    // Stop
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
  }
}

void cmdVelocityControl() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    // Read the incoming command string
    String command = Serial.readStringUntil('\n');

    // Parse left and right motor speeds
    int left_pwm = ((command.substring(1, command.indexOf("R")).toInt()) * 0.313);
    int right_pwm = ((command.substring(command.indexOf("R") + 1).toInt()) * 0.313);

    // Set motor speeds
    analogWrite(leftMotorPin, left_pwm);
    analogWrite(rightMotorPin, right_pwm);
  } else {
    // Stop motors if no command is received
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
  }
}

