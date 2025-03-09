/// Joystick pin signals (Analog Read)
const int x_pin = A0;
const int y_pin = A1;

// Analog input for voice control
const int x_pin_v = A2;
const int y_pin_v = A3;

// Motor drive pins (PWM Output)
const int R_right = 3;
const int L_right = 5;
const int R_left = 6;
const int L_left = 9;

// Motion Thresholds
const int x_start_fwd = 540;
const int x_end_fwd = 1023;
const int y_start_right = 540;
const int y_end_right = 1023;
const int y_start_left = 490;
const int y_end_left = 0;

// Selection Pins
const int slct1 = 7;
const int slct2 = 4;
int state1 = 0;
int state2 = 0;
int state1_prev = 0;
int state2_prev = 0;
bool first_run = true;

// Scaling factors for voice control
const float scaleFactor1 = 0.42;
const float scaleFactor2 = 0.42;

// Autonomous Mode Parameters
const unsigned long timeoutDuration = 100; // Timeout for autonomous mode
unsigned long lastCommandTime = 0;
int left_pwm = 0;
int right_pwm = 0;
int current_left_pwm = 0;
int current_right_pwm = 0;

void setup() {
  pinMode(R_right, OUTPUT);
  pinMode(L_right, OUTPUT);
  pinMode(R_left, OUTPUT);
  pinMode(L_left, OUTPUT);

  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);
  pinMode(x_pin_v, INPUT);
  pinMode(y_pin_v, INPUT);
  pinMode(slct1, INPUT);
  pinMode(slct2, INPUT);
}

void loop() {
  state1_prev = state1;
  state2_prev = state2;
  state1 = digitalRead(slct1);
  state2 = digitalRead(slct2);

  if (state1_prev != state1 && state2_prev != state2) {
    first_run = true;
  }
  
  if (state1 == HIGH && state2 == HIGH) {
    joystickMode();
  } else if (state1 == HIGH && state2 == LOW) {
    voiceControlMode();
  } else if (state1 == LOW && state2 == HIGH) {
    autonomousMode();
  } else {
    stopMotors();
  }
}

// -------------------- MODE FUNCTIONS -------------------- //

void joystickMode() {
  double x_data = analogRead(x_pin);
  double y_data = analogRead(y_pin);

  int speed_fwd = map(x_data, x_start_fwd, x_end_fwd, 0, 80);
  int turn_right = map(y_data, y_start_right, y_end_right, 0, 80);
  int turn_left = map(y_data, y_start_left, y_end_left, 0, 80);

  if (y_data > y_start_right) {  // Turning Right
    analogWrite(R_right, speed_fwd - turn_right);
    analogWrite(R_left, speed_fwd + turn_right);
  } 
  else if (y_data < y_start_left) {  // Turning Left
    analogWrite(R_right, speed_fwd + turn_left);
    analogWrite(R_left, speed_fwd - turn_left);
  } 
  else if (x_data > x_start_fwd) {  // Forward
    moveForward(speed_fwd);
  } 
  else {  // Stop if trying to move backward
    stopMotors();
  }
}

void voiceControlMode() {
  analogWrite(R_left, 0);
  analogWrite(R_right, 0);

  int input1 = analogRead(x_pin_v);
  int input2 = analogRead(y_pin_v);

  int output1 = constrain(map(input1 * scaleFactor1, 0, 1023, 0, 255), 0, 255);
  int output2 = constrain(map(input2 * scaleFactor2, 0, 1023, 0, 255), 0, 255);

  analogWrite(R_left, output2);
  analogWrite(R_right, output1);

  delay(10);
}

void autonomousMode() {
  if (first_run) {
    delay(200);
    first_run = false;
  }
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    int l_index = command.indexOf("L");
    int r_index = command.indexOf("R");

    if (l_index != -1 && r_index != -1 && r_index > l_index) {
      left_pwm = command.substring(l_index + 1, r_index).toInt();
      right_pwm = command.substring(r_index + 1).toInt();
      lastCommandTime = millis();
    }
  }

  if (millis() - lastCommandTime > timeoutDuration) {
    left_pwm = 0;
    right_pwm = 0;
  }

  // Gradual PWM Increase
  if (current_left_pwm < left_pwm) current_left_pwm += 5;
  else if (current_left_pwm > left_pwm) current_left_pwm -= 5;

  if (current_right_pwm < right_pwm) current_right_pwm += 5;
  else if (current_right_pwm > right_pwm) current_right_pwm -= 5;

  analogWrite(R_left, current_left_pwm > 62 ? current_left_pwm : 0);
  analogWrite(R_right, current_right_pwm > 62 ? current_right_pwm : 0);

  delay(10);
}

// -------------------- MOTOR CONTROL FUNCTIONS -------------------- //

void moveForward(int speed) {
  analogWrite(R_right, speed);
  analogWrite(L_right, 0);
  analogWrite(R_left, speed);
  analogWrite(L_left, 0);
}

void stopMotors() {
  analogWrite(R_right, 0);
  analogWrite(L_right, 0);
  analogWrite(R_left, 0);
  analogWrite(L_left, 0);
}
