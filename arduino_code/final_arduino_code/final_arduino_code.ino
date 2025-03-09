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
const int x_start_back = 490;
const int x_end_back = 0;
const int y_start_right = 540;
const int y_end_right = 1023;
const int y_start_left = 490;
const int y_end_left = 0;

// Selection Pins
const int slct1 = 7;
const int slct2 = 4;

// Scaling factors for voice control
const float scaleFactor1 = 0.42;
const float scaleFactor2 = 0.42;

// Autonomous Mode Parameters
const unsigned long timeoutDuration = 100; // Timeout for autonomous mode
unsigned long lastCommandTime = 0;
int left_pwm = 0;
int right_pwm = 0;

void setup() {
  Serial.begin(9600);

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
  int state1 = digitalRead(slct1);
  int state2 = digitalRead(slct2);

  // Joystick Mode
  if (state1 == HIGH && state2 == HIGH) {
    double x_data = analogRead(x_pin);
    double y_data = analogRead(y_pin);

    int speed_fwd = map(x_data, x_start_fwd, x_end_fwd, 0, 80);
    int speed_back = map(x_data, x_start_back, x_end_back, 0, 80);
    int turn_right = map(y_data, y_start_right, y_end_right, 0, 80);
    int turn_left = map(y_data, y_start_left, y_end_left, 0, 80);

    if (y_data > y_start_right && x_data < x_start_back && x_data > x_start_fwd) {
      analogWrite(R_right, speed_fwd - turn_right);
      analogWrite(R_left, speed_fwd + turn_right);
    } 
    else if (y_data > y_start_right && x_data > x_start_back && x_data < x_start_fwd) {
      analogWrite(R_right, 0);
      analogWrite(R_left, 80);
    } 
    else if (y_data < y_start_left && x_data < x_start_back && x_data > x_start_fwd) {
      analogWrite(R_right, speed_fwd + turn_left);
      analogWrite(R_left, speed_fwd - turn_left);
    } 
    else if (y_data < y_start_left && x_data > x_start_back && x_data < x_start_fwd) {
      analogWrite(R_right, 80);
      analogWrite(R_left, 0);
    } 
    else if (x_data > x_start_fwd) {  // Forward
      analogWrite(R_right, speed_fwd);
      analogWrite(L_right, 0);
      analogWrite(R_left, speed_fwd);
      analogWrite(L_left, 0);
    } 
    else if (x_data < x_start_back) {  // Backward
      analogWrite(R_right, 0);
      analogWrite(L_right, speed_back);
      analogWrite(R_left, 0);
      analogWrite(L_left, speed_back);
    } 
    else {  // Stop
      analogWrite(R_right, 0);
      analogWrite(L_right, 0);
      analogWrite(R_left, 0);
      analogWrite(L_left, 0);
    }
  } 

  // Voice Control Mode
  else if (state1 == HIGH && state2 == LOW) {
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

  // Autonomous Mode (Receiving Serial Commands)
  else if (state1 == LOW && state2 == HIGH) {
    left_pwm = 0;   // Ensure default to zero
    right_pwm = 0;
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      left_pwm = command.substring(1, command.indexOf("R")).toInt();
      right_pwm = command.substring(command.indexOf("R") + 1).toInt();
      lastCommandTime = millis();
    }

    if (millis() - lastCommandTime > timeoutDuration) {
      left_pwm = 0;
      right_pwm = 0;
    }

    analogWrite(R_left, left_pwm > 62 ? left_pwm : 0);
    analogWrite(R_right, right_pwm > 62 ? right_pwm : 0);
  }

  // Stop Mode (Failsafe)
  else if (state1 == LOW && state2 == LOW) {
    analogWrite(R_right, 0);
    analogWrite(L_right, 0);
    analogWrite(R_left, 0);
    analogWrite(L_left, 0);
  }
}
