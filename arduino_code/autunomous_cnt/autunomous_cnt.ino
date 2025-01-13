const int leftMotorPin = 6;   // PWM pin for left motor
const int rightMotorPin = 5; // PWM pin for right motor
unsigned long lastCommandTime = 0; // Track the time of the last received command
const unsigned long timeoutDuration = 100; // Timeout duration in milliseconds
int left_pwm;
int right_pwm;

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming command string
    String command = Serial.readStringUntil('\n');
    
    // Parse the left and right PWM values
     left_pwm = ((command.substring(1, command.indexOf("R")).toInt()));
     right_pwm = ((command.substring(command.indexOf("R") + 1).toInt()));

    lastCommandTime = millis();

  }

  if (millis() - lastCommandTime > timeoutDuration) {
    // If timeout occurs, stop the motors
    left_pwm = 0;
    right_pwm = 0;
  }

    if (left_pwm <= 46) {
      analogWrite(leftMotorPin, 0);
    }
    else {
    analogWrite(leftMotorPin, left_pwm); 
    }

    if (right_pwm <= 46) {
      analogWrite(rightMotorPin, 0);
    }
    else {
    analogWrite(rightMotorPin, right_pwm); 
    }
}
