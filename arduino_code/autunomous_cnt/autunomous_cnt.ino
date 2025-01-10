const int leftMotorPin = 9;   // PWM pin for left motor
const int rightMotorPin = 10; // PWM pin for right motor

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
    int left_pwm = ((command.substring(1, command.indexOf("R")).toInt())*0.313);
    int right_pwm = ((command.substring(command.indexOf("R") + 1).toInt())*0.313);

    analogWrite(leftMotorPin, left_pwm);
    analogWrite(rightMotorPin, right_pwm);

}
else{
    analogWrite(leftMotorPin, 0);
    analogWrite(rightMotorPin, 0);
}
}
