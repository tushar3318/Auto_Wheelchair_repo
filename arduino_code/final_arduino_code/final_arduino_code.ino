///joystick pin signal ;main input signal ;analog read 
 double x_pin =A0;
 double y_pin =A1;

//analog input for voice control
double x_pin_v =A2;
 double y_pin_v =A3;

 // right motor drive pins ;this is output pwm output 
 int R_right=3;
 int L_right=5;

  
 //lift motor 
 int R_left=6;
 int L_left=9;
  
 // forward moment values
 int x_start_fwd=540;
 int x_end_fwd=1023;
 //backward motion limits 
 int x_start_back=490;
 int x_end_back=0;

 //right turn
  int y_start_right =540;
 int y_end_right =1023;

//left  turn
  int y_start_left =490;
 int y_end_left=0;

//selection pins 
const int slct1 = 7;
const int slct2 = 2;


// Scaling factors for magnitude adjustment
float scaleFactor1 = 0.42;    // Scale factor for first input (50% of original)
float scaleFactor2 = 0.42;    // Scale factor for second input (150% of original)

void setup() {
  // serial comm 
   Serial.begin(9600);
 pinMode(R_right,OUTPUT);
  pinMode(L_right,OUTPUT);
   pinMode(R_left,OUTPUT);
    pinMode(L_left,OUTPUT);
 
 // analog input pins  for js
pinMode(x_pin,INPUT);
pinMode(y_pin,INPUT);

// analog input pins for voice
pinMode(x_pin_v,INPUT);
pinMode(y_pin_v,INPUT);
 
 // selection pins 
  pinMode(slct1, INPUT);
  pinMode(slct2, INPUT);

 
   
}

void loop() {
  // Read the state of the selector pins
  int state1 = digitalRead(slct1);
  int state2 = digitalRead(slct2);


// joystick mode 
  if (state1 == HIGH && state2 == HIGH) {
     //receive the analog inputs from wiper or joystick or A0 and A1 reading 
  double x_data =analogRead(x_pin );
    double y_data =analogRead(y_pin );
    // Serial.print("Y data :");
    // Serial.println(y_data);
    //speed mapping _x_axis 
    int speed_fwd=map(x_data,x_start_fwd,x_end_fwd,0,80);
    int speed_back=map(x_data,x_start_back,x_end_back,0,80);

    // speed mapping _y_axis 
    int turn_right=map(y_data,y_start_right, y_end_right,0,80);
    int turn_left=map(y_data,y_start_left, y_end_left,0,80);

     // condition 

     if(y_data > y_start_right && x_data<x_start_back && x_data>x_start_fwd  ){
      analogWrite(R_right,speed_fwd-turn_right);
     
      analogWrite(R_left,speed_fwd+turn_right);
    
     }
    else  if(y_data > y_start_right && x_data>x_start_back && x_data<x_start_fwd  ){
      analogWrite(R_right,0);
     
      analogWrite(R_left,80);
    
     }
      //left
      else if(y_data <  y_start_left && x_data<x_start_back && x_data>x_start_fwd ){
      analogWrite(R_right,speed_fwd+turn_left);
  
      analogWrite(R_left,speed_fwd-turn_left);
 
     }

      else if(y_data <  y_start_left && x_data>x_start_back && x_data<x_start_fwd ){
      analogWrite(R_right,80);
  
      analogWrite(R_left,0);
 
     }
     //foward condition 
     else if (x_data > x_start_fwd){
      analogWrite(R_right,speed_fwd); //3
      analogWrite(L_right,0);
      analogWrite(R_left,speed_fwd);  //6
      analogWrite(L_left,0);
  
     }
     //backward condition

      else if(x_data < x_start_back){
        
      analogWrite(R_right,0);
      analogWrite(L_right,speed_back); //5
      analogWrite(R_left,0);
      analogWrite(L_left,speed_back);  //9

     }
      else {
      analogWrite(R_right,0);
      analogWrite(L_right,0 );
      analogWrite(R_left,0);
      analogWrite(L_left,0);
     }
  } 


  // voice control mode 
  else if (state1 == HIGH && state2 == LOW) {
   // Read analog inputs
  // Write adjusted values to PWM pins
  analogWrite(R_left, 0);
  analogWrite(R_right, 0);

  int input1 = analogRead(x_pin_v);
  int input2 = analogRead(y_pin_v);

  // Adjust the magnitude using scale factors
  int output1 = input1 * scaleFactor1;
  int output2 = input2 * scaleFactor2;

  // Map analog values to PWM range (0-255)
  output1 = constrain(map(output1, 0, 1023, 0, 255), 0, 255);
  output2 = constrain(map(output2, 0, 1023, 0, 255), 0, 255);

  // Write adjusted values to PWM pins
  analogWrite(R_left, output2);
  analogWrite(R_right, output1);
  
  // Small delay for stability
  delay(10);


// autonomous mode 
  }
  else if (state1 == LOW && state2 == HIGH) {
    //digitalWrite(pin3, HIGH);
    //digitalWrite(pin2, LOW);
    //digitalWrite(pin1, LOW);
  }
  else if (state1 == LOW && state2 == LOW) {
    analogWrite(R_right,0);
      analogWrite(L_right,0 );
      analogWrite(R_left,0);
      analogWrite(L_left,0);
  }
}