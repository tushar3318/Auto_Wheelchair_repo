///joystick pin signal ;main input signal ;analog read 
 double x_pin =A0;
 double y_pin =A1;
 int rev_pin=4;

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

  void setup() {
  Serial.begin(9600);
 pinMode(R_right,OUTPUT);
  pinMode(L_right,OUTPUT);
   pinMode(R_left,OUTPUT);
    pinMode(L_left,OUTPUT);
    pinMode(rev_pin ,OUTPUT);

pinMode(x_pin,INPUT);
pinMode(y_pin,INPUT);

}

void loop() {
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


     
      //right 
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

        
     digitalWrite(rev_pin,1);
     delay(20);
     digitalWrite(rev_pin,0);

      
 
      
     }
  
 
      else {
      analogWrite(R_right,0);
      analogWrite(L_right,0 );
      analogWrite(R_left,0);
      analogWrite(L_left,0);
     }




    

}