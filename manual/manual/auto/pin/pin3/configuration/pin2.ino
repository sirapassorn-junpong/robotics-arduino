/*
   Set Board: Tools --> Board --> Arduino Mega2560
   Set Processor: Tools --> Processor --> ATmega2560
*/



#include <Servo.h> 
#include "configuration.h"

//................Function Declaration..............................
int OutputToMotor1(int value);
int OutputToMotor2(int value);
int OutputToMotor3(int value);
int OutputToMotor4(int value);

double Deadband(double value,double limit);
//................Parameters Declaration............................

int input1 = 0;
int input2 = 0;
int input3 = 0; //Wheel
int input4 = 0; //wheel
int input5 = 0; //LED
int input6 = 0; //servo

int out1 = 0;
int out2 = 0;
int out3 = 0;
int out4 = 0;

float currentValue1 = 0.0;
float currentValue2 = 0.0;
float currentValue3 = 0.0;
float currentValue4 = 0.0;

int left_w = 0;
int right_w = 0;
int buttonState = 0; 

String sCH1;
String sCH2;
String sCH3;
String sCH4;

String inputString = "";
char inChar;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
//..................................................................
float S1_bar = 0.0;
float S2_bar = 0.0;
float S3_bar = 0.0;
float S4_bar = 0.0;

float Tc = 0.1;
float dt = 0.01;
float Tcc = Tc+dt;
int T_lim = 20;
int led_status = 0;

void setup(){
  pinMode(buttonPin,INPUT);
  
  pinMode(CH1,INPUT);   //channel 1
  pinMode(CH2,INPUT);   //channel 2
  pinMode(CH3,INPUT);   //channel 3
  pinMode(CH4,INPUT);   //channel 4
  pinMode(CH5,INPUT);   //channel 5
  pinMode(CH6,INPUT);   //channel 6
  
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  
  pinMode(INA1,OUTPUT);
  pinMode(INA2,OUTPUT);
  pinMode(INA3,OUTPUT);
  pinMode(INA4,OUTPUT);
  
  pinMode(INB1,OUTPUT);
  pinMode(INB2,OUTPUT);
  pinMode(INB3,OUTPUT);
  pinMode(INB4,OUTPUT);
  
  pinMode(ledPin1,OUTPUT);
  pinMode(ledPin2,OUTPUT);
  pinMode(ledPin3,OUTPUT);
  pinMode(ledPin4,OUTPUT);
  
  digitalWrite(INA1,LOW);
  digitalWrite(INA2,LOW);
  digitalWrite(INA3,LOW);
  digitalWrite(INA4,LOW);
  
  digitalWrite(INB1,LOW);
  digitalWrite(INB2,LOW);
  digitalWrite(INB3,LOW);
  digitalWrite(INB4,LOW);
  
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  analogWrite(PWM3,0);
  analogWrite(PWM4,0); 
  
  myservo1.attach(servo1);
  myservo2.attach(servo2);
  myservo3.attach(servo3);
  myservo4.attach(servo4);
  
  myservo1.writeMicroseconds(1500);  // set servo to mid-point
  myservo2.writeMicroseconds(1500);
  myservo3.writeMicroseconds(1500);
  myservo4.writeMicroseconds(1500);
     
  Serial.begin(115200);
  previousTime = micros();
}

void loop(){
  
  currentTime = micros()-previousTime;
  
  if(currentTime >= 10000) //if current time more than 10000 microsecond or 100Hz 
  {
    //.............input signal from receiver......................
    //pulseIn from Receiver vary between 1000 - 2000.
    //if signal from receiver is 1500(in the middle position), the moter should stop.
    //We can change the signal range from (1000,2000) to (-500,500) by -1500; 
    input1 = pulseIn(CH1,HIGH)-1500;                   //channel 1
    input2 = pulseIn(CH2,HIGH)-1500;                   //channel 2
    input3 = pulseIn(CH3,HIGH)-1500;                   //channel 3
    input4 = pulseIn(CH4,HIGH)-1500;                   //channel 4
    input5 = pulseIn(CH5,HIGH)-1500;                   //channel 5
    input6 = pulseIn(CH6,HIGH)-1500;                   //channel 6
  
    input1 = Deadband(input1,30);
    input2 = Deadband(input2,30);
    input3 = Deadband(input3,50);
    input4 = Deadband(input4,30);
    input5 = Deadband(input5,30);
    input6 = Deadband(input6,30);
    
    //Left wheel and Right wheel are controlled by input3 and input4 (ch3,ch4)
    left_w = input1; //original input3-input4
    right_w = input2; //original input3+input4
    
    //.................Read Motor's Current From Pololu................
    //The resolution of analogRead is 5/1024 volt/unit
    //The resolution of CS from POLOLU VNH5019 about 0.14 V/Amp
    //Then 5/1024/0.14 = 0.035
    currentValue1 = analogRead(CS1)*0.035;
    currentValue2 = analogRead(CS2)*0.035;
    currentValue3 = analogRead(CS3)*0.035;
    currentValue4 = analogRead(CS4)*0.035;
    
    //......................Drive Motor...........................................
    // The motor can operate if current value at each motor is not over than 5 amp.
    if(currentValue1 < 5) out1 = OutputToMotor1(right_w);//originally input1
    else out1 = OutputToMotor1(0);
    
    if(currentValue2 < 5) out2 = OutputToMotor2(right_w);//originally input2
    else out2 = OutputToMotor2(0);
    
    if(currentValue3 < 5) out3 = OutputToMotor3(left_w);
    else out3 = OutputToMotor3(0);
    
    if(currentValue4 < 5) out4 = OutputToMotor4(left_w);
    else out4 = OutputToMotor4(0);
    
    analogWrite(PWM1,out1);
    analogWrite(PWM2,out2);
    analogWrite(PWM3,out3);
    analogWrite(PWM4,out4);
    
    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    //if (buttonState == HIGH) { 
      
    if (input5 > 200){
      // turn LED on:
      if(buttonState == HIGH){    
        digitalWrite(ledPin1, HIGH); 
        digitalWrite(ledPin2, HIGH); 
        digitalWrite(ledPin3, HIGH); 
        digitalWrite(ledPin4, HIGH);  
      }else{
        digitalWrite(ledPin1, LOW); 
        digitalWrite(ledPin2, LOW);
        digitalWrite(ledPin3, LOW);
        digitalWrite(ledPin4, LOW);
      }
    } else {
      // turn LED off:
      if(buttonState == HIGH){
        digitalWrite(ledPin1, LOW); 
        digitalWrite(ledPin2, LOW);
        digitalWrite(ledPin3, LOW);
        digitalWrite(ledPin4, LOW);
      }else{
        digitalWrite(ledPin1, HIGH); 
        digitalWrite(ledPin2, HIGH); 
        digitalWrite(ledPin3, HIGH); 
        digitalWrite(ledPin4, HIGH);
      }
    }
    
    //Low pass filter
    S1_bar = (Tc*S1_bar + dt*input1)/Tcc;
    S2_bar = (Tc*S2_bar + dt*input2)/Tcc;
    S3_bar = (Tc*S3_bar + dt*input3)/Tcc;
    S4_bar = (Tc*S4_bar + dt*input4)/Tcc;
    
    //servo
    if (input6 > 200){
      myservo1.writeMicroseconds(1500+S1_bar);  // set servo to mid-point
      myservo2.writeMicroseconds(1200+S2_bar);
      myservo3.writeMicroseconds(900+S3_bar);
    }else{
      myservo1.writeMicroseconds(1500);
      myservo2.writeMicroseconds(1200);
      myservo3.writeMicroseconds(900);
    }
  
    previousTime = micros();
    
    Serial.print("ch1 = ");
    Serial.print(currentValue1);
    Serial.print("\t ch2 = ");
    Serial.print(currentValue2);
    Serial.print("\t ch3 = ");
    Serial.print(currentValue3);
    Serial.print("\t ch4 = ");
    Serial.print(currentValue4);
    Serial.print("\t dt = ");
    Serial.println(currentTime);
  }
}

int OutputToMotor1(int value)
{
  int temp = 0;
  if(value >= 0)
  {
    digitalWrite(INA1,LOW);
    digitalWrite(INB1,HIGH);
    temp = map(value,0,500,0,255);
  }else
  {
    digitalWrite(INA1,HIGH);
    digitalWrite(INB1,LOW);
    temp = map(-value,0,500,0,255);
  }
  return temp;
}

int OutputToMotor2(int value)
{
  int temp = 0;
  if(value >= 0)
  {
    digitalWrite(INA2,LOW);
    digitalWrite(INB2,HIGH);
    temp = map(value,0,500,0,255);
  }else
  {
    digitalWrite(INA2,HIGH);
    digitalWrite(INB2,LOW);
    temp = map(-value,0,500,0,255);
  }
  return temp;
}

int OutputToMotor3(int value)
{
  int temp = 0;
  if(value >= 0)
  {
    digitalWrite(INA3,LOW);
    digitalWrite(INB3,HIGH);
    temp = map(value,0,500,0,255);
  }else
  {
    digitalWrite(INA3,HIGH);
    digitalWrite(INB3,LOW);
    temp = map(-value,0,500,0,255);
  }
  return temp;
}

int OutputToMotor4(int value)
{
  int temp = 0;
  if(value >= 0)
  {
    digitalWrite(INA4,LOW);
    digitalWrite(INB4,HIGH);
    temp = map(value,0,500,0,255);
  }else
  {
    digitalWrite(INA4,HIGH);
    digitalWrite(INB4,LOW);
    temp = map(-value,0,500,0,255);
  }
  return temp;
}

double Deadband(double value,double limit)
{
  double temp = 0.0;
  
  if(value >= limit) temp = value-limit;
  else if(value <= -limit) temp = value+limit;
  else temp = 0.0;
  
  return temp;
}
