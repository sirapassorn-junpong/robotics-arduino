/*
   Set Board: Tools --> Board --> Arduino Mega2560
   Set Processor: Tools --> Processor --> ATmega2560
*/


#include <Servo.h>
#include "configuration.h"


//=============== Function Name Declaration ===============
double Deadband(double value, double limit);
int OutputToMotor1(int value);
int OutputToMotor2(int value);
int OutputToMotor3(int value);
int OutputToMotor4(int value);


//=============== Parameters Declaration ==================

// Type Servo
Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;
// Time
unsigned long previousLoopTime = 0;
unsigned long loopTime = 0;
// Input Signal from Receiver
int input1 = 0;
int input2 = 0;
int input3 = 0;
int input4 = 0;
int input5 = 0;
int input6 = 0;
// Output Signal to Motor Driver
int out1 = 0;
int out2 = 0;
int out3 = 0;
int out4 = 0;
// Processed Input Signal
int left_w = 0;
int right_w = 0;
// Motor's Current
float currentValue1 = 0.0;
float currentValue2 = 0.0;
float currentValue3 = 0.0;
float currentValue4 = 0.0;
int currentLimit = 5;
int servovalue = 90;

int count = 0;
int reverse = 0;
//A front B left C right D back
/*const int pingPinA = ; // Trigger Pin of Ultrasonic Sensor
const int echoPinA = ; // Echo Pin of Ultrasonic Sensor
const int pingPinB = ; // Trigger Pin of Ultrasonic Sensor
const int echoPinB = ; // Echo Pin of Ultrasonic Sensor
const int pingPinC = ; // Trigger Pin of Ultrasonic Sensor
const int echoPinC = ; // Echo Pin of Ultrasonic Sensor
const int pingPinD = ; // Trigger Pin of Ultrasonic Sensor
const int echoPinD = ; // Echo Pin of Ultrasonic Sensor
*/
long dA,dB,dC,dD,t,s;


//===================== setup() ========================


void setup() {
  //===== Set Digital Pin Mode =====
  // Set pinmode to read command signal from Receiver.
  pinMode(CH1, INPUT);  //channel 1
  pinMode(CH2, INPUT);  //channel 2
  pinMode(CH3, INPUT);  //channel 3
  pinMode(CH4, INPUT);  //channel 4
  pinMode(CH5, INPUT);  //channel 5
  pinMode(CH6, INPUT);  //channel 6
  // Set pinmode to read command signal from Test Switch.
  pinMode(buttonPin, INPUT);
  // Set pinmode to write command signal to Motor Driver.
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(INA3, OUTPUT);  
  pinMode(INB3, OUTPUT);
  pinMode(PWM3, OUTPUT);

  pinMode(INA4, OUTPUT);
  pinMode(INB4, OUTPUT);
  pinMode(PWM4, OUTPUT);
  
/*  pinMode(pingPinA, OUTPUT);
  pinMode(echoPinA, INPUT);
  pinMode(pingPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  pinMode(pingPinC, OUTPUT);
  pinMode(echoPinC, INPUT);
  pinMode(pingPinD, OUTPUT);
  pinMode(echoPinD, INPUT);
*/
  // Set pinmode to write command signal to LED.
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  // Assign Servo variable to a servo pin
  myServo1.attach(servo1);
  myServo2.attach(servo2);
  myServo3.attach(servo3);
  myServo4.attach(servo4);


  //===== Initialize Command =====
  // Initialize Motor Driver.
  digitalWrite(INA1, LOW);
  digitalWrite(INB1, LOW);
  analogWrite(PWM1, 0);

  digitalWrite(INA2, LOW);
  digitalWrite(INB2, LOW);
  analogWrite(PWM2, 0);

  digitalWrite(INA3, LOW);
  digitalWrite(INB3, LOW);
  analogWrite(PWM3, 0);

  digitalWrite(INA4, LOW);
  digitalWrite(INB4, LOW);
  analogWrite(PWM4, 0);

  // Initialize Servo Motor, Set servo to Mid-point.
  myServo1.write(90);
  myServo2.write(servovalue);
  myServo3.write(90);
  myServo4.write(90);

  // Open Serial port, Set baud rate for serial data transmission.
  Serial.begin(115200); // USB:Rx0,Tx0

  // Returns time(us)
  previousLoopTime = micros();

} // End SetUp


//======================= loop() ==========================

long d(int pingP,int echoP){
  digitalWrite(pingP, LOW);
  delayMicroseconds(2);
  digitalWrite(pingP, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingP, LOW);
  t = pulseIn(echoP, HIGH);
  return t*0.0001*346;
}

int isNear(long s){if (s<100){return 0;} else return 1;}

void loop() {
  while(input5 > 0){
  //code for servo 1 control by input5(switch)
  if ( input5 >= 0) {
    myServo1.write(90);
  } else {
    myServo1.write(180);
  }

  //code for servo 2 control by input6(Analog)
  int servovalue = input6 ;
  servovalue = map(servovalue, -500, 500, 0, 180);

  const byte limit_range = 10;

  for (int run_value = 0; run_value <= 180; run_value += limit_range) {
    if (servovalue  >= run_value && servovalue < run_value + limit_range) {
      myServo2.write(run_value);
      break;
    }
  }


  loopTime = micros() - previousLoopTime;

  // if loop time is more than 10000 microseconds, do the next loop.
  // Limit Maximum feedback loop at 100Hz.
  if (loopTime >= 10000)
  {
    // Set new loop time
    previousLoopTime = micros();


    // Read input signal from receiver. PulseIn signal from Receiver vary between 1000 - 2000.
    // Substract 1500 as the offset to set new signal range from (1000, 2000) to (-500, 500)
    // Also set Deadband limit to the input Signal

    input1 = pulseIn(CH1, HIGH) - 1500; //Channel 1
    input2 = pulseIn(CH2, HIGH) - 1500; //Channel 2
    input3 = pulseIn(CH3, HIGH) - 1500; //Channel 3
    input4 = pulseIn(CH4, HIGH) - 1500; //Channel 4
    input5 = pulseIn(CH5, HIGH) - 1500; //Channel 5
    input6 = pulseIn(CH6, HIGH) - 1500; //Channel 6

    input1 = Deadband(input1, 30); //Channel 1
    input2 = Deadband(input2, 30); //Channel 2
    input3 = Deadband(input3, 50); //Channel 3
    input4 = Deadband(input4, 30); //Channel 4
    input5 = Deadband(input5, 30); //Channel 5
    input6 = Deadband(input6, 30); //Channel 6

    // Read Motor's Current From Motor Driver
    // The resolution of Arduino analogRead is 5/1024 Volts/Unit. (10-Bit, Signal vary from 0 to 1023 units)
    // The resolution of Current Sensor from POLOLU VNH5019 is 0.14 Volt/Amp.
    // Convert analogRead signal(Volt) to Current(Amp) by multiply (5/1024)/0.14 = 0.035 Amp/Unit.
    currentValue1 = analogRead(CS1) * 0.035; // Motor Driver 1
    currentValue2 = analogRead(CS2) * 0.035; // Motor Driver 2
    currentValue3 = analogRead(CS3) * 0.035; // Motor Driver 3
    currentValue4 = analogRead(CS4) * 0.035; // Motor Driver 4


    // Robot Direction is controlled by input3 and input4 (CH3, CH4)
    left_w = -(input3 - input4);
    right_w = -(input3 + input4);

    // Check Motor Current and Assign Motor Direction
    // Motor can be operated, if motor's current is lower than 5 amp.
    if (currentValue1 < currentLimit) out1 = OutputToMotor1(input1);
    else out1 = OutputToMotor1(0);
    if (currentValue2 < currentLimit) out2 = OutputToMotor2(input2);
    else out2 = OutputToMotor2(0);
    if (currentValue3 < currentLimit) out3 = OutputToMotor3(left_w);
    else out3 = OutputToMotor3(0);
    if (currentValue4 < currentLimit) out4 = OutputToMotor4(right_w);
    else out4 = OutputToMotor4(0);


    // Drive Motor
    // Assign motor's direction by function OutputToMotor.
    // Command PWM Duty Cycle to drive motor.
    analogWrite(PWM1, out1);
    analogWrite(PWM2, out2);
    analogWrite(PWM3, out3);
    analogWrite(PWM4, out4);


    // Print
    Serial.print("M 1 = ");
    Serial.print(input1);
    Serial.print("\t M 2 = ");
    Serial.print(input2);
    Serial.print("\t M 3 = ");
    Serial.print(input3);
    Serial.print("\t M 4 = ");
    Serial.print(input4);
    Serial.print("\t M 5 = ");
    Serial.print(input5);
    Serial.print("\t M 6= ");
    Serial.print(input6);
    Serial.print("\t LoopTime = ");
    Serial.println(loopTime);

    } // End if
  }// End while
  while(input5 < -200){ //start auto
/*    dA=d(pingPinA,EchoPinA);
    dB=d(pingPinB,EchoPinB);
    dC=d(pingPinC,EchoPinC);
    dD=d(pingPinC,EchoPinC);*/
    while(isNear(dA)==1){
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    analogWrite(PWM2, 50);  
    digitalWrite(INA3, LOW);
    digitalWrite(INB3, HIGH);
    analogWrite(PWM3, 50);
    }  
    if(isNear(dA)==0 && isNear(dB)==0 && isNear(dC)==1 && isNear(dB)==1){  //turn right
      digitalWrite(INA2, HIGH);
      digitalWrite(INB2, HIGH);
      analogWrite(PWM2, 0);  
      digitalWrite(INA3, HIGH);
      digitalWrite(INB3, HIGH);
      analogWrite(PWM3, 0);  
      delay(500);
      digitalWrite(INA2, LOW);
      digitalWrite(INB2, HIGH);      
      analogWrite(PWM2, 50);
      }
    if(isNear(dA)==0 && isNear(dC)==0 && isNear(dB)==1){  //turn left
      digitalWrite(INA2, HIGH);
      digitalWrite(INB2, HIGH);
      analogWrite(PWM2, 0);
      digitalWrite(INA3, HIGH);
      digitalWrite(INB3, HIGH);
      analogWrite(PWM3, 0);
      delay(500);      
      digitalWrite(INA3, LOW);
      digitalWrite(INB3, HIGH);
      analogWrite(PWM3, 50);
      }
    if(isNear(dA)==1 && isNear(dB)==1 && isNear(dC)==1 && isNear(dB)==1 && count ==0){  //stoppu
      digitalWrite(INA2, HIGH);
      digitalWrite(INB2, HIGH);
      analogWrite(PWM2, 0);
      digitalWrite(INA3, HIGH);
      digitalWrite(INB3, HIGH);
      analogWrite(PWM3, 0);
      count=1;}
    if(isNear(dA)==1 && isNear(dB)==1 && isNear(dC)==1 && isNear(dB)==1 && count ==1 && input6<-200){  //reverse
      digitalWrite(INA2, HIGH);
      digitalWrite(INB2, LOW);
      analogWrite(PWM2, 50);
      digitalWrite(INA3, LOW);
      digitalWrite(INB3, HIGH);
      analogWrite(PWM3, 50);
      delay(500);
      count=0;}      
      
    
  }//end while 2
} // End loop


//=============== Function Declaration ===============


//===== double Deadband(double value,double limit) =====
//===== Set Dead Band =====
// If the input signal from receiver is in the band limit, set input signal to 0.0.
double Deadband(double value, double limit)
{
  double temp = 0.0;
  if (value >= limit) temp = value - limit;
  else if (value <= -limit) temp = value + limit;
  else temp = 0.0;
  return temp;
}


//===== int OutputToMotor(int value) ======
//===== Assign Motor's Direction and Scale Down Input Signal =====
// value must be positive and scaled down to fit 8-Bit PWM Range.

// Motor 1
int OutputToMotor1(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    // CW
    digitalWrite(INA1, LOW);
    digitalWrite(INB1, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    // CCW
    digitalWrite(INA1, HIGH);
    digitalWrite(INB1, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// Motor 2
int OutputToMotor2(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA2, LOW);
    digitalWrite(INB2, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    digitalWrite(INA2, HIGH);
    digitalWrite(INB2, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// Motor 3
int OutputToMotor3(int value)
{
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA3, LOW);
    digitalWrite(INB3, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    digitalWrite(INA3, HIGH);
    digitalWrite(INB3, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}

// Motor 4
  int OutputToMotor4(int value){
  int temp = 0;
  if (value >= 0)
  {
    digitalWrite(INA4, LOW);
    digitalWrite(INB4, HIGH);
    temp = map(value, 0, 500, 0, 255);
  } else {
    digitalWrite(INA4, HIGH);
    digitalWrite(INB4, LOW);
    temp = map(-value, 0, 500, 0, 255);
  }
  return temp;
}
