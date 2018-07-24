//NOTE: borrowed some code from https://github.com/sparkfun/ADXL337_Breakout/blob/master/firmware/ADXL337_example/ADXL337_example.ino
//This comment is a test

// Includes
#include <math.h>

//Pin declares
int yAccPin = A5;
int xAccPin = A2;


//initialize variables
int yAcc_raw = 0;
int xAcc_raw = 0;
int half_range = 3; //+-3g range
double scaledY = 0;
double scaledX = 0;
double zero_offset = 1.69;//manually check this
double g = 1; //m/s^2
double y_acc_ratio = 0; 
double x_acc_ratio = 0; 
double theta = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(yAccPin, INPUT); //set acceleration pin to input
  pinMode(xAccPin, INPUT); //set acceleration pin to input
  yAcc_raw = analogRead(yAccPin);
  xAcc_raw = analogRead(xAccPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  yAcc_raw = analogRead(yAccPin);
  scaledY = mapf(yAcc_raw,0,675,-half_range,half_range); //3.3/5*1023=~675
  scaledY = scaledY - zero_offset;
  xAcc_raw = analogRead(xAccPin);
  scaledX = mapf(xAcc_raw,0,675,-half_range,half_range); //3.3/5*1023=~675
  scaledX = scaledX - zero_offset;
  //scaledY = scaledY * -1*g;
  y_acc_ratio = scaledY/g;
  x_acc_ratio = scaledX/g;
  //saturate between -1 and 1
  if (x_acc_ratio<-1){
    x_acc_ratio=-1;
  } else if (x_acc_ratio>1) {
    x_acc_ratio=1;
  }
  //WON'T THE MEASUREMENTS BE CORRUPTED BY ACTUAL ACCELERATION IN X??
  //HOW TO ACCOUNT FOR THIS??
  //answer: not sure, but it seems like it works pretty well without
    //compensating for it
  
  theta = asin(x_acc_ratio);
  //convert to degrees
  theta = theta*180/M_PI;
  Serial.println(theta);
  delay(100);
  //note: currently outputs reasonable angles, haven't checked absolute
   //accuracy yet though. maybe check with pocketlab or encoder

   //BECAUSE WE WANT THEM TO HAVE AN ODE, SEE IF I CAN GET A FORMULA FOR
   //ACCELERATION FROM GEOMETRY OF HAPKIT FOR EXAMPLE
}
// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
