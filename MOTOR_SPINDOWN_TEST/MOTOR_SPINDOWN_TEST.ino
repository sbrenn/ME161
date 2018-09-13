//BETTER THAN THE SECOND VERSION!!!
#include <TimerOne.h>
//using normal 3.3V output 
//it works but it probably isn't getting as much as possible out of the motor because a max duty cycle (255)
//corresponds to 3.5V/5V on the driver
int encoder0PinA = 2;
int encoder0PinB = 3;
int motorPWM = 17;//changed from 6
int motorDirA = 7;//changed from 7
int motorDirB = 8;//changed from 8
unsigned long ticks_per_rev = 12;
volatile signed long num_revs = 0;
unsigned long lastRead = 0;
unsigned long interval = 1000;//seconds
unsigned int sec_count =0;
double pwm_freq = 25000;
long int ti;
volatile bool intFlag=false;
int curr_time = 0;
int prev_time = 0;
double prev_ang = 0;
double ang_vel = 0;
double time_diff =0;
double ang_vel_filt = 0;
double ang_diff = 0;
double ang_vel_prev = 0;

volatile signed long encoder0Pos = 0;
int clickres = 4; //quarter ticks

volatile signed long tick = 0; 
volatile signed long prev_tick = 0;
volatile signed long tick_thresh = 1000;
double angle_enc = 0; //encoder angle
int count = 0;
int count_max = 2000;
//#define ARRAY_SIZE 200 //500 was too much for the matrix
//double vals[ARRAY_SIZE][2];
int not_printed = 1;
//int pin_17 = 17;
double rpm =0;

void setup() {
  // put your setup code here, to run once:
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirA, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  analogWriteFrequency(motorPWM, pwm_freq);
  analogWrite(motorPWM, 0); //make sure motor is off
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);
  // Store initial time

  //make sure array starts with all zeros
  /*for (int i=0; i<count_max-1; i++){
        vals[i][0]= 0;
        vals[i][1]=0;
  }*/
  digitalWrite(motorDirB, LOW);
  digitalWrite(motorDirA, LOW);
  //analogWrite(motorPWM, 250);
  delay(2000);
  digitalWrite(motorDirA, HIGH);
  //pinMode(pin_17,OUTPUT);
  //analogWrite(pin_17, 255);
  Serial.begin(9600);//changed from 115200
  for(int x = 0; x < 255; x++){
      analogWrite(motorPWM, x);
      delay(50);
    }
  digitalWrite(motorDirA, LOW);
  ti=micros();
}

void loop() {
  curr_time = micros()-ti;
  if (encoder0Pos%clickres==0 && encoder0Pos!=0) {
    int sign = encoder0Pos/clickres; 
    tick+=(signed long)(sign);
    encoder0Pos=0;
  }
  if (abs(tick-prev_tick)>=tick_thresh) {
    tick = prev_tick;
  } else {
    prev_tick = tick;
  }

  double tick_fraction = (double)encoder0Pos/clickres;
  double tick_decimal = (double)(tick+tick_fraction);
  double arc_fraction = (double)(tick_decimal/ticks_per_rev);
  //Serial.println(arc_fraction);
  angle_enc = -1*(double)(arc_fraction*360); //degrees
  angle_enc = double(angle_enc * double(M_PI/180));//radians
  time_diff = double(double(curr_time-prev_time)/1000000);
  ang_diff = double(angle_enc-prev_ang);
  ang_vel = double(ang_diff/time_diff);
  //ang_vel = 
  //ang_vel = double(double(angle_enc-prev_ang)/double(double(curr_time-prev_time)/1000));
  if (count<=count_max && ang_vel<0&&curr_time>=100&&count>=1&&count%50==0) {//if ang_vel is positive it's an outlier, and start after 100us
     time_diff = double(double(curr_time-prev_time)/1000000);
     ang_diff = double(angle_enc-prev_ang);
     ang_vel = double(ang_diff/time_diff);
     rpm = double(abs(ang_vel)*60*double(1/(2*M_PI)));
     Serial.print(double(double(curr_time)/1000000),6);
     Serial.print("\t");
     Serial.println(rpm,7);
     prev_time = curr_time;
     prev_ang = angle_enc;
  }
  /*if (count<=count_max &&curr_time>=1000) {
    Serial.print(double(double(curr_time)/1000000),6);
    Serial.print("\t");
    Serial.println(angle_enc,9);
  }*/
  count+=1;
  /*prev_time = curr_time;
  prev_ang = angle_enc;
  //ang_vel_prev = ang_vel;*/
}
void doEncoderA() {
  //Serial.println("I'm here");
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    //Serial.println("A went high");
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}


  //----------------End---------------------

  







