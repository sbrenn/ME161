//ON TEENSY
//currently using 3.3V to power encoder but in reality would do 5V and voltage divider 
//doesn't yet use gyroscope to control (and doesn't yet read gyroscope, but see the test_IMU_10... file) 
//and attempt to merge them

int encoder0PinA = 2;
int encoder0PinB = 3;
int motorPWM = 17;//changed from 5
int motorDirA = 7;//changed from 7
int motorDirB = 8;//changed from 8

double force =0;//CHANGED FROM ZERO
double duty = 0;
double duty_coeff = 7.94326331388610; //from Matlab regression of torque duty test
double torque = 0;
double pend_rad = 0.10529; //pendulum radius [m]
double output=0;
int clickres = 4; //quarter ticks

//Encoder Variables
unsigned long ticks_per_rev = 12;
volatile signed long num_revs = 0;
unsigned long lastRead = 0;
unsigned long interval = 1000;//seconds
unsigned int sec_count =0;
double curr_time = 0;
volatile signed long encoder0Pos = 0;

volatile signed long tick = 0; 
volatile signed long prev_tick = 0;
volatile signed long tick_thresh = 1000;
double angle_enc = 0; //encoder angle
double angle = 0;

//for feedback control:
//double x_des = double(pend_rad*double(M_PI/double(2))); //desired angle, changed from 0
double x_des=0;
double vel_des = 0;
int output_count = 0;
//double pend_rad = 0.10529; //pendulum radius [m]
double kp = 0.9;
double kd = 0.03;
double ki = 0.001;//changed from 0.0001
double pos_diff = 0;
double e_int = 0;
double vel_diff = 0;
//double force = 0;
//double torque = 0;


// Kinematics variables
double x = 0; // Position of the handle [m]
double x_prev; // Distance of the pendulum at previous time step
double x_prev2;
double dx; // Velocity of the handle
double dx_prev;
double dx_prev2;
double dx_filt; // Filtered velocity of the handle
double dx_filt_prev;
double dx_filt_prev2;

//GEARS
int num_teeth_small = 30;
int num_teeth_big = 50;
double gear_rat = double(double(num_teeth_small)/double(num_teeth_big));
double axle_ang = 0;
double car_dist = 0;
double wheel_rad = 0.01475; //meters
double torque_wheels = 0;


//---------End------------

double t_val = 0;


// Initial time
long int ti;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);//changed from 115200
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirA, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  analogWrite(motorPWM, 0); //make sure motor is off
  digitalWrite(motorDirA, LOW);
  digitalWrite(motorDirB, LOW);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);
  
}

void loop() {
  curr_time = millis()-ti;
  //Serial.println(curr_time);
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
  angle_enc = (double)(arc_fraction*360); //degrees
  angle = double(angle_enc * double(M_PI/180));
  //Serial.println(angle_enc);
  t_val = millis()-ti;
  axle_ang = -angle * gear_rat;
  x = wheel_rad*axle_ang;
  //Serial.println(car_dist,5);
  dx = (double)(x - x_prev) / 0.001;
  dx_filt = .9*dx + 0.1*dx_prev;
  pos_diff = x-x_des;
  e_int +=pos_diff;
  vel_diff = dx_filt-vel_des;
  force = kd*(vel_diff)+kp*(pos_diff)+ki*e_int;
  torque_wheels = force*wheel_rad;
  torque = torque_wheels*gear_rat;
  duty = duty_coeff* sqrt(abs(torque));

  
  /*torque = force*pend_rad;
  //Serial.println(torque,7);
  duty = duty_coeff* sqrt(abs(torque));
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);
  //Serial.println(output);
  if(force<0) {
    digitalWrite(motorDirA, HIGH);
    digitalWrite(motorDirB,LOW);
  } else {
    digitalWrite(motorDirA, LOW);
    digitalWrite(motorDirB,HIGH);
  }
  // Make sure the duty cycle is between 0 and 100%
  //Serial.println(output);
  analogWrite(motorPWM,output);  // output the signal
  */
   // Set previous to current 
   x_prev2 = x_prev;
   x_prev = x;
   dx_prev2 = dx_prev;
   dx_prev = dx;
   dx_filt_prev2 = dx_filt_prev;
   dx_filt_prev = dx_filt;
   
  if (duty > 1) {            
      duty = 1;
  } else if (duty < 0) { 
      duty = 0;
  }  
  output =(int)(duty* 255);
  /*Serial.print(force,5);
  Serial.print("\t");
  Serial.println(output);
  */
  //Serial.println(output);
   if(force>0) { //SWITCHED!
    digitalWrite(motorDirA, HIGH);
    digitalWrite(motorDirB,LOW);
  } else {
    digitalWrite(motorDirA, LOW);
    digitalWrite(motorDirB,HIGH);
  }
  // Make sure the duty cycle is between 0 and 100%
  //Serial.println(output);
  analogWrite(motorPWM,output);  // output the signal
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

  







