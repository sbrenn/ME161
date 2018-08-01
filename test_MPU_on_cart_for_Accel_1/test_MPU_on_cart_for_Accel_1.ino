//MPU9250 on cart and connected to Teensy

#include <TimerOne.h>

#include <Wire.h>
#include <WireKinetis.h>

//#include <Wire.h>

//#include <TimerOne.h>

//TRY voltage divider so powering it with 2.5V
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//My code:
double acc_sens =8192; //counts/g
double acc_range = 8; //+-4g
//double gyro_sens =32.8; //counts/deg/s
//double gyro_range = 2000; //+-1000 deg/s
//double mag_range = 4800; //microstesla
//double mag_sens = 0.6; //microtesla/count 
double ay_g = 0;
double ay_ms2 = 0;
long int t_curr=0;
//double theta = 0;

double acc_y_offset = 0.00850; //found manually
//MAYBE MAKE THIS MORE ACCUARTE BY TAKING MEAN RMS WHEN ON TABLE

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}



// Initial time
long int ti;
volatile bool intFlag=false;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  //+-1000 degrees per second
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  //+-4g
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  //+-4800 microTesla
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();
}





// Counter
long int cpt=0;

void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
void loop()
{
  while (!intFlag);
  intFlag=false;

  t_curr = millis()-ti;
  // Display time
  //Serial.print (millis()-ti,DEC);
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  //Just care about ay for now
  //int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  ay_g = (double)(double(ay)/acc_sens);
  //Serial.println(ay_g,5);
  ay_g-=acc_y_offset;
  //Serial.println(ay_g,5);
  //Serial.println(ax_g,5);
  ay_ms2 = double(ay_g*double(9.8));
  if (t_curr<5000) {
    Serial.print(t_curr);
    Serial.print("\t");
    Serial.println(ay_g,5);
  }
}









