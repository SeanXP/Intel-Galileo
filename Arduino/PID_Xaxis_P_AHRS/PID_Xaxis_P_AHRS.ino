/************************************************************************

  read data from AHRS module to PID controller for control three motor.
  
          ^ Y     up X with Y, AHRS.roll from 180 -> 90 -> 0.
          |      
          |        
  X <-----|        
  
Connect:

  Serial1 - /dev/ttyS0   in Galileo - AHRS CJMCU
   XXX: AHRS must connect the VCC-5V with Galileo!
  
  Motor: (pwm 0~255) (dir: 0 up, 1 down)
  pwm1:  IO 3
  pwm2:  IO 6
  pwm3:  IO 9
  
  dir1:  IO 2
  dir2:  IO 5
  dir3:  IO 8

  read three double data from AHRS(always keep output data ).
  
 
***********************************************************************/

#include "Wire.h"
#include <PID_v1.h>

// AHRS output : #YPR: xxx, xxx, xxx
double yaw = 0;
double pitch = 0;
double roll = 0;
// AHRS read_data() 
char ch = 'c';
int len = 100;
char buf[100];

//Motor pwm
// PWM pin : 3,5,6,9,10,11
int pwm1 = 3;
int pwm2 = 6;
int pwm3 = 9;
// motor dir
int dir1 = 2;
int dir2 = 5;
int dir3 = 8;

int pwm_tmp;
int pwm_time = 10;

int counter = 0;


//PID controller
double KP = 1, KI = 0, KD = 0;
//Define Variables we'll be connecting to
double Setpoint = 0, Input = 0, Output = 0;
//PID output Limit.
double OutMinLimit = -100 * KP, OutMaxLimit = 100 * KP; 
double OutputRange = 3 * KP;
//Specify the links and initial tuning parameters
PID myPID1(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);   // just a P PID controller.

//dir_state = 0, stop;
//dir_state = 1, up;
//dir_state = 2, down;

int dir_state = 0;
//get X axis angle.
//return -90 ~ 90.
double getX();

void setup() {
  //AHRS Baud rate = 57600
  Serial1.begin(57600);
  
  //begin Wire for controlling PWM by I2C.
  Wire.begin();
  setPwmI2C (pwm1, 0); 
  setPwmI2C (pwm2, 0);
  setPwmI2C (pwm3, 0);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  
  //PID set
  Setpoint = 0; // set aim of X axis angle.
  dir_state = 0;
  
  //turn the PID on
  myPID1.SetOutputLimits(OutMinLimit, OutMaxLimit); //set the PID output Limits; 
  myPID1.SetMode(AUTOMATIC);
  Serial.begin(9600);
}

void loop() {

  //1. read data
  //read three double data from AHRS in 100ms interval.
  //100ms will make Arduino Serial buffer overflow, so need read the buffer and discard them.
  Input = getX();
  
  counter++;
  Serial.print(counter);
  //2. PID
  Serial.print(", Input:");
  Serial.print(Input);
  myPID1.Compute();              //return a error * KP.
  Serial.print(", Output:");
  Serial.print(Output); //should make roll to output.
  
  if(Output > OutputRange) // need to up!
  {
    if(dir_state != 1)
    {
      dir_state = 1;
      digitalWrite(dir1, LOW);
      setPwmI2C (pwm1, 250);
      Serial.print(",^");
    }
  }
  else if(Output < -OutputRange) // need to down
  { 
    if(dir_state != 2)
    {
      dir_state = 2;
      digitalWrite(dir1, HIGH);
      setPwmI2C (pwm1, 250);
      Serial.print(",V");
    }
  }
  else //else , Input - range < Output < Input + range. nice!
  {
    dir_state = 0;
    setPwmI2C (pwm1, 0);
    Serial.print(",-");
  }
  
  Serial.println(" ");
}

double getX()
{
  //get roll data from AHRS
  read_data();
  //roll = 180~90 or -180 ~ -90
  //transform roll to the X axis angle.
  
  return roll > 0? (180 - roll) : -(180 + roll);
}

// read three data from AHRS.
void read_data()
{ 
  if(Serial1.available())
  {
    //read buffer of Arduino and discard them for get fresh data soon.
    Serial1.readBytes(buf, len);

    //stop while until find a '#', for sync data.
    while(ch != '=')
    {
      ch = Serial1.read();
    }
    while(ch != '#')
    {
      ch = Serial1.read();
    }
  
    //get three double type of data.
    yaw = Serial1.parseFloat();
    pitch = Serial1.parseFloat();
    roll = Serial1.parseFloat(); 
  }
}


//set PWM use I2C 
//set pin(_iPinNum) output PWM with duty cycle of (_iPwmVal / 255).
void setPwmI2C(int _iPinNum, int _iPwmVal)
{
  //Set a PWM limit 
  if(_iPwmVal > 255) 
    _iPwmVal = 255;
  else if(_iPwmVal < 0)
    _iPwmVal = 0;
    
  // Select pin to write I2C commands to
  analogWrite(_iPinNum,1);            //analogWrite(pin, 1) will enable the PWM on the pin. 
                                      //but AnalogWrite(pin,0) does not disable the pwm.

  //Then there’s the I2C commands...


  // Select programmable PWM CLK source
  Wire.beginTransmission(0x20);
  Wire.write(0x29);  // CY8C9540A -  Config (29h)
  //xxxxx000b - 32kHz (default)
  //xxxxx001b - 24Mhz
  //xxxxx010b - 1.5Mhz
  //xxxxx011b - 93.75 kHz
  //xxxxx100b - 367.6 Hz (programmable)
  //xxxxx101b - Previous PWM
  //Wire.write(0x04);  // xxxxx100b - 367.6 Hz (programmable)
  Wire.write(0x02);    // 1.5MHZ , but output just 6khz. just use it~
  Wire.endTransmission();
  
  // Set divider the PWM CLK .
  Wire.beginTransmission(0x20);
  Wire.write(0x2C);
  Wire.write(0x01);
  Wire.endTransmission();

  // Set period register (2Ah) - 0x01 / 0xff
  Wire.beginTransmission(0x20);
  Wire.write(0x2a);
  //xxxx0xxxb - Falling pulse edge (default)      
  //xxxx1xxxb - Rising pulse edge
  Wire.write(0xff);
  Wire.endTransmission();

  // Set minimum duty cycle - Pulse Width Register (2Bh)
  Wire.beginTransmission(0x20);
  Wire.write(0x2b);
  //This register sets the pulse width of the PWM output. 
  //Allowed values are between zero and the (Period - 1) value.
  ////// this is the pulse width...0-255 where 255 is all "on" for one cycle (1/clock frequency)
  Wire.write(_iPwmVal);
  Wire.endTransmission();
  
}
