/************************************************************************

  read data from SCA60C module to PID controller for control three motor.
  
 ^ Y        
 |      
 |        
 |-----> X  
  
Connect:
  
  SCA60C:
  Vo1 - Galileo Analog Input A0
  Vo2 - Galileo Analog Input A1
  
   (0.5~4.5V) (-90~90 degrees)
   Vo1: left-right   (x axis)
   Vo2: up-down      (y axis)

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

//SCA60C module
//Vo1 -> A0
//Vo2 -> A1
int SCA60C_Pin_x = A0;
int SCA60C_Pin_y = A1;
int sensorValue_x = 0;
int sensorValue_y = 0;
//output voltage when horizontal.
double voltage_offset_x = 2.29;
double voltage_offset_y = 2.37;
double voltage_x = 0;
double voltage_y = 0;
double angle_x = 0;
double angle_y = 0;

//Motor pwm
// PWM pin : 3,5,6,9,10,11
int pwm1 = 3;
int pwm2 = 6;
int pwm3 = 9;
// motor dir
int dir1 = 2;
int dir2 = 5;
int dir3 = 8;

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
//get Y axis angle.
//return -90 ~ 90.




void setup() {
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
  
  //turn on Serial for debugging...
  Serial.begin(9600);   
}

void loop() {

  //1. read data of Y axis angle.
  Input = SCA60C_getY();
  //2. PID
  myPID1.Compute();              //return a error * KP.
  
  //debug...
  counter++;
  Serial.print(counter);
  Serial.print(", In:");
  Serial.print(Input);
  Serial.print(", Out:");
  Serial.print(Output);
  
  if(Output > OutputRange) // need to up!
  {
    if(dir_state != 1) // if not up state, change state to up now.
    {
      dir_state = 1;
      digitalWrite(dir1, LOW);
      setPwmI2C (pwm1, 250);
      Serial.print(",^");
    }
  }
  else if(Output < -OutputRange) // need to down!
  { 
    if(dir_state != 2) // if not down state, change state to up now.
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

double SCA60C_getX()
{
  sensorValue_x = analogRead(SCA60C_Pin_x);
  voltage_x = (sensorValue_x / 204.8);
  angle_x = -asin(0.5 * (voltage_x - voltage_offset_x)) * 57.292779524;
  return angle_x;
}

double SCA60C_getY()
{
  sensorValue_y = analogRead(SCA60C_Pin_y);
  voltage_y = (sensorValue_y / 204.8);
  angle_y = -asin(0.5 * (voltage_y - voltage_offset_y)) * 57.292779524;
  return angle_y;
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
