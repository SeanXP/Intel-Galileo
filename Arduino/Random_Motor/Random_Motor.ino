
#include "Wire.h"
//Motor pwm
// PWM pin : 3,5,6,9,10,11
int pwm1 = 3;
int pwm2 = 6;
int pwm3 = 9;
// motor dir
int dir1 = 2;
int dir2 = 5;
int dir3 = 8;

int count = 0;

#define M1_UP    (digitalWrite(dir1, LOW))
#define M1_DOWN  (digitalWrite(dir1, HIGH))
#define M2_UP    (digitalWrite(dir2, LOW))
#define M2_DOWN  (digitalWrite(dir2, HIGH))
#define M3_UP    (digitalWrite(dir3, LOW))
#define M3_DOWN  (digitalWrite(dir3, HIGH))

#define M1_SPEED(X)  (setPwmI2C(pwm1, (X)))
#define M2_SPEED(X)  (setPwmI2C(pwm2, (X)))
#define M3_SPEED(X)  (setPwmI2C(pwm3, (X)))

//finite-state machine
#define STATE_STOP  0  //dir_state = 0, stop;
#define STATE_UP    1  //dir_state = 1, up;
#define STATE_DOWN  2  //dir_state = 2, down;
int dir_state_x = STATE_STOP;
int dir_state_y = STATE_STOP;


//100ms change pwm data. (100)
float data[] = 
{
  -0.226,
  0.461,
  0.719,
  0.810,
  0.419,
  -0.033,
  -1.598,
  -2.590,
  -3.361,
  -3.087,
  -1.095,
  0.987,
  3.300,
  4.413,
  2.555,
  -1.226,
  -5.856,
  -9.303,
  -10.276,
  -7.494,
  -0.962,
  7.991,
  17.721,
  26.819,
  34.070,
  38.878,
  39.678,
  35.255,
  24.697,
  6.959,
  -16.231,
  -41.430,
  -61.296,
  -68.720,
  -58.202,
  -29.411,
  12.825,
  58.498,
  95.746,
  114.497,
  109.740,
  86.806,
  51.751,
  15.254,
  -14.218,
  -32.896,
  -41.137,
  -42.075,
  -38.102,
  -29.903,
  -17.677,
  -1.940,
  15.991,
  29.485,
  34.843,
  26.230,
  4.905,
  -23.591,
  -52.426,
  -72.393,
  -76.534,
  -65.990,
  -47.161,
  -28.474,
  -17.430,
  -17.296,
  -25.158,
  -35.616,
  -42.159,
  -41.698,
  -33.768,
  -23.466,
  -16.349,
  -16.458,
  -24.457,
  -36.222,
  -46.690,
  -48.938,
  -41.213,
  -24.360,
  -4.524,
  12.394,
  20.751,
  18.527,
  8.495,
  -2.912,
  -8.817,
  -6.920,
  2.029,
  13.263,
  20.190,
  19.219,
  7.653,
  -8.165,
  -22.482,
  -26.027,
  -17.924,
  1.653,
  23.957,
  41.551
};

void setup() {  
  //begin Wire for controlling PWM by I2C.
  Wire.begin();
  setPwmI2C (pwm1, 0);  //first need 50ms~ (because of enable pwm output), and then need ~11ms
  setPwmI2C (pwm2, 0);
  setPwmI2C (pwm3, 0);

  pinMode(dir1, OUTPUT); //cost 25ms
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
}

float now_data_y;
void loop() {
 
  now_data_y = data[++count];
  if(now_data_y > 0) //M1 need up
  {
     if(dir_state_y != STATE_UP)
     {
       dir_state_y = STATE_UP;
       M1_UP;
     }
     M1_SPEED(now_data_y);
  }
  else
  {
    
  }
    
}


//set PWM use I2C 
//set pin(_iPinNum) output PWM with duty cycle of (_iPwmVal / 255).
void setPwmI2C(int _iPinNum, int _iPwmVal)
{
  if(_iPwmVal > 250) _iPwmVal = 250;
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

  // Set period register (2Ah) - 0x01 / 0xff, need 10ms
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

//just set pwm period & pulse of the PWM output. //need 10ms
void setPwm(int _iPinNum, int _iPwmVal)
{
  if(_iPwmVal > 250) _iPwmVal = 250;
  // Select pin to write I2C commands to
  analogWrite(_iPinNum,1);            //analogWrite(pin, 1) will enable the PWM on the pin. 

  Wire.beginTransmission(0x20);
  Wire.write(0x2a);
  Wire.write(0xff);
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x2b);
  Wire.write(_iPwmVal);
  Wire.endTransmission();
}


