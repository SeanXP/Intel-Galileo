#include "Wire.h"


unsigned long time_start = 0, time_end = 0;
#define TIME_S     (time_start = millis())
#define TIME_E     (time_end = millis(), Serial.print("Time:"), Serial.println(time_end - time_start))


int pwm_pin = 11;

void setup() {
  Wire.begin();
  setPwmI2C (pwm_pin, 127);
}

void loop() {
  
}

//set PWM use I2C 
//set pin(_iPinNum) output PWM with duty cycle of (_iPwmVal / 255).
void setPwmI2C(int _iPinNum, int _iPwmVal)
{
  // Select pin to write I2C commands to
  analogWrite(_iPinNum,1);            //analogWrite(pin, 1) will enable the PWM on the pin. 
                                      //but AnalogWrite(pin,0) does not disable the pwm.

  //Then there’s the I2C commands...


  // Select programmable PWM CLK source
  Wire.beginTransmission(0x20);
  Wire.write(0x29);  // CY8C9540A -  Config (29h)
  //xxxxx000b - 32kHz (default)        -> output 100+ HZ
  //xxxxx001b - 24Mhz                  -> output 96.36KHz    
  //xxxxx010b - 1.5Mhz                 -> output 6 kHz
  //xxxxx011b - 93.75 kHz              -> output 378 Hz
  //xxxxx100b - 367.6 Hz (programmable)-> output 367 Hz , can divider CLK
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
  Wire.write(0xff); // Selete Rising pulse edge.
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
