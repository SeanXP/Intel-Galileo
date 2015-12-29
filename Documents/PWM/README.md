Galileo PWM
====

[PWM on Galileo board in an arduino sketch](https://communities.intel.com/thread/45367)

	#include "Wire.h"

	int pwm_pin = 3;

	void setup() {
		Wire.begin();
		setPwmI2C (pwm_pin, 100);
	}

	void loop() { }

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


Then in your main loop() you just use:    
setPwmI2C ( the pin number that you want to control, the value from 0-255);
 
One thing to note about this method- the servo is expecting pulses from 1-2ms every 20ms.      
If we slow down the clock to 53 hz- that gives us the ~20ms cycle, but 0-255 gives pusle widths from zero all the way up to 20ms.     
 
So in order to send 1-2ms, your functional PWM range is only about 13 - 26!  

Depending on what you need, this range may be fine, but for more finite control, you can speed up the signal and increase the steps (125hz = range from 34-66.) As you increase the frequency, your servo will probably become less and less happy with you, so you can experiment to find a nice balance of happy servos and finite control.

###Brief

If you just use AnalogWrite(pinNumber, duty-cycle->0-255);

 it sends a PWM signal at the default frequency (~600Hz.)  But if you want to change the frequency (at least for now,) you need to manually change the clock using Wire and I2C commands every time you use AnalogWrite.
 
 To avoid repeating code, we wrote a function called **setPwmI2C** that we use in place of AnalogWrite that does pretty much the same thing, but at a lower frequency for servos.   If we wanted to do it on multiple pins we’d write (for example)
 
	loop(){
		setPwmI2C( 5, 255);  // sets pin 5 to 255 (100% duty cycle)
		setPwmI2C( 9, 127);  // sets pin 9 to 128 (~50% duty cycle)
		// only pins 3,5,6,9,10,11
	}


###I2C設備
Galileo使用CY8C9540A實現Arduino I/O & PWM的拓展。
Galileo的以太網接口旁邊，是I2C* Address Jumper，可以通過Jumper來實現I2C地址的設定。

To prevent a clash between the I2C* Slave address of the on board I/O expander and EEPROM with any external I2C* Slave devices, jumper J2 can be used to vary the I2C* address of the on-board devices.     With J2 connected to pin 1 (marked with white triangle), the 7-bit I/O Expander address is 0100001 and the 7-bit EEPROM address is 1010001.    Changing the jumper position changes the I/O Expander address to 0100000 and the EEPROM address to 1010000.    
即，默認情況下，I2C的地址爲0x20與0x50；    如果將Jumper將在白色三角的位置，I2C地址則爲0x21 & 0x51.
