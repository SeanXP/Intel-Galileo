IO Speed in Galileo
====

###GPIO
* Default max throghput is 230 Hz
* IO2 and IO3 if configured as OUTPUT_FAST - will clock in at 477 kHz - to 2.93 MHz
	* digitalWrite() will achieve 477 kHz     
	* fastGpioDigitalWrite() - an Intel extension of the Arduino API will achieve 680 kHz
	* fastGpioDigitalWriteDestructive - can achieve GPIO speeds of about 2.93 MHz
* Example sketch usage below

###SPI
* The Arduino API does byte-by-byte SPI transactions - which are not especially fast
* Intel has provided an API extension to allow transfer of larger blocks of SPI data transferBuffer
* This extension should allow Arduino sketches to take better advantage of the SPI interface.
* I don't have the throughput number to hand - will search for it.


###ADC
* The throughput of the ADC is constrained by the SPI

###Interrupts
* In theory it would be possible to circumvent the GPIO input mechanism to kick a task in user-space
* Right now we allow
	* GPIO lib based callbacks    
	* HPET driven callbacks with granularity up to 1 kHz

* As with all systems based on a general purpose operating system and even some systems that claim determinism - your reaction time to any given event is a function of the workload the processors is engaged in. A quiescent system is likely to react very quickly to a GPIO even if its triggering a user-space application. A system under load could have significant jitter. This is down to the choices made by the application engineer re: how much concurrent processing is appropriate.





##three methods
There are three methods to communicate with these pins - which have increasing throughput
 
#####digitalWrite()
Using this method it is possible to toggle an individual pin in a tight loop @ about 477 kHz

	pinMode(2, OUTPUT_FAST);
	pinMode(3, OUTPUT_FAST);

This is a read-modify-write   
	* To toggle a bit - first we read    
	* Then we update     
	* Then we write back    
	
#####fastGpioDigitalWrite(register uint8_t gpio, register uint8_t val)

This function actually lets you write directly to the registers - without going through the code around digitalWrite() and consequently has better performance than a straight digitalWrite     

Using this method it is possible to toggle an individual pin (GPIO_FAST_IO2, GPIO_FAST_IO3) at about 680 kHz    

	pinMode(2, OUTPUT_FAST);
	pinMode(3, OUTPUT_FAST);
	
	fastGpioDigitalWrite(GPIO_FAST_IO2, 1);
	fastGpioDigitalWrite(GPIO_FAST_IO3, 0);
Again this uses read/modify/write - and can toggle one GPIO at a time
 
#####fastGpioDigitalWriteDestructive(register uint8_t gpio_mask); 
Using this method it is possible to achieve 2.93 Mhz data toggle rate on IO2/IO3     individually or simultaneously    

	pinMode(2, OUTPUT_FAST);
	pinMode(3, OUTPUT_FAST);
	
It is the responsibility of the application to maintain the state of the GPIO registers directly     
To enable this a function called fastGpioDigitalLatch() is provided - which allows the calling logic to latch the initial state of the GPIOs - before updating later    
This method just writes GPIO bits straight to the GPIO register - i.e. a destructive write - for this reason it is approximately 2 x faster then read/modify/write    
Twice as fast for a given write - means four times faster for a given wave form - hence ~700kHz (680kHz) becomes ~2.8Mhz (2.93 MHz)    
 

##Example
###1 - outputs 477kHz waveform on IO2

	void setup(){
    		pinMode(2, OUTPUT_FAST);
	}

	void loop()
	{
		register int x = 0;
		while(1){
			digitalWrite(2, x);
			x =!x;
		}
	}
###2 - outputs 683kHz waveform on IO3

	void setup(){
		pinMode(3, OUTPUT_FAST); //IO3 fast output mode.
	}

	void loop()
	{
		register int x = 0;
		while(1){
			//digitalWrite(3, x); // 477kHz , the same with IO2 fast output mode.
 
			 fastGpioDigitalWrite(GPIO_FAST_IO3, x); 
			 //an Intel extension of the Arduino API will achieve 680 kHz
			 
			x =!x;
		}
	}
	
###3 - outputs 2.93MHz waveform on IO3

	uint32_t latchValue;

	void setup(){
		pinMode(3, OUTPUT_FAST);
		latchValue = fastGpioDigitalLatch();
	}
	void loop()
	{
		while(1){
			//fastGpioDigitalWriteDestructive - can achieve GPIO speeds of about 2.93 MHz
			fastGpioDigitalWriteDestructive(latchValue);
			latchValue ^= GPIO_FAST_IO3;
		}
	}
	
	

