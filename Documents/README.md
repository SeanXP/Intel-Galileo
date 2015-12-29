Galileo is a microcontroller board based on the Intel Quark SoC X1000 Application Processor, 
a 32-bit Intel Pentium-class system on a chip.

Digital pins 0 to 13 (and the adjacent AREF and GND pins),   
Analog inputs 0 to 5, the power header,   
ICSP header, and the UART port pins (0 and 1),   
are all in the same locations as on the Arduino Uno R3.    

工作所需电压(3.3V / 5.0V);   
The core operating voltage of Galileo is 3.3V.    
板上有一jumper(IOREF), 可通过跳线帽切换I/O的参考电压为3.3V / 5.0V;  

##Arduino Shield Supported Features
#####14 digital input/output pins.  
#####A0~A5 - 6 analog inputs,    
via an AD7298 analog-to-digital (A/D) converter, provides 12 bits of resolution 
(i.e., 4096 different values). By default they measure from ground to 5 volts.
#####Analogue out pins 3,5,6,9,10,11
这六个默认为数字引脚, 但可以通过配置, 改为模拟输出引脚.
#####I2C bus, TWI
#####SPI   
Defaults to 4MHz to support Arduino Uno shields. Programmable up to 25MHz.
#####UART
UART0 (IO0 -> RX0, IO1 -> TX0) [Galileo: /dev/ttyS0]   
UART1 (headphone jack)				[Galileo: /dev/ttyS1]

Intel Galileo board features three serial ports:

	/dev/ttyGS0:    
	This is the port provided by client USB connector. It is normally used by Arduino IDE to upload sketches to the Galileo.

	/dev/ttyS1: 
	This is the serial port that is connected to the 3.5 mm connector, and it is normally used for the Linux console.


	/dev/ttyS0: 
	Galileo can be configured to connect this serial port to Arduino pins 0 and 1. 
	
	
/dev/ttyS0 is the first Quark UART (UART0) and it's the Digital 0 and 1 pins    
/dev/ttyS1 is the second Quark UART (UART1) and it's the audio jack one  
/dev/ttyGS0 is the virtual serial device created by the Linux cdc-acm driver via the Galileo USB port marked as "USB client".

the Arduino mappings		
	
	Serial is routed to the /dev/ttyGS0   
	Serial1 is routed to /dev/ttyS0     
	Serial2 is not used

#####VIN
#####5V/3.3V OUTPUT PIN
Maximum current draw to the shield is 800 mA.
#####IOREF   
a selection jumper on the board is used to select between 3.3V and 5V shield operation. 


###Arduino集成开发环境(IDE)
只需要写出简单的Arduino程序, IDE会自动将其转换为C语言,再创给avr-gcc编译器,最后编译成微处理器能理解的语言, 这这一切都是隐藏给用户的。
####隐藏复杂的编译过程,让用户尽可能以简单的方式去控制微处理器

##[Intel Quark BSP Available Downloads](https://downloadcenter.intel.com/Detail_Desc.aspx?DwnldID=23197)



##[Sergey's blog](http://www.malinov.com/Home/sergey-s-blog)


##[AlexT's Galileo pages](http://alextgalileo.altervista.org/package-repo-configuration-instructions.html)


