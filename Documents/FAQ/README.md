BUG of Galileo
====

##Reboot-BUG

拿到手的Intel Galileo有个BUG:    
不能正常reboot, 无论是按复位键,还是通过命令行, Galileo都是正常关闭, 但是在reboot时会卡住, 卡在下面这条信息:
	
	[ 8779.024881] reboot: machine restart
	
只有通过断电/上电才能正常reboot.

####解决办法
this is a known issue in the official 0.7.5 builds.   
It was fixed in BSP 0.8 and above. 

[AlexT_Intelhas](http://alextgalileo.altervista.org) newer unofficial builds on his site.

On firmware version 1.0.0 that problem goes away.


[Github: meta-alext-galileo](https://github.com/alext-mkrs/meta-alext-galileo)


## theTimerOne example bug 

https://communities.intel.com/message/219001#219001

Intel提供的例程Timerone->ISRBlink有问题.

	there's actually a second argument to attachInterrupt(), which is missing in the sketch.
	It has a default value of -1, which causes the callback not to be attached due to inconsistency with a certain check within that attachInterrupt().
	
eg.

	Timer1.initialize(100000); // the timer period is 100000 useconds, that is 0.1 sec  
	Timer1.attachInterrupt(timerIsr, 500000); // the callback will be called on each 5th timer interrupt, i.e. every 0.5 sec  
 
	
##Using rx tx instead of headphone jack?

https://communities.intel.com/message/230742#230742

Headphone jack是UART1, 对应/dev/ttyS1, 而板子上的RX,TX是UART0, 对应/dev/ttyS0.   
所以如果想要代替, 需要在grub配置文件中修改linux console.

##/tmp/log.txt File Too Big

https://communities.intel.com/message/231768#231768

	That is the sketch debug log.You could turn off logging, or find the file that has the trace statement. I would start looking for a file called mux-info.c or .cpp in the sketch libraries.
	
	 
	
	a function called "trace_info()" being called in "mux.c", on line 88. 
	
	
##Start Serial Console on Galileo using Linux, w/ USB->UART FTDI cable

https://communities.intel.com/message/231778#231778


##SoftwareSerial Library

https://communities.intel.com/message/231749#231749

	"fatal error: SoftwareSerial.h: No such file or directory
	
The SoftwareSerial library is not available in the current release.

##[GPIO pin goes high on reboot of Galileo board?](https://communities.intel.com/message/231619#231619)

	Pins have pullups enabled at reset time.
	
This behavior is listed in the known issues of [the Release notes](https://communities.intel.com/docs/DOC-21837) you can take a look at it in section 1.7.9.

##[SD Library compilation error](https://communities.intel.com/message/208624#208624)

Arduino: 1.5.3 (Mac OS X), Board: "Intel® Galileo"
Arduino.app/Contents/Resources/Java/hardware/arduino/x86/libraries/SD/SD.h:5:18: fatal error: string: No such file or directory
compilation terminated.

	This is a bug in the way Yocto builds the gcc toolchain for the Mac. The net effect is that C++ .h header files are not found. As a workaround, can you please try to create a symbolic link like this?

Don't copy and paste, retype the line into terminal inside your Arduino For Galileo app folder. (mine is called ArduiG)	
	
	cd /Applications/ArduiG.app/Contents/Resources/Java
	ln -s . hardware/tools/x86/i586-poky-linux-uclibc/usr/include/c++/4.2.1
	
	
##[the random() function](https://communities.intel.com/message/211224#211224)

error: too many arguments to function 'long int random()'

Galileo没有实现Arduino的random(int, int), 以及random(int), 而是实现了linux下的random(),
所以应该将其更改为

	long int rand();
对应函数的用法。

int a = random(10);

	randNumber = small + (random() % (big - small));
