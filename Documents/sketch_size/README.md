Arduino Sketch Size Limits
====


https://communities.intel.com/message/222971#222971


The Arduino IDE defines sketch upload limits in its board.txt file. 

You can find the file at:
 
	[IDE location]\hardware\arduino\x86\boards.txt
 
You should see the following entry:
 
	izmir_fd.upload.maximum_size=262144

This defines the sketch size limit of the Galileo board in the Arduino IDE. 
Modify as needed and reload the IDE for changes to take effect.


-----

I suspect that this is an oversight, but I can confirm that if you edit the line in arduino/hardware/arduino/x86/boards.txt, you can change it to something closer to 200MB:
 
izmir_fd.upload.maximum_size=200000000
 
I tested with this sketch:
 
	int array[1000000];
 
	void setup() {
		for (int i=0; i<1000000; ++i)
		array[i] = rand();
		Serial.begin(115200);
	}
 
	void loop() {
		for (int i=0; i<1000000; ++i)
		Serial.println(array[i]);
	}