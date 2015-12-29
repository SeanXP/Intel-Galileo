/*
  This example reads an file from a micro-SD card.
  
   * SD card attached to SPI bus as follows:
     ** MOSI - pin 11
     ** MISO - pin 12
     ** CLK - pin 13
     ** CS - pin 4
*/


// include the necessary libraries
#include <SD.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
#define sd_cs  4

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // wait for serial line to be ready
  }
  
  
   // try to access the SD card. If that fails (e.g.
  // no card present), the setup process will stop.
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(sd_cs)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("OK!card initialized.");
  
  
}

void loop() {
  
  String dataString = "write to SD file.";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);  
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
