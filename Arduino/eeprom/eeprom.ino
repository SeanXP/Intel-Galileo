/*
 * EEPROM
 *
 */

#include <EEPROM.h>

// start reading from the first byte (address 0) of the EEPROM
// 11kbytes, so address 0~11264
int address = 0;
byte value;

void setup()
{
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

void loop()
{
  address = 0;
  
  EEPROM.write(address,110);
  // read a byte from the current address of the EEPROM
  value = EEPROM.read(address);
  Serial.print("address:");
  Serial.print(address, DEC);
  Serial.print("\tValue:\t");
  Serial.print(value, DEC);
  Serial.println();
}

void eeprom_clear()
{
    // write a 0 to all 11264 bytes of the EEPROM
    for (int i = 0; i < 11264; i++)
      EEPROM.write(i, 0);   //the EEPROM can only hold a value from 0 to 255.
}


