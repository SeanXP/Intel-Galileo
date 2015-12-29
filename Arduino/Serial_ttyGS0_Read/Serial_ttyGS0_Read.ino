/*
    Galileo /dev/ttyGS0 (client usb) 
    
*/


void setup()
{
  // initialize the serial communication:
  Serial.begin(9600);
}

void loop() {
  byte read_data;

  // check if data has been sent from the computer:
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    read_data = Serial1.read();

    Serial.print("I got ");
    Serial.println(read_data);
  }
}
