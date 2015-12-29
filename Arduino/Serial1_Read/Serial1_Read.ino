/*
  Serial1 - /dev/ttyS0 in Galileo
 */

void setup() {

  // initialize the serial communication:
  Serial1.begin(9600);
}

void loop() {
  byte read_data;

  // check if data has been sent from the computer:
  if (Serial1.available()) {
    // read the most recent byte (which will be from 0 to 255):
    read_data = Serial1.read();

    Serial1.print("I got ");
    Serial1.println(read_data);
  }
}



