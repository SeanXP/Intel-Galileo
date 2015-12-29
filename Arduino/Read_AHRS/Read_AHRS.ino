/*
  Serial1 - /dev/ttyS0   in Galileo - AHRS CJMCU
 Serial  - /dev/ttyGS0  in Galileo - Arduion IDE Serial Monitor
 
 
 XXX: AHRS must connect the VCC-5V with Galileo!

  read three double data from AHRS(always keep output data ).
  
 
 */

// AHRS output : #YPR: xxx, xxx, xxx
double yaw = 0;
double pitch = 0;
double roll = 0;

char ch = 'c';
int len = 100;
char buf[100];

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600);
  Serial.begin(9600);
  
  delay(1000);
}

void loop() {

  //read_all();
  //read_buffer();
  //read_find();

  //read three double data from AHRS in 100ms interval.
  //100ms will make Arduino Serial buffer overflow, so need read the buffer and discard them.
  
  read_data();
  Serial.print(yaw);
  Serial.print(",");  
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print("\n"); 
  delay(100);
  
  //Serial1.flush();
}

// method 1:
// read all data in all time. 
void read_all()
{
  char read_data;
  // check if data has been sent from AHRS:
  if (Serial1.available()) {
    // read the most recent byte (which will be from 0 to 255):
    read_data = Serial1.read();
    Serial.print(read_data);  // print to Serial(/dev/ttyGS0)
  }

  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    read_data = Serial.read();
    Serial1.print(read_data);  // print to Serial(/dev/ttyGS0)
  }
}

//method 2:
//read data by using buffer in sometime.
void read_buffer()
{
  int len = 50;
  char buffer[len];

  // check if data has been sent from AHRS:
  if (Serial1.available()) {
 
       
    // read the most recent byte (which will be from 0 to 255):
    Serial1.readBytes(buffer, len);
    Serial.print(buffer);  // print to Serial(/dev/ttyGS0)
    Serial.print("\n-------\n");
  }
}

//method 3:
// read three data from AHRS.
void read_data()
{ 
  if(Serial1.available())
  {
    //read buffer of Arduino and discard them for get fresh data soon.
    Serial1.readBytes(buf, len);

    //stop while until find a '#', for sync data.
    while(ch != '=')
    {
      ch = Serial1.read();
    }
    while(ch != '#')
    {
      ch = Serial1.read();
    }
  
    //get three double type of data.
    yaw = Serial1.parseFloat();
    pitch = Serial1.parseFloat();
    roll = Serial1.parseFloat(); 
  }
}

