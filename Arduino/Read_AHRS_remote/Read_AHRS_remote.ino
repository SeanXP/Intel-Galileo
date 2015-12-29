/*
  Serial1 - /dev/ttyS0   in Galileo - AHRS CJMCU
 Serial  - /dev/ttyGS0  in Galileo - Arduion IDE Serial Monitor
 
 
 XXX: AHRS must connect the VCC-5V with Galileo!

  read three double data from AHRS(always keep output data ).
  write data to Galileo /home/root/data.txt
  
 
 */

// AHRS output : #YPR: xxx, xxx, xxx
double yaw = 0;
double pitch = 0;
double roll = 0;

char ch = 'c';
int len = 100;
char buf[100];

FILE *fp;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(57600);
 // Serial.begin(9600);
  
  fp = fopen("/home/root/ahrs_data.txt", "w");
  
  delay(1000);
}

void loop() {

  //read_all();
  //read_buffer();
  //read_find();

  //read three double data from AHRS in 100ms interval.
  //100ms will make Arduino Serial buffer overflow, so need read the buffer and discard them.
  
  read_data();

  fprintf(fp,"%f,%f,%f\n", yaw, pitch, roll);
 
 /* Serial.print(yaw);
  Serial.print(",");  
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print("\n"); 
  */
  
  delay(100);
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

