//read Galileo board's Mac address from Arduino Sketch...
void setup()
{   

  
  Serial.begin(9600);


}
void loop() {
  Serial.println("read1().....");
  read1();
  Serial.println("read2().....");
  read2();
  delay(3000);
}

void read1()
{
  system("cat /sys/class/net/eth0/address > mac.txt");
  char mac_str[100];
  FILE *fp;
  fp = fopen("mac.txt","r");
  while(fgets(mac_str, 100, fp) != NULL)
   ;
  fclose(fp);
  Serial.println(mac_str);
}

void read2()
{
  Serial.println(system("cat /sys/class/net/eth0/address > /dev/ttyGS0"));
}
