/*
  SD card read/write
 
 This example shows how to read and write data to and from an SD card file 	
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 	 
 */

#include <SD.h>
#include <SCA60C.h>  // use it in test3();

unsigned long time_start = 0, time_end = 0;
#define TIME_S     (time_start = millis())
#define TIME_E     (time_end = millis(), Serial.print("Time:"), Serial.println(time_end - time_start))


File sd_file;
SCA60C sca(A0, A1, 2.23, 2.38);
unsigned long run_app_count = 0;
unsigned long file_count = 0;
String dir_name1 = "sca60c_", dir_name2 = "1";
String name_part1 = "1", name_part2 = ".txt";
String name_all, dir_all;
//dir_all = dir_name1 + dir_name2;
//name_all = dir_all + name_part1 + name_part2;
String temp_str;
char char_buffer[100];

void setup() {
  Serial.begin(9600);
  
  system("/bin/date 0616000014");

  //get the value of run_app_count by reading a file.
  sd_file = SD.open("sca60c.conf", FILE_WRITE);
  run_app_count = (int)(sd_file.read());
  sd_file.close();
  SD.remove("sca60c.conf");
  //Serial.println(run_app_count);
  sd_file = SD.open("sca60c.conf", FILE_WRITE); // rewrite a new value of the run_app_count.
  sd_file.write(run_app_count+1);
  //sd_file.write(1); //return run_app_count = 1;
  sd_file.close();

  //now, mkdir a new directory.
  my_itoa(run_app_count, char_buffer);
  dir_name2 = char_buffer;  
  dir_all = dir_name1 + dir_name2;
  dir_all.toCharArray(char_buffer, dir_all.length() + 1);
  SD.mkdir(char_buffer);

  //now, create a new file and save data.
  sd_file = get_new_filename();
  time_start = millis();
}

int data_count = 0;
void loop() {
  time_end = millis();
  if((time_end - time_start) >= 5) //Sample Time = 100ms.
  {
    time_start = time_end;
    sd_file.print(sca.GetAngleX());
    sd_file.print(",Y:");
    sd_file.println(sca.GetAngleY());
    if(++data_count >= 10000) // change a sd file to save data.
    {
      data_count = 0;
      sd_file.close();
      //if(file_count >= 60)
        //system("/sbin/reboot");  //10 min will reboot again.
        
      sd_file = get_new_filename();
      sd_file.println("---->");
    }
  }
}

//反转字符串  
void reverse(char *s)  
{  
  char temp;  
  char *p = s;    //p指向s的头部  
  char *q = s;    //q指向s的尾部  
  while(*q)  
    ++q;  
  q--;  

  //交换移动指针，直到p和q交叉  
  while(q > p)  
  {  
    temp = *p;  
    *p++ = *q;  
    *q-- = temp;  
  }  
}  

void my_itoa(int n, char* s)
{  
  int i = 0, isNegative = 0;  

  if((isNegative = n) < 0) //如果是负数，先转为正数  
  {  
    n = -n;  
  }  
  do      //从各位开始变为字符，直到最高位，最后应该反转  
  {  
    s[i++] = n % 10 + '0';  
    n = n / 10;  
  }
  while(n > 0);  

  if(isNegative < 0)   //如果是负数，补上负号  
  {  
    s[i++] = '-';  
  }  
  s[i] = '\0';    //最后加上字符串结束符  
  reverse(s);    
}  

//get a new filename and open it .
File get_new_filename()
{
  my_itoa(++file_count, char_buffer);
  name_part1 = char_buffer;
  name_all = dir_all + "/" + name_part1 + name_part2;
  name_all.toCharArray(char_buffer, name_all.length() + 1);
  return (SD.open(char_buffer, FILE_WRITE));
}

