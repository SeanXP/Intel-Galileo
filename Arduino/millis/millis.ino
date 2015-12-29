/********************
  test the speed of some function in Galileo Arduino.

  test in Intel Galileo( LINUX_IMAGE_FOR_SD_Intel_Galileo_v1.0.0 ).
  compile in Arduino for Galileo ( Intel_Galileo_Arduino_SW_1.5.3_on_MacOSX _v1.0.0 ).
-------------------------->
  pinMode()        25ms
  
  digitalWrite()   
     if IO has set pinMode(xxx, OUTPUT), digitalWrite() will cost 2~3 ms.
     if IO has not set pinMOde, digitalWrite() will cost 0~1 ms when write LOW and 19~20ms when write HIGH.
  
  analogWrite();   //first need enable PWM output (~40ms), then need 5ms. [PIN 0~13]
  analogRead();    6ms [PIN A0 ~ A5]
    Analog Input: PIN A0~A5.
    can't use it as Analog Output.(No DAC in Intel Galileo).
    
  '+ - * / 'operation calculate       very fast.
  
  
-------------------------->
Arduino millis(): Returns the number of milliseconds since the Arduino board began running the current program.
  unsigned long millis();
********************/

unsigned long time_start = 0, time_end = 0;
#define TIME_S     (time_start = millis())
#define TIME_E     (time_end = millis(), Serial.print("Time:"), Serial.println(time_end - time_start))

/*Usage: (need to begin 'Serial' first...)
TIME_S;
  test_function();  //calculate the cost time of this function.
TIME_E;
*/

void setup()
{
  Serial.begin(9600);
  pinMode(2, OUTPUT); //25ms
}


void loop()
{
  //test_millis();
  
  //test_pinmode();       // 25ms
  //test_digitalWrite(); //cost 2~3 ms if set pinMode, and write (X -> high -> low) need 5ms
  //test_analogWrite(); //first need enable PWM output (~40ms), then need 5ms.
  //test_analogRead();  //6ms
  test_serial_output(); // 100 char, 6~12ms
  //test_cal_int();       // '+' operation, many time will spend 1ms, so it's fast.
  //test_cal_double();        // '*' operation, so fast too.
}

void test_millis()
{
  Serial.print("Time: ");
  time_start = millis(); 
  //Returns Number of milliseconds since the program started (unsigned long)
  //prints time since program started
  Serial.println(time_start);
  // wait a second so as not to send massive amounts of data
  delay(1000);
  Serial.print("delay 1000ms, then ");
}

void test_pinmode()
{
  TIME_S;
  pinMode(2, OUTPUT);
  TIME_E;
}

void test_digitalWrite()
{
  /*
  if IO has set pinMode(xxx, OUTPUT), digitalWrite() will cost 2~3 ms.
  if IO has not set pinMOde, digitalWrite() will cost 0~1 ms when write LOW and 19~20ms when write HIGH.
  */
  
  TIME_S;
  //PIN 2 (output MODE).
  //digitalWrite(2, HIGH);     //2~3 ms
  //digitalWrite(2, LOW);      //2~3 ms
  
  //IO 2, (if not set output mode in setup() ).
  //digitalWrite(2, LOW);      //0~1 ms
  //digitalWrite(2, HIGH);     //23~24 ms
  
  //IO 3, (not set output mode int setup() ).
  //digitalWrite(3, LOW);        //0~1 ms
  //digitalWrite(3, HIGH);        //23~24 ms
  TIME_E;
}

void test_analogWrite()
{
  TIME_S;
  analogWrite(6, 1);      //first need enable PWM output (~40ms), then need 5ms.
  TIME_E;
}

int analog_input_0 = 0;
void test_analogRead()
{
  TIME_S;
  analog_input_0 = analogRead(A0);    //5~6 ms
  TIME_E;
  Serial.println(analog_input_0);
}

char ch = 'a';
int ch_i = 0;
void test_serial_output()
{
  TIME_S;
  for(ch_i = 0; ch_i < 100; ch_i++)
    Serial.print(ch);
  TIME_E;              // 6~12 ms
  
  delay(200);
}

int i = 0;
int int_value = 0;
long long_value = 0;
void test_cal_int()
{
  int_value = 0;
  TIME_S;
  for(i = 0; i < 100; i++)
    int_value += i;
  TIME_E;
}

double dou_value = 1;
void test_cal_double()
{
  dou_value = 1;
  TIME_S;
  for(i = 0; i < 1000; i++)
    dou_value *= 1.01;
  TIME_E;
}
