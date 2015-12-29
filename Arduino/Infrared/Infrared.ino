//Infred distance sensor -- GP2Y0A02YK0F
/**
 Detection range: 20cm ~ 150cm
 The detection interval: 50ms 
 
 //Linear relationship
 10cm - 2.4V (0~10cm , 0V -> 2.4V , distance = Voltage / 2.4;)
 
 //Nonlinear relationship
 15cm - 2.75V
 20cm - 2.5V
 30cm - 2V
 40cm - 1.5V
 50cm - 1.25V
 60cm - 1V
 90cm - 0.75V
 130cm - 0.5V
 
 ----------------------------------------------------------------
 // real relationship
 
**/

//select the input pin for the Infred distance sensor 
int pin_M1 = A2;
int pin_M2 = A3;
int pin_M3 = A4;

//variable to store the value coming from the sensor.
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  sensorValue1 = analogRead(pin_M1); //6ms
  //sensorValue2 = analogRead(pin_M2); //6ms
  //AD, 0V -> 0, 5V -> 1024, voltage = sensorValue * 5 / 1024 = sensorValue / 204.8;
  
  Serial.print(sensorValue1 / 204.8);
  Serial.print(" ");
  Serial.println(get_distance(sensorValue1 / 204.8));
}


int get_distance(double voltage)
{
  // 1s motor yi shang.
  
  /*
  10      2.59 -
  11      2.56 - 2.59
  12      2.55 - 2.56
  13      2.53 - 2.54
  14      2.52 - 2.53
  15      2.51 - 2.52
  16      2.49 - 2.50 
  17      2.48 - 2.49
  18      2.45 - 2.48 -----
  19      2.43 - 2.44
  20      2.42 - 2.43
  21      2.38 - 2.42
  22      2.37 - 2.38
  23      2.33 - 2.37
  24      2.33 - 2.37
  25      2.30 - 2.33
  26      2.27 - 2.30
  27      2.24 - 2.27
  27      2.20 - 2.24  
  */
  
  if(voltage > 2.59) return 10;
  else if(voltage > 2.56) return 11;
  else if(voltage > 2.55) return 12;
  else if(voltage > 2.53) return 13;
  else if(voltage > 2.52) return 14;
  else if(voltage > 2.51) return 15;
  else if(voltage > 2.49) return 16;
  else if(voltage > 2.48) return 17;
  else if(voltage > 2.45) return 18;
  else if(voltage > 2.43) return 19;
  else if(voltage > 2.42) return 20;
  else if(voltage > 2.38) return 21;
  else if(voltage > 2.37) return 22;
  else if(voltage > 2.34) return 23;
  else if(voltage > 2.33) return 24;
  else if(voltage > 2.30) return 25;
  else if(voltage > 2.27) return 26;
  return 27;
}
