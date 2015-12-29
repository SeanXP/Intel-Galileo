#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 0.1,8,0, DIRECT);

int count = 0;

void setup()
{
  Setpoint = 200;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  

  Serial.begin(9600);
}

void loop()
{
  myPID.Compute();
  Input = Output;
  count++;
  Serial.print(count);
  Serial.print(" : ");
  Serial.println(Output);
  delay(100);
}


