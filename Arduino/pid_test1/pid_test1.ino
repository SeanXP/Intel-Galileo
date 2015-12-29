
//br3ttb/Arduino-PID-Library
//https://github.com/br3ttb/Arduino-PID-Library

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
int SampleTime = 1000; //1 sec

int count = 0;

void setup()
{
  Serial.begin(9600);

  SetSampleTime(100);
  
  //SetTunings(0.8, 0.5, 0);   // 195~200 -> count = 111;
  //SetTunings(0.9, 0.5, 0);  //                      115
  //SetTunings(0.9, 0.3, 0);  //                      192
  //SetTunings(0.9, 0.8, 0);  //                      75
  //SetTunings(0.9, 1.0, 0);  //                      70
  //SetTunings(0.9, 1.5, 0);  //                      115
  //SetTunings(0.87 , 1.0, 0);  //                    60         
  //SetTunings(0.83, 1.0, 0);   //                    55
  //SetTunings(0.83, 1.3, 0);   //                    45
  //SetTunings(0.80, 1.4, 0);   //                    39
  //SetTunings(0.750, 1.5, 0);   //                   36
  //SetTunings(0.700, 1.5, 0);   //                   35
  //SetTunings(0.700, 1.8, 0);   //                   29
  //SetTunings(0.600, 2.0, 0);   //                   25
  //SetTunings(0.500, 2.5, 0);   //                   19
  //SetTunings(0.400, 3.0, 0);   //                   15
  //SetTunings(0.300, 4.0, 0);   //                   10
  //SetTunings(0.200, 5.0, 0);   //                   7
  //SetTunings(0.100, 6.0, 0);   //                   5
  //SetTunings(0.100, 8.0, 0);   //                   3
  
  SetTunings(0.10, 8.0, 0);
  Input = 0;
  Output = 0;
  Setpoint = 200;
  lastTime = 0;
  errSum = 0;
  lastErr = 0;

}

void loop()
{

   Compute();
   count++;
   Serial.print(count);
   Serial.print(": ");
   Serial.println(Output);
   Input = Output;
   delay(100);
}

void Compute()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=SampleTime)
  {
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    errSum += error;
    double dErr = (error - lastErr);

    /*Compute PID Output*/
    Output = kp * error + ki * errSum + kd * dErr;

    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
  }
}

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime
      / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}


