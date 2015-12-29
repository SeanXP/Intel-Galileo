/************************************************************************
 * 
 * read data from SCA60C module to PID controller for control three motor.
 * AIM: Makes three motors to keep the platform level.
 * 
 *
 * SCA60C( N1000060 accelerometer) 
 * 
 *        (Vo2 output pin)
 * 
 *               UP (face to Ground, Motor 1)
 *               ^
 *               |
 *               |  
 * LEFT <--------|--------- RIGHT
 *               |
 *               |
 *             DOWN
 * 
 *        (Vo1 output pin)
 * 
 * 
 * (0.5~4.5V) (-90~90 degrees)
 * Vo1: left-right   (x axis), LEFT is X-axis positive direction.
 * Vo2: up-down      (y axis), UP   is Y-axis positive direction.
 *
 *          M1          ^ Y
 *         / \          | 
 *        M2  M3        L___> X
 *
 *    Y-axis up, SCA60C_angle_Y > 0, PID output_y < 0, M1 down.
 *    X-axis up, SCA60C_angle_X > 0, PID output_x < 0, M3 down & M2 up.
 *-----------------------------------------------------------------------
 * Connect:
 * 
 * SCA60C:
 * Vo1 - Galileo Analog Input A1    --  green   wire 
 * Vo2 - Galileo Analog Input A0    --  blue    wire 
 * VCC - Galileo 5V                 --  red     wire
 * GND - Galileo GND                --  brown   wire
 *
 * (0.5~4.5V) (-90~90 degrees)
 * Vo1: left-right   (x axis)
 * Vo2: up-down      (y axis)
 * 
 *  2   3    4    5     6      7
 *  D1  P1   D2   P2    P3    D3
 *
 * Motor: (pwm 0~255) (dir: 0 up, 1 down)
 * pwm1:  IO 3
 * pwm2:  IO 5
 * pwm3:  IO 6
 * 
 * dir1:  IO 2
 * dir2:  IO 4
 * dir3:  IO 7
 * 
 * read three double data from AHRS(always keep output data ).
 * 
 * 
 ***********************************************************************/

#define SERIAL_DEBUG   //open the Serial for debugging...turn off by commenting this row.
#define KALMANFILTER   //open kalman filter.

#include "Wire.h"
#include <PID_v2.h>
#include <SCA60C.h>

/***** timer *****/
int time_start = 0, time_end = 0;
#define TIME_S (time_start = millis())
#define TIME_E (time_end = millis(),Serial.print("Time:"),Serial.println(time_end - time_start))

/***** SCA60C module in <SCA60C.h> *****/
//Vo1 -> A0, Vo2 -> A1
SCA60C sca(A1, A0, 2.22, 2.34); // set the output offset voltage when horizontal. 
double angle_x = 0;
double angle_y = 0;

/***** Motor PWM *****/
// All Arduino PWM pin is : 3,5,6,9,10,11
int pwm1 = 3;
int pwm2 = 5;
int pwm3 = 6;
// motor dir
int dir1 = 2;
int dir2 = 4;
int dir3 = 7;

//need set pinMode OUTPUT first!
//digitalWrite() cost 2~3 ms each time.
#define M1_UP    (digitalWrite(dir1, LOW))  
#define M1_DOWN  (digitalWrite(dir1, HIGH))
#define M2_UP    (digitalWrite(dir2, LOW))
#define M2_DOWN  (digitalWrite(dir2, HIGH))
#define M3_UP    (digitalWrite(dir3, LOW))
#define M3_DOWN  (digitalWrite(dir3, HIGH))
//set Motor speed by set PWM.
#define M1_SPEED(X)  (setPwm(pwm1, (X)))
#define M2_SPEED(X)  (setPwm(pwm2, (X)))
#define M3_SPEED(X)  (setPwm(pwm3, (X)))

//just for serial debug output index.
int counter = 0;

/***** PID controller *****/
double KP_x = 80, KI_x = 10, KD_x = 1;
double KP_y = 80, KI_y = 10, KD_y = 1;
//Define Variables we'll be connecting to
double Setpoint_x = 0, Input_x = 0, Output_x = 0;
double Setpoint_y = 0, Input_y = 0, Output_y = 0;
//PID output Limit, ±100;    Stable range = ±3.
double OutMinLimit_x = -250, OutMaxLimit_x = 250, OutputRange_x = 10;
double OutMinLimit_y = -250, OutMaxLimit_y = 250, OutputRange_y = 10;
double last_angle = 0;
//Specify the links and initial tuning parameters
PID myPID_x(&Input_x, &Output_x, &Setpoint_x, KP_x, KI_x ,KD_x , DIRECT);
PID myPID_y(&Input_y, &Output_y, &Setpoint_y, KP_y, KI_y ,KD_y , DIRECT);
boolean output_changed = false; // the return value of PID.Compute().

//finite-state machine
#define STATE_STOP  0  //dir_state = 0, stop;
#define STATE_UP    1  //dir_state = 1, up;
#define STATE_DOWN  2  //dir_state = 2, down;
int dir_state_x = STATE_STOP;
int dir_state_y = STATE_STOP;

//variable for decoupling();
int move1 = 0, move2 = 0, move3 = 0;
int old_move1 = 0, old_move2 = 0, old_move3 = 0; // record the last value of move2 & move3.

//Kalman filter
double raw_data[2], kalman_data[2];


//PID Tuning...MUST open SERIAL_DEBUG.
pthread_t mythread1;
int read_data = 0;  //read data from serial.
String inString = "";  //switch string to int.(Serial.read() return char value.)
int int_value[3];
int int_index = 0;    
boolean change_PID = false;  // the flag of change PID parameter.

//button - A4
pthread_t mythread2;
int pin_key = A4;
int key_value = 0;
boolean run_pid_flag = false;
boolean reverse_flag = false;

//select the input pin for the Infred distance sensor 
int pin_M1 = A2;
//int pin_M2 = A3;
int pin_M3 = A3;
//variable to store the value coming from the sensor.
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
//-------------------------------------------------------------------------------//

void setup() {
#ifdef SERIAL_DEBUG
  //turn on Serial for debugging...
  Serial.begin(115200);   
  Serial.println("Motor ready...");
#endif

  
  //begin Wire for controlling PWM by I2C.
  Wire.begin();
  setPwmI2C (pwm1, 0);  //first need 50ms~ (because of enable pwm output), and then need ~11ms
  setPwmI2C (pwm2, 0);
  setPwmI2C (pwm3, 0);

  pinMode(dir1, OUTPUT); //cost 25ms each time.
  pinMode(dir2, OUTPUT); 
  pinMode(dir3, OUTPUT);
  //----> need 220ms.

  //set an initial position for three motors.
  downall(3000);
  Serial.println("Motor ok now.");

  //PID set
  Setpoint_x = dir_state_x = 0; // set aim of X axis angle.
  Setpoint_y = dir_state_y = 0; // set aim of Y axis angle.
  //turn the PID on.
  myPID_x.SetSampleTime(80);
  myPID_x.SetOutputLimits(OutMinLimit_x, OutMaxLimit_x); //set the PID output Limits; 
  myPID_x.SetMode(AUTOMATIC);
  myPID_y.SetSampleTime(80);
  myPID_y.SetOutputLimits(OutMinLimit_y, OutMaxLimit_y); //set the PID output Limits; 
  myPID_y.SetMode(AUTOMATIC);
  //PID set over ............

#ifdef SERIAL_DEBUG
  //create a new thread to set PID parameter from Serial.
  if(pthread_create(&mythread1, NULL, &thread_function, NULL))  //return 0 when success.
  {
    Serial.println("Error creating thread.\n");
  }
#endif

  //create a new thread to set PID parameter from Serial.
  if(pthread_create(&mythread2, NULL, &button_function, NULL))  //return 0 when success.
  {
    Serial.println("Error creating thread.\n");
  }

}

//-------------------------------------------------------------------------------//
void loop() {

  if(run_pid_flag == true)
  {
    if(reverse_flag == true) // begin pid.
    {
      upall(2200);
      reverse_flag = false;
      run_pid_flag = true;
    }
    
    // ---------------> go on pid, don't need begin.  

    //1. read data of X,Y axis angle.
    Input_x = sca.GetAngleX();    //need 6ms, because of analogRead();
    Input_y = sca.GetAngleY();
    
#ifdef KALMANFILTER  
    //1.5, Kalman filter.
    raw_data[0] = Input_x;
    raw_data[1] = Input_y;
    kalmanfilter(raw_data, kalman_data); 
    Input_x = kalman_data[0];
    Input_y = kalman_data[1];
#endif

    /*
  //1.5 , FANG ZHI DOU DONG.
     if(Input_x * last_angle >= 0)
     {
     //2. PID, just need 1ms to calculate pid output.
     //   return new value of (Output_x & Output_y).
     myPID_x.Compute();
     output_changed = myPID_y.Compute();
     }
     else
     {
     output_changed = true;
     }
     */
/*
    if(abs(Input_x) > 3)
      myPID_x.SetTunings(KP_x + 10, KI_x, KD_x);
    else
      myPID_x.SetTunings(KP_x, KI_x, KD_x);
    if(abs(Input_y) > 3)
      myPID_y.SetTunings(KP_y + 10, KI_y, KD_y);
    else
      myPID_y.SetTunings(KP_y, KI_y, KD_y);
*/
    
    myPID_x.Compute();
    output_changed = myPID_y.Compute();
    last_angle = Input_x;

    //2.5 serial output for debugging ...
#ifdef SERIAL_DEBUG      //need 10ms;
    counter++;
    if(output_changed == true)
    {
      Serial.print(counter);
      Serial.print(", In:");
      Serial.print(Input_x);
      Serial.print("/");
      Serial.print(Input_y);
      Serial.print(", Out:");
      Serial.print(Output_x);
      Serial.print("/");
      Serial.print(Output_y);
    }
#endif


    //3. decoupling for three motor.
    //set dir need 2ms. set pwm need 12ms. each motor will cost (2 + 12 ms), all cost is 40ms.
    if(output_changed == true)
      decoupling();
    
    sensorValue1 = analogRead(pin_M1);
    if((sensorValue1 / 204.8) <= 2.25) // too low
    {
      upall(1000);
    }
    
  }
  else // run_pid_flag == false;
  {
    if(reverse_flag == true) // stop pid
    {
      downall(3000);
      reverse_flag = false;
      run_pid_flag = false;
    }
    else // go on wait...
    {
      delay(50);
    }
  }
  
  /*
  Serial.print("run_pid_flag = ");
  Serial.println(run_pid_flag);
  */


#ifdef SERIAL_DEBUG
  if(change_PID == true)
  {
    change_PID = false;
    int_index = 0;
    KP_x = KP_y = int_value[0];
    KI_x = KI_y = int_value[1];
    KD_x = KD_y = int_value[2];
    myPID_x.SetTunings(int_value[0], int_value[1], int_value[2]);
    myPID_y.SetTunings(int_value[0], int_value[1], int_value[2]);
  }
#endif
  //loop() is over.
}

//-------------------------------------------------------------------------------//

//decoupling X/Y axis angle to three motor's speed.
//Direction of the motor 2 and motor 3 with an opposite direction of the motor.
//if PID output > 0, should up; if output < 0, down!
void decoupling()
{
  //motor 1 & motor 2,3 <---> Y axis.
  if(Output_y > OutputRange_y) // Output_y > 3 * KP_y, Y axis need to up!
  {
    if(dir_state_y != STATE_UP) // if not UP state, change state to UP now.
    {
      dir_state_y = STATE_UP;
      M1_UP;
    }
#ifdef SERIAL_DEBUG
    Serial.print(",M1^");
#endif
    //M2,3 need to DOWN.
    move2 = -Output_y;
    move3 = -Output_y;
  }
  else if(Output_y < -OutputRange_y) // Output_y < -3 * Kp_y, Y axis need down!
  { 
    if(dir_state_y != STATE_DOWN) // if not DOWN state, change state to DOWN now.
    {
      dir_state_y = STATE_DOWN;
      M1_DOWN;
    }
#ifdef SERIAL_DEBUG
    Serial.print(",M1V");
#endif
    //M2,3 need to down.
    move2 = -Output_y;
    move3 = -Output_y;
  }
  else //else , Input - range < Output < Input + range. Y axis in Range(-3, +3), mean STABLE!
  {
    dir_state_y = STATE_STOP;
    Output_y = 10;      //a very little pwm value means a very slow speed of Motor.
    move2 = move3 = 0;

#ifdef SERIAL_DEBUG
    Serial.print(",M1-");
#endif
  }

  M1_SPEED(abs(Output_y));

  //X axis, Motor2 & 3;
  if(Output_x > OutputRange_x) // X axis need to up, M3.
  {
    move3 += Output_x;
    move2 -= Output_x;
#ifdef SERIAL_DEBUG
    Serial.print(",X^"); //X axis is stable.
#endif
  }
  else if(Output_x < -OutputRange_x) //X axis need to down, M3.
  { 
    move3 += Output_x;
    move2 -= Output_x;
#ifdef SERIAL_DEBUG
    Serial.print(",XV"); //X axis is stable.
#endif
  }
  else //else , Input - range < Output < Input + range. X axis in Range(-3, +3).
  {
#ifdef SERIAL_DEBUG
    Serial.print(",X-"); //X axis is stable.
#endif
  }

#ifdef SERIAL_DEBUG
  //set Motor 2 & 3 by judging the value of move2&3;
  Serial.print(",move:");
  Serial.print(move2);
  Serial.print(",");
  Serial.print(move3);
#endif

  if(move2 != old_move2)
  {
    old_move2 = move2;  //record the new value.
    //M2 direction
    if(move2 > 0)
      M2_UP;
    else
      M2_DOWN;

    M2_SPEED(abs(move2));
  }

  if(move3 != old_move3)
  {
    old_move3 = move3;  //record the new value.
    if(move3 > 0)
      M3_UP;
    else
      M3_DOWN;

    M3_SPEED(abs(move3));
  }

#ifdef SERIAL_DEBUG
  Serial.println(" ");
#endif
}

void downall(int time)
{
  //set a high speed.
  Set_Motor_Speed(250);
  //set dir(down). 
  M1_DOWN;
  M2_DOWN;
  M3_DOWN;
  delay(time); //delay some times.
  Set_Motor_Speed(0);
}

void upall(int time)
{
  //set a high speed.
  Set_Motor_Speed(250);
  //set dir(up). 
  M1_UP;
  M2_UP;
  M3_UP;
  delay(time); //delay some times.
  Set_Motor_Speed(0);
}

void Set_Motor_Speed(int pwm_speed)
{
  //set three motor's PWM value for control speed.(0 is stop and 255 is best speed of motor.)
  M1_SPEED(pwm_speed);
  M2_SPEED(pwm_speed);
  M3_SPEED(pwm_speed);
}



//set PWM use I2C 
//set pin(_iPinNum) output PWM with duty cycle of (_iPwmVal / 255).
void setPwmI2C(int _iPinNum, int _iPwmVal)
{
  if(_iPwmVal > 250) _iPwmVal = 250;
  // Select pin to write I2C commands to
  analogWrite(_iPinNum,1);            //analogWrite(pin, 1) will enable the PWM on the pin. 
  //but AnalogWrite(pin,0) does not disable the pwm.

  //Then there’s the I2C commands...

  // Select programmable PWM CLK source
  Wire.beginTransmission(0x20);
  Wire.write(0x29);  // CY8C9540A -  Config (29h)
  //xxxxx000b - 32kHz (default)
  //xxxxx001b - 24Mhz
  //xxxxx010b - 1.5Mhz
  //xxxxx011b - 93.75 kHz
  //xxxxx100b - 367.6 Hz (programmable)
  //xxxxx101b - Previous PWM
  //Wire.write(0x04);  // xxxxx100b - 367.6 Hz (programmable)
  Wire.write(0x02);    // 1.5MHZ , but output just 6khz. just use it~
  Wire.endTransmission();

  // Set divider the PWM CLK .
  Wire.beginTransmission(0x20);
  Wire.write(0x2C);
  Wire.write(0x01);
  Wire.endTransmission();

  // Set period register (2Ah) - 0x01 / 0xff, need 10ms
  Wire.beginTransmission(0x20);                
  Wire.write(0x2a);
  //xxxx0xxxb - Falling pulse edge (default)      
  //xxxx1xxxb - Rising pulse edge
  Wire.write(0xff);
  Wire.endTransmission();

  // Set minimum duty cycle - Pulse Width Register (2Bh)
  Wire.beginTransmission(0x20);
  Wire.write(0x2b);
  //This register sets the pulse width of the PWM output. 
  //Allowed values are between zero and the (Period - 1) value.
  ////// this is the pulse width...0-255 where 255 is all "on" for one cycle (1/clock frequency)
  Wire.write(_iPwmVal);
  Wire.endTransmission();
}

//just set pwm period & pulse of the PWM output. //need 10ms
void setPwm(int _iPinNum, int _iPwmVal)
{
  if(_iPwmVal > 250) _iPwmVal = 250;
  // Select pin to write I2C commands to
  analogWrite(_iPinNum,1);            //analogWrite(pin, 1) will enable the PWM on the pin. 

  Wire.beginTransmission(0x20);
  Wire.write(0x2a);
  Wire.write(0xff);
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x2b);
  Wire.write(_iPwmVal);
  Wire.endTransmission();
}



/***** Kalman filter *****/
double x_est[6];
double p_est[36];
void kalmanfilter(double z[2], double y[2])
{
  int Q[36];
  int r2;
  double A[36];
  int r1;
  double x_prd[6];
  static const int b_A[36] = { 
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0,
    0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1   };

  int k;
  double p_prd[36];
  double a21;
  static const int B[36] = { 
    1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1   };

  double klm_gain[12];
  static const int c_A[12] = { 
    1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0   };

  double S[4];
  static const int b_B[12] = { 
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0   };

  static const long int R[4] = { 
    1000000000, 0, 0, 1000000000   };

  double c_B[12];
  double a22;
  double Y[12];
  double b_z[2];

  /*    Copyright 2010 The MathWorks, Inc. */
  /*  Initialize state transition matrix */
  /*      % [x  ] */
  /*      % [y  ] */
  /*      % [Vx] */
  /*      % [Vy] */
  /*      % [Ax] */
  /*  [Ay] */
  /*  Initialize measurement matrix */
  for (r2 = 0; r2 < 36; r2++) {
    Q[r2] = 0;
  }

  for (r1 = 0; r1 < 6; r1++) {
    Q[r1 + 6 * r1] = 1;

    /*  Initial state conditions */
    /*  Predicted state and covariance */
    x_prd[r1] = 0.0;
    for (r2 = 0; r2 < 6; r2++) {
      x_prd[r1] += (double)b_A[r1 + 6 * r2] * x_est[r2];
    }

    for (r2 = 0; r2 < 6; r2++) {
      A[r1 + 6 * r2] = 0.0;
      for (k = 0; k < 6; k++) {
        A[r1 + 6 * r2] += (double)b_A[r1 + 6 * k] * p_est[k + 6 * r2];
      }
    }
  }

  for (r2 = 0; r2 < 6; r2++) {
    for (k = 0; k < 6; k++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        a21 += A[r2 + 6 * r1] * (double)B[r1 + 6 * k];
      }

      p_prd[r2 + 6 * k] = a21 + (double)Q[r2 + 6 * k];
    }
  }

  /*  Estimation */
  for (r2 = 0; r2 < 2; r2++) {
    for (k = 0; k < 6; k++) {
      klm_gain[r2 + (k << 1)] = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        klm_gain[r2 + (k << 1)] += (double)c_A[r2 + (r1 << 1)] * p_prd[k + 6 *
          r1];
      }
    }
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (k = 0; k < 2; k++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        a21 += klm_gain[r2 + (r1 << 1)] * (double)b_B[r1 + 6 * k];
      }

      S[r2 + (k << 1)] = a21 + (double)R[r2 + (k << 1)];
    }
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (k = 0; k < 6; k++) {
      c_B[r2 + (k << 1)] = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        c_B[r2 + (k << 1)] += (double)c_A[r2 + (r1 << 1)] * p_prd[k + 6 * r1];
      }
    }
  }

  if (fabs(S[1]) > fabs(S[0])) {
    r1 = 1;
    r2 = 0;
  } 
  else {
    r1 = 0;
    r2 = 1;
  }

  a21 = S[r2] / S[r1];
  a22 = S[2 + r2] - a21 * S[2 + r1];
  for (k = 0; k < 6; k++) {
    Y[1 + (k << 1)] = (c_B[r2 + (k << 1)] - c_B[r1 + (k << 1)] * a21) / a22;
    Y[k << 1] = (c_B[r1 + (k << 1)] - Y[1 + (k << 1)] * S[2 + r1]) / S[r1];
  }

  for (r2 = 0; r2 < 2; r2++) {
    for (k = 0; k < 6; k++) {
      klm_gain[k + 6 * r2] = Y[r2 + (k << 1)];
    }
  }

  /*  Estimated state and covariance */
  for (r2 = 0; r2 < 2; r2++) {
    a21 = 0.0;
    for (k = 0; k < 6; k++) {
      a21 += (double)c_A[r2 + (k << 1)] * x_prd[k];
    }

    b_z[r2] = z[r2] - a21;
  }

  for (r2 = 0; r2 < 6; r2++) {
    a21 = 0.0;
    for (k = 0; k < 2; k++) {
      a21 += klm_gain[r2 + 6 * k] * b_z[k];
    }

    x_est[r2] = x_prd[r2] + a21;
  }

  for (r2 = 0; r2 < 6; r2++) {
    for (k = 0; k < 6; k++) {
      A[r2 + 6 * k] = 0.0;
      for (r1 = 0; r1 < 2; r1++) {
        A[r2 + 6 * k] += klm_gain[r2 + 6 * r1] * (double)c_A[r1 + (k << 1)];
      }
    }
  }

  for (r2 = 0; r2 < 6; r2++) {
    for (k = 0; k < 6; k++) {
      a21 = 0.0;
      for (r1 = 0; r1 < 6; r1++) {
        a21 += A[r2 + 6 * r1] * p_prd[r1 + 6 * k];
      }

      p_est[r2 + 6 * k] = p_prd[r2 + 6 * k] - a21;
    }
  }

  /*  Compute the estimated measurements */
  for (r2 = 0; r2 < 2; r2++) {
    y[r2] = 0.0;
    for (k = 0; k < 6; k++) {
      y[r2] += (double)c_A[r2 + (k << 1)] * x_est[k];
    }
  }

  /*  of the function */
}

void *thread_function(void *arg) {
  //get the new PID parameter from Serial.
  while(1){
    //check if data has been sent from Serial.
    if(Serial.available()) {
      read_data = Serial.read();
      if(isDigit(read_data)) {
        inString += (char)read_data;
      }
      else {
        if(inString != "") {
          //get int value from string .
          int_value[int_index++] = inString.toInt();
          if(int_index >= 3)
          {  
            int_index = 0;
            //Serial.println("..."); //for debug.
            Serial.println(int_value[0]);
            Serial.println(int_value[1]);
            Serial.println(int_value[2]);
            //Serial.println("...");  //for debug.
            change_PID = true;
          }
          inString = "";
        }
      }
    }
  }
}

void *button_function(void *arg) {
  while(1){
    key_value = analogRead(pin_key); //read analog input data from pin_key.

    if(key_value > 1000) // if value > 1000(4.88V), it means button pushed.
    {
      delay(10);  //delay some time for debounces.
      key_value = analogRead(pin_key);
      if(key_value > 1000)
        reverse_run_flag();
    }
    delay(100);
  }
}

void reverse_run_flag() // reverse the run_pid_flag.
{
  if(run_pid_flag == true) //true
  {
    run_pid_flag = false;
  }
  else //false
  {
    run_pid_flag = true;
  }
  reverse_flag = true;
}

