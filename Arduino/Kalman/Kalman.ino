
#define SERIAL_DEBUG   //open the Serial for debugging...turn off by commenting this row.

#include "Wire.h"
#include <SCA60C.h>

int time_start = 0, time_end = 0;
#define TIME_S (time_start = millis())
#define TIME_E (time_end = millis(),Serial.print("Time:"),Serial.println(time_end - time_start))

//SCA60C module in <SCA60C.h>
//Vo1 -> A0
//Vo2 -> A1
SCA60C sca(A0, A1, 2.28, 2.30); // set the output offset voltage when horizontal. 
double angle_x = 0;
double angle_y = 0;

int counter = 0;

//-------------------------------------------------------------------------------//


void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);   
  Serial.println("Kalman filter test...");
#endif
}

double raw_data[2], kalman_data[2];
void loop()
{
  raw_data[0] = sca.GetAngleX();
  raw_data[1] = sca.GetAngleY();
  Serial.print(raw_data[0]);  //X axis.
  Serial.print(" ");
  kalmanfilter(raw_data, kalman_data);
  Serial.println(kalman_data[0]);
}

void test3()
{
  Serial.print("X:");
  Serial.print(sca.GetValueX());
  Serial.print(",");
  Serial.print(sca.GetVolX());
  Serial.print(",");
  Serial.print(sca.GetAngleX());
  Serial.print("; ");
  
  Serial.print("Y:");
  Serial.print(sca.GetValueY());
  Serial.print(",");
  Serial.print(sca.GetVolY());
  Serial.print(",");
  Serial.print(sca.GetAngleY());
  Serial.println("; ");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double x_est[6];
double p_est[36];

void kalmanfilter(double z[2], double y[2])
{
  int Q[36];
  int r2;
  double A[36];
  int r1;
  double x_prd[6];
  static const int b_A[36] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0,
    0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1 };

  int k;
  double p_prd[36];
  double a21;
  static const int B[36] = { 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  double klm_gain[12];
  static const int c_A[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  double S[4];
  static const int b_B[12] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const long int R[4] = { 1000000000, 0, 0, 1000000000 };

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
  } else {
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
