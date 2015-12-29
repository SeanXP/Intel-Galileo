/****************************************************************
            Copyright (C) 2014 All rights reserved.
					      									  
    > File Name:         < SCA60C.cpp >
    > Author:            < Shawn Guo >
    > Mail:              < iseanxp+code@gmail.com >
    > Created Time:      < 2014/06/01 >
    > Last Changed: 
    > Description:		Arduino Library (SCA60C) - Version 1.0.0

****************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <SCA60C.h>
#define RADIAN	57.292779524	// (180 / PI)

// constructor. links the SCA60C to the pin_x, pin_y. 
// Initial output voltage offset.
SCA60C::SCA60C(int pin_x, int pin_y, double v_offset_x, double v_offset_y)
{
	_pin_x = pin_x;
	_pin_y = pin_y;	

	SetOffsetX(v_offset_x);
	SetOffsetY(v_offset_y);
}

//return Arduino Analog Input. (0 ~ 1024)
int SCA60C::GetValueX()
{
	return (sensorValue_x = analogRead(_pin_x));
}

int SCA60C::GetValueY()
{
	return (sensorValue_y = analogRead(_pin_y));
}

//return output Voltage by calculating.
//0 	-> 	0(V)
//1024 	->	5(V)
double SCA60C::GetVolX()
{
	sensorValue_x = analogRead(_pin_x);
	//(value * 5.0 / 1024)
	return (voltage_x = (sensorValue_x / 204.8));
}	

double SCA60C::GetVolY()
{
	sensorValue_y = analogRead(_pin_y);
	return (voltage_y = (sensorValue_y / 204.8));
}

//return output angle.
double SCA60C::GetAngleX()
{
	static double X_tmp = 0;
	
	GetVolX();
	X_tmp = -asin(0.5 * (voltage_x - voltage_offset_x)) * RADIAN;

	//if X_tmp is 'nan', just discard this bad value.
	if(isnan(X_tmp))
		return angle_x;
	else
		return (angle_x = X_tmp);
}

double SCA60C::GetAngleY()
{
	static double Y_tmp = 0;
	
	GetVolY();
	Y_tmp = -asin(0.5 * (voltage_y - voltage_offset_y)) * RADIAN;

	//if Y_tmp is 'nan', just discard this bad value.
	if(isnan(Y_tmp))
		return angle_y;
	else
		return (angle_y = Y_tmp);

}

//set output voltage offset.
void SCA60C::SetOffsetX(double offset) { voltage_offset_x = offset; }
void SCA60C::SetOffsetY(double offset) { voltage_offset_y = offset; }
