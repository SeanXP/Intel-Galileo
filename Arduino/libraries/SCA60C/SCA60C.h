/****************************************************************
            Copyright (C) 2014 All rights reserved.
					      									  
    > File Name:         < SCA60C.h >
    > Author:            < Shawn Guo >
    > Mail:              < iseanxp+code@gmail.com >
    > Created Time:      < 2014/06/01 >
    > Last Changed: 
    > Description:		A library for Arduino. (SCA60C)

  SCA60C( N1000060 accelerometer) 
   
        (Vo2 output pin)
       
              UP
              ^
              |
              |  
LEFT <--------|--------- RIGHT
              |
              |
             DOWN

       (Vo1 output pin)


(0.5~4.5V) (-90~90 degrees)
Vo1: left-right   (x axis), LEFT is X-axis positive direction.
Vo2: up-down      (y axis), UP	 is Y-axis positive direction.


****************************************************************/


#ifndef __SCA60C__
#define __SCA60C__

#define SCA60C_LIB_VERSION	1.0.0	//define library version

class SCA60C
{
	public:
		// constructor. links the SCA60C to the pin_x, pin_y. 
		// Initial output voltage offset.
		SCA60C(int pin_x, int pin_y, double v_offset_x, double v_offset_y);
		
		int GetValueX();	//return Arduino Analog Input. (0 ~ 1024)
		int GetValueY();

		double GetVolX();	//return output Voltage by calculating.
		double GetVolY();

		double GetAngleX();	//return output Angle.
		double GetAngleY();

		//set output voltage offset.
		void SetOffsetX(double offset);
		void SetOffsetY(double offset);

	private:
		//Connect Arduino Analog Input Pin. (A0 ~ A5)
		int _pin_x;		// left-right axis
		int _pin_y;		// up - down  axis

		//Arduino Analog input data; (0~1024)
		int sensorValue_x;
		int sensorValue_y;

		//SCA60C output voltage (calculate from Arduino Analog input data)
		//0.5~4.5 (V)
		double voltage_x;
		double voltage_y;

		//SCA60C output voltage offset
		double voltage_offset_x;
		double voltage_offset_y;

		//output angle (calculate from voltage)
		double angle_x;
		double angle_y;
};

#endif	/* SCA60C library */

