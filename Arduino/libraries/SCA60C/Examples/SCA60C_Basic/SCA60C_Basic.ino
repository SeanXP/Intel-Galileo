/********************************************************
 * SCA60C Basic Example
 * Reading analog input from SCA60C (vo1 & vo2), then calculate X,Y axis angle.
 
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

 ********************************************************/
#include <SCA60C.h>

SCA60C sca(A0, A1, 2.34, 2.47);

void setup() {
  Serial.begin(9600);
}

void loop() {
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