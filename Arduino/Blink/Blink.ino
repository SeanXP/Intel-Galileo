/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  switch_on_off(led, 1000);
  //pwm_on_off(led, 10);        //It doesn't work because pin13 is not a PWM pin in Intel Galileo! OMG!
}

void switch_on_off(int led, int delayms) //as a switch to control LED
{
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delayms);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(delayms);               // wait for a second
}

void pwm_on_off(int led, int delayms) 
{
  for(int i = 0; i < 256; i++)
  {
    analogWrite(led, i);
    
    delay(delayms); //advice : delayms = 100
  }
}
