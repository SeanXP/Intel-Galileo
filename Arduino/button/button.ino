
//button - A4
//Add pull-down resistor between PIN_KEY and GND.
//Add capacitance between VCC and PIN_KEY.

int pin_led = 13;
int pin_key = A4;

int value = 0;

void setup()
{
  pinMode(pin_led, OUTPUT);
}
void loop()
{
  value = analogRead(pin_key); //read analog input data from pin_key.
  
  if(value > 1000) // if value > 1000(4.88V), it means button pushed.
  {
    delay(30);  //delay some time for debounces.
    value = analogRead(pin_key);
    if(value > 1000)
      digitalWrite(pin_led, HIGH);
    else
      digitalWrite(pin_led,LOW);
  }
  else
    digitalWrite(pin_led,LOW); 
}
