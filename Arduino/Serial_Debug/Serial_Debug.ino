//get int value form Serial Input.

void setup() {
  Serial.begin(9600);
  
  delay(1000);
}

int read_data = 0;
int intvalue = 0;
String inString = "";

void loop() {
    if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    read_data = Serial.read();

    if(isDigit(read_data))
    {
      inString += (char)read_data;
    }
    else
    {
      if(inString != "")
      {
        intvalue = inString.toInt();
        Serial.println(intvalue);
        inString = "";
      }
      else
      {
        //Serial.println(read_data);
      }
    }
    
  }
}
