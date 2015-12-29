void setup() {  

}  


void loop() {  
  delay(1000);  
  if(Serial) {  
    Serial.println("This is Serial, must be mapped to /dev/ttyGS0, i.e. visible in Arduino IDE's Serial Monitor");  
  }  
  else {  
    printf("Serial is not ready");  
  } 
  
  if(Serial1) {  
    Serial1.println("This is Serial1, must be mapped to /dev/ttyS0, i.e. visible on something connected to the Digital pins 0 & 1");  
  }  
  else {  
    printf("Serial1 is not ready");  
  }  
  
  if(Serial2) {  
    Serial2.println("This is Serial2, unused, you shouldn't see this anywhere");  
  }  
  else {  
    printf("Serial2 is not ready");  
  }   
}  
