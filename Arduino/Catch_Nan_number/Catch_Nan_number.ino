double a = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  a = asin(90);

  if(isnan(a))
  Serial.println("Catch you!");
  else 
  Serial.println("Nothing..");
  
  Serial.println(a);
}

void loop() {
  // put your main code here, to run repeatedly: 

}
