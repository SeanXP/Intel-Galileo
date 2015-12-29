/*
  set a date for Intel Galileo. (system("/bin/date 1021222513"); or use popen(...) )
  then, read date from Galileo sometimes. (just use popen(...)).
*/

void setup()
{
  Serial.begin(9600);

  char *cmd = "/bin/date 1021222513"; // Oct. 21, 2013, 22:25
  char buf[64];
  FILE *ptr;

  if ((ptr = popen(cmd, "r")) != NULL) //set date by pipe shell.
  {
    while (fgets(buf, 64, ptr) != NULL)
    {
      Serial.print(buf);
    }
  }
  (void) pclose(ptr);
}

void loop()
{
  char *cmd = "/bin/date";
  char buf[64];
  FILE *ptr;

  if ((ptr = popen(cmd, "r")) != NULL)  //get date from linux.
  {
    while (fgets(buf, 64, ptr) != NULL)
    {
      Serial.print(buf);
    }
  }
  (void) pclose(ptr);
  
  delay(1000);
}
