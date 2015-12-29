pthread_t mythread1;

void setup() {

  Serial.begin(9600);    //open Serial for debugging

  if(pthread_create(&mythread1, NULL, &thread_function, NULL))  //return 0 when success.
  {
    Serial.println("Error creating thread.\n");
  }

  hello("main()."); //use delay() to schedule pthread.

  if(pthread_join(mythread1, NULL)) //return 0 when success.
  {
    Serial.println("Error joining thread.\n");
  }
  else
    Serial.println("pthread_join() end.");
}

void loop() {
  // put your main code here, to run repeatedly: 

}

void *thread_function(void *arg) {

  hello("thread_function().");
  return NULL;
}

void hello(char *thread_name)
{
  int i;
  for(i = 0; i < 5; i++)
  {

    Serial.print("Thread id [");
    Serial.print((unsigned)pthread_self());
    Serial.print("]:");
    Serial.print("Hello, This is ");
    Serial.println(thread_name);
    delay(1000); 
  }
}

