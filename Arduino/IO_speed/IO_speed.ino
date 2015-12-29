/****************
 * I/O speed in Galileo Arduino.
 * 
 * GPIO
 * Default max throghput is 230 Hz
 * IO2 and IO3 if configured as OUTPUT_FAST - will clock in at 477 kHz - to 2.93 MHz
 * digitalWrite() will achieve 477 kHz
 * fastGpioDigitalWrite() - an Intel extension of the Arduino API will achieve 680 kHz
 * fastGpioDigitalWriteDestructive - can achieve GPIO speeds of about 2.93 MHz
 * 
 ****************/

#define TEST_OUTPUT_MODE        // Default GPIO (230HZ)
//#define TEST_OUTPUT_FAST_MODE   // OUTPUT_FAST mode in (IO2 & IO3) , (477 HZ)
//#define TEST_OUTPUT_FAST_MODE2  // OUTPUT_FAST mode in (IO2 & IO3) , (680 kHZ)
//#define TEST_OUTPUT_MORE_FAST_MODE  //more fast mode in (IO2 & IO3), (2.93 MHZ)

uint32_t latchValue;

void setup() {

#ifdef TEST_OUTPUT_MODE
  pinMode(2, OUTPUT);  // 225HZ
#endif 

#ifndef TEST_OUTPUT_MODE
  pinMode(2, OUTPUT_FAST); // 477KHZ, IO2 fast output mode.
#endif

#ifdef TEST_OUTPUT_MORE_FAST_MODE
  latchValue = fastGpioDigitalLatch(); //just run it before test_morefast();
#endif
}



void loop() {
#ifdef TEST_OUTPUT_MODE
  test_digitalWrite();
#endif

#ifdef  TEST_OUTPUT_FAST_MODE
  test_digitalWrite();
#endif

#ifdef TEST_OUTPUT_FAST_MODE2
  test_fast();
#endif

#ifdef TEST_OUTPUT_MORE_FAST_MODE
  test_morefast();
#endif
}

//in OUTPUT(230HZ) mode or OUTPUT_FAST(477KHZ).
void test_digitalWrite()
{
  register int x = 0;
  while(1){
    digitalWrite(2, x);
    x =!x;
  }
}

// OUTPUT_FAST(680KHZ)
//  GPIO_FAST_IO2 , GPIO_FAST_IO3;
void test_fast()
{
  register int x = 0;
  while(1){
    //digitalWrite(3, x); // 477kHz , the same with IO2 fast output mode.

    fastGpioDigitalWrite(GPIO_FAST_IO2, x); //an Intel extension of the Arduino API will achieve 680 kHz
    x =!x;
  }
}

//OUTPUT_FAST MODE (2.9 MHZ)
//  GPIO_FAST_IO2 , GPIO_FAST_IO3;
void test_morefast()
{
  while(1){
    //fastGpioDigitalWriteDestructive - can achieve GPIO speeds of about 2.93 MHz
    fastGpioDigitalWriteDestructive(latchValue);
    latchValue ^= GPIO_FAST_IO2;
  }
}




