/*
  Electronic Speed Controller Control using the Arduino Servo Library
*/

#include <ESCs.h>


int potValue;  // value from the analog pin

void setup() {
  ESC_FrontLeft.attach(2, 1000, 2000);  // (PWN pin, 1000, 2000)
  ESC_FrontRight.attach(3, 1000, 2000);
  ESC_BackRight.attach(4, 1000, 2000);
  ESC_BackLeft.attach(5, 1000, 2000);
}

void loop() {
  potValue = analogRead(A0);  // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);  // scale it to use it with the servo library (value between 0 and 180)
  ESC_BackRight.write(potValue);  // send the signal to the ESC
}
